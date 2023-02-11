#include "carve/voxel-carver/VoxelCarver.h"
#include "carve/utils/segment.h"


#include "carve/ex2utils/Eigen.h"
#include "carve/ex2utils/ImplicitSurface.h"
#include "carve/ex2utils/Volume.h"
#include "carve/ex2utils/MarchingCubes.h"




namespace crv
{
    VoxelCarver::VoxelCarver(const VoxelCarverParams &voxelCarverParams)
        : m_params(voxelCarverParams)
    {
        _initVoxelSpace();
    }

    void VoxelCarver::carveByBinaryImage(const cv::Mat& binaryImage, const cv::Matx33d& intrinsics, const cv::Mat& distCoeffs, const cv::Vec3d& rVec, const cv::Vec3d& tVec)
    {
        _carveByBinaryImage(binaryImage, intrinsics, distCoeffs, rVec, tVec);
    }

    void VoxelCarver::saveCurrentStateAsPLY(const std::string& fileName)
    {
        CRV_INFO("Writing carved point cloud into \"output.ply\"...");

        std::ofstream file(CRV_RELATIVE(fileName));

        file << "ply\nformat ascii 1.0\nelement vertex " << m_pointCount << "\nproperty float x\nproperty float y\nproperty float z\nend_header\n";

        uint32_t size = m_params.voxelSpaceDim;

        for (int i = 0; i < m_params.voxelSpaceDim; i++)
        {
            for (int j = 0; j < m_params.voxelSpaceDim; j++)
            {
                for (int k = 0; k < m_params.voxelSpaceDim; k++)
                {
                    uint32_t i1D = (k * size + j) * size + i;


                    uint sum =0;
                    for(int xx=-1;xx<2;xx++) {
                        if (i==0 || i==size-1) continue;
                        for(int yy=-1;yy<2;yy++) {
                            if (j==0 || j==size-1) continue;
                            for(int zz=-1;zz<2;zz++) {
                                if (k==0 || k==size-1) continue;
                                sum+=m_space[((k+zz) * size + j+yy) * size + i+xx];
                            }
                        }
                    }


                    if (!m_space[i1D] || sum>20)
                    {
                        continue;
                    }

                    float dim = (float)m_params.voxelSpaceDim;
                    file << i / dim << " " << j / dim << " " << k / dim << "\n";
                }
            }
        }

        CRV_INFO("Saved the carved point cloud into \"output.ply\".");

        file.close();
    }


    void VoxelCarver::marche(const std::string& fileName){

        // implicit surface
        ImplicitSurface* surface;
        //surface = new RBF(m_space);
        surface = new Hoppe(m_space);
        
        CRV_INFO("Filling Volume -----------------");
        // fill volume with signed distance values
        unsigned int mc_res = 100; //m_params.voxelSpaceDim; // resolution of the grid, for debugging you can reduce the resolution (-> faster)
        Volume vol(Vector3d(-0.1,-0.1,-0.1), Vector3d(1.1,1.1,1.1), mc_res, mc_res, mc_res, 1);
        for (unsigned int x = 0; x < vol.getDimX(); x++)
        {
            CRV_INFO("Filling slice: "<<x)
            for (unsigned int y = 0; y < vol.getDimY(); y++)
            {
                for (unsigned int z = 0; z < vol.getDimZ(); z++)
                {
                    Eigen::Vector3d p = vol.pos(x, y, z);
                    double val = surface->Eval(p);
                    vol.set(x,y,z, val);
                }

            }
        }

        CRV_INFO("Filling DONE -----------------");

    

        // extract the zero iso-surface using marching cubes
        SimpleMesh mesh;
        for (unsigned int x = 0; x < vol.getDimX() - 1; x++)
        {
            std::cerr << "Marching Cubes on slice " << x << " of " << vol.getDimX() << std::endl;

            for (unsigned int y = 0; y < vol.getDimY() - 1; y++)
            {
                for (unsigned int z = 0; z < vol.getDimZ() - 1; z++)
                {
                    ProcessVolumeCell(&vol, x, y, z, 0.00f, &mesh);
                }
            }
        }

        // write mesh to file
        if (!mesh.WriteMesh(fileName))
        {
            std::cout << "ERROR: unable to write output file!" << std::endl;
            return ;
        }

        CRV_INFO("DONEEEE")


    }










    void VoxelCarver::_initVoxelSpace()
    {
        uint32_t size = m_params.voxelSpaceDim * m_params.voxelSpaceDim * m_params.voxelSpaceDim;
        m_space = std::vector<bool>(size,  true);
        m_pointCount = size;

        CRV_INFO("Initialized a voxel space with the dimensions: " << m_params.voxelSpaceDim << " " << m_params.voxelSpaceDim << " " << m_params.voxelSpaceDim);
    }

    void VoxelCarver::_carveByBinaryImage(const cv::Mat& binaryImage, const cv::Matx33d& intrinsics, const cv::Mat& distCoeffs, const cv::Vec3d& rVec, const cv::Vec3d& tVec)
    {
        CRV_INFO("Carving... (This might take some time)");

        // voxel to image solution
        double s = m_params.volumeScale;
        double shalf = s / 2;
        double off = s / m_params.voxelSpaceDim;
        uint32_t dim = m_params.voxelSpaceDim;
        
        cv::Matx33d rot;
        cv::Rodrigues(rVec, rot);

        for (double i = 0; i < dim; i++)
        {
            double x = (i * off) - shalf;
            for (double j = 0; j < dim; j++)
            {
                double y = (j * off) - shalf;
                for (double k = 0; k < dim; k++)
                {
                    double z = (k * off) - s;

                    uint32_t i1D = (k * dim + j) * dim + i;
                    
                    auto a = intrinsics * (rot*cv::Vec3d(x, y, z) + tVec);
                    a = a / a[2];

                    uint32_t u = a[0];
                    uint32_t v = a[1];

                    if (m_space[i1D] == false)
                    {
                        continue;
                    }

                    if (u < 0 || v < 0 || u >= binaryImage.cols || v >= binaryImage.rows)
                    {
                        m_pointCount--;
                        m_space[i1D] = false;
                        continue;
                    }

                    if (binaryImage.at<uchar>(int(v), int(u)) != 0)
                    {
                        continue;
                    }

                    m_pointCount--;
                    m_space[i1D] = false;
                }
            }
        }

        CRV_INFO("Carved: " << m_space.size() - m_pointCount << " points in total.");
    }
}