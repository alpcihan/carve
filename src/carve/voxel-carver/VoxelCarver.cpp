#include "carve/voxel-carver/VoxelCarver.h"
#include "carve/utils/segment.h"

#define CARVED 0
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
   
    void VoxelCarver::saveCurrentStateAsPLY(const std::string& fileName) const
    {
        CRV_INFO("Writing carved point cloud into: " << fileName << "...");

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

                    if (m_space[i1D] == CARVED)
                    {
                        continue;
                    }

                    float dim = (float)m_params.voxelSpaceDim;
                    file << (i / dim)-0.5f << " " << (j / dim)-0.5f << " " << (k / dim)-0.5f << "\n"; // TODO: fix the precision
                }
            }
        }

        CRV_INFO("Saved the carved point cloud into " << fileName);

        file.close();
    }

    void VoxelCarver::_initVoxelSpace()
    {
        uint32_t size = m_params.voxelSpaceDim * m_params.voxelSpaceDim * m_params.voxelSpaceDim;
        m_space = std::vector<float>(size, -1);
        m_pointCount = size;

        CRV_INFO("Initialized a voxel space with the dimensions: " << m_params.voxelSpaceDim << " " << m_params.voxelSpaceDim << " " << m_params.voxelSpaceDim);
    }

    void VoxelCarver::_carveByBinaryImage(const cv::Mat& binaryImage, const cv::Matx33d& intrinsics, const cv::Mat& distCoeffs, const cv::Vec3d& rVec, const cv::Vec3d& tVec)
    {
        CRV_INFO("Carving... (This might take some time)");

        double s = m_params.volumeScale;
        double sHalf = s / 2;
        double off = s / m_params.voxelSpaceDim;
        uint32_t dim = m_params.voxelSpaceDim;
        
        cv::Matx33d rot;
        cv::Rodrigues(rVec, rot);

        for (uint32_t i = 0; i < dim; i++)
        {
            double x = (i * off) - sHalf;
            for (uint32_t j = 0; j < dim; j++)
            {
                double y = (j * off) - sHalf;
                for (uint32_t k = 0; k < dim; k++)
                {
                    double z = (k * off) - s;

                    uint32_t i1D = (k * dim + j) * dim + i;
                    
                    auto a = intrinsics * (rot*cv::Vec3d(x, y, z) + tVec);
                    a = a / a[2];

                    uint32_t u = a[0];
                    uint32_t v = a[1];

                    // continue if the voxel is already carved
                    if (m_space[i1D] == CARVED)
                    {
                        continue;
                    }

                    // carve the voxel if it's 2D projection is outside the camera's screen space 
                    if (u < 0 || v < 0 || u >= binaryImage.cols || v >= binaryImage.rows)
                    {
                        m_pointCount--;
                        m_space[i1D] = CARVED;
                        continue;
                    }

                    // carve the voxel if it's 2D projection corresponds to a black pixel at the camera's screen space  
                    if (binaryImage.at<uchar>(int(v), int(u)) == 0)
                    {
                        m_pointCount--;
                        m_space[i1D] = CARVED;
                        continue;
                    }


                    m_space[i1D] = -((a[0] - u) + (a[1] - v));
                }
            }
        } 

        CRV_INFO("Carved: " << m_space.size() - m_pointCount << " points in total.");
    }
}