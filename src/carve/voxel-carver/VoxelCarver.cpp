#include "carve/voxel-carver/VoxelCarver.h"
#include "carve/utils/segment.h"

namespace crv
{
    VoxelCarver::VoxelCarver(const VoxelCarverParams &voxelCarverParams)
        : m_params(voxelCarverParams)
    {
        _initVoxelSpace();
    }

    void VoxelCarver::carveByBinaryMask(const cv::Mat& binaryImage, const cv::Matx33d& intrinsics, const cv::Mat& distCoeffs, const cv::Vec3d& rVec, const cv::Vec3d& tVec)
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

                    auto a = intrinsics * (rot * cv::Vec3d(x, y, z) + tVec);
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
   
    void VoxelCarver::saveCurrentStateAsPLY(const std::string& fileName) const
    {
        CRV_INFO("Writing carved point cloud into: " << fileName << "...");

        std::ofstream file(fileName);

        file << "ply\nformat ascii 1.0\nelement vertex " << m_pointCount << "\nproperty float x\nproperty float y\nproperty float z\nend_header\n";

        uint32_t size = m_params.voxelSpaceDim;

        for (int i = 0; i < m_params.voxelSpaceDim; i++)
        {
            for (int j = 0; j < m_params.voxelSpaceDim; j++)
            {
                for (int k = 0; k < m_params.voxelSpaceDim; k++)
                {
                    uint32_t i1D = (k * size + j) * size + i;

                    if (m_space[i1D] == false)
                    {
                        continue;
                    }

                    float dim = (float)m_params.voxelSpaceDim;
                    file << i / dim << " " << j / dim<< " " << k / dim << "\n"; // TODO: fix the precision
                }
            }
        }

        CRV_INFO("Saved the carved point cloud into " << fileName);

        file.close();
    }

    void VoxelCarver::_initVoxelSpace()
    {
        uint32_t size = m_params.voxelSpaceDim * m_params.voxelSpaceDim * m_params.voxelSpaceDim;
        m_space = std::vector<bool>(size, true);
        m_pointCount = size;

        CRV_INFO("Initialized a voxel space with the dimensions: " << m_params.voxelSpaceDim << " " << m_params.voxelSpaceDim << " " << m_params.voxelSpaceDim);
    }
}