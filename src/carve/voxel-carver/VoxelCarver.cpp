#include "carve/voxel-carver/VoxelCarver.h"
#include "carve/utils/segment.h"

namespace crv
{
    VoxelCarver::VoxelCarver(const VoxelCarverParams &voxelCarverParams)
        : m_params(voxelCarverParams)
    {
        _initVoxelSpace();
    }

    void VoxelCarver::run()
    {
        cv::Mat frame, masked;
        calib::CamToBoardData camToBoardData;
        m_sceneVideo = std::make_unique<crv::VideoReader>(m_params.sceneVideoSource, m_params.sceneVideoSkipBy); // set space carving scene video

        _calibCamera(); // calibrate the camera from the calibration video
        
        while (m_sceneVideo->getNextFrame(frame))
        {
            segment::evaluateObjectOnlyImage(frame, masked);

            calib::estimateCamToARBoard(frame, m_cam, camToBoardData);

            _carveSegmentedImage(masked, camToBoardData);
        }

        CRV_INFO("Carving has finished. (" << m_sceneVideo->getFileName() << ") Remaining point count: " << m_pointCount)
    }

    void VoxelCarver::saveAsPLY(const std::string& fileName)
    {
        CRV_INFO("Writing carved point cloud (" << m_sceneVideo->getFileName() << ") into \"output.ply\"...");

        std::ofstream file(CRV_RELATIVE(fileName));

        file << "ply\nformat ascii 1.0\nelement vertex " << m_pointCount << "\nproperty float x\nproperty float y\nproperty float z\nend_header\n";

        for (int i = 0; i < m_params.voxelSpaceDim; i++)
        {
            for (int j = 0; j < m_params.voxelSpaceDim; j++)
            {
                for (int k = 0; k < m_params.voxelSpaceDim; k++)
                {
                    if (!m_space[i][j][k])
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

    void VoxelCarver::_initVoxelSpace()
    {
        uint32_t dim = m_params.voxelSpaceDim;
        m_space = std::vector<std::vector<std::vector<bool>>>(dim, std::vector<std::vector<bool>>(dim, std::vector<bool>(dim, true)));
        m_pointCount = dim * dim * dim;
    }

    void VoxelCarver::_calibCamera()
    {
        auto calibVideo = VideoReader(m_params.calibVideoSource, m_params.calibVideoSkipBy); // set the calibration video
        calib::estimateCamMatrixAndDistortion(calibVideo, m_params.checkerBoardDims, m_cam);
    }

    void VoxelCarver::_carveSegmentedImage(const cv::Mat &segmentedImage, const calib::CamToBoardData &camToBoardData)
    {
        CRV_INFO("Carving... (This might take some time)");

        // voxel to image solution
        double s = m_params.volumeScale;
        double off = s / m_params.voxelSpaceDim;
        uint32_t dim = m_params.voxelSpaceDim * m_params.voxelSpaceDim * m_params.voxelSpaceDim;

        static std::vector<cv::Point3d> samples = std::vector<cv::Point3d>(dim);

        for (double i = 0; i < m_params.voxelSpaceDim; i++)
        {
            double x = (i * off) - s * 0.5;
            for (double j = 0; j < m_params.voxelSpaceDim; j++)
            {
                double y = (j * off) - s * 0.5;
                for (double k = 0; k < m_params.voxelSpaceDim; k++)
                {
                    double z = (k * off) - s;

                    samples.at((k * m_params.voxelSpaceDim + j) * m_params.voxelSpaceDim + i) = std::move(cv::Point3d(x, y, z));
                }
            }
        }

        std::vector<cv::Point2d> projected;
        cv::projectPoints(samples, camToBoardData.rVec, camToBoardData.tVec, m_cam.cameraMatrix, m_cam.distCoeffs, projected);

        for (int i = 0; i < dim; i++)
        {
            int i1D = i;
            int z = i1D / (m_params.voxelSpaceDim * m_params.voxelSpaceDim);
            i1D -= z * m_params.voxelSpaceDim * m_params.voxelSpaceDim;
            int y = i1D / m_params.voxelSpaceDim;
            int x = i1D % m_params.voxelSpaceDim;

            if (m_space[x][y][z] == false)
            {
                continue;
            }

            if (projected[i].x < 0 || projected[i].y < 0 || projected[i].x >= segmentedImage.cols || projected[i].y >= segmentedImage.rows)
            {
                m_pointCount--;
                m_space[x][y][z] = false;
                continue;
            }

            if (segmentedImage.at<uchar>(int(projected[i].y), int(projected[i].x)) != 0)
            {
                continue;
            }

            m_pointCount--;
            m_space[x][y][z] = false;
        }

        CRV_INFO("Carved: " << dim - m_pointCount << " points in total.");
    }
}