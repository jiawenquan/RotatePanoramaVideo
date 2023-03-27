#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/opencv.hpp>

// createLUT 和 rotatePanoramaWithLUT 函数
void createLUT(cv::Mat &mapX, cv::Mat &mapY, int width, int height, double pitch, double yaw, double roll)
{
    double centerX = width / 2.0;
    double centerY = height / 2.0;
    double radius = width / (2 * M_PI);

    cv::Mat rotMat = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat rotMatX, rotMatY, rotMatZ;

    cv::Mat xAxis = (cv::Mat_<double>(3, 1) << 1, 0, 0);
    cv::Mat yAxis = (cv::Mat_<double>(3, 1) << 0, 1, 0);
    cv::Mat zAxis = (cv::Mat_<double>(3, 1) << 0, 0, 1);

    cv::Rodrigues(xAxis * pitch * M_PI / 180.0, rotMatX);
    cv::Rodrigues(yAxis * yaw * M_PI / 180.0, rotMatY);
    cv::Rodrigues(zAxis * roll * M_PI / 180.0, rotMatZ);

    rotMat = rotMatZ * rotMatY * rotMatX;

    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            double theta = (2 * M_PI * x) / width - M_PI;
            double phi = (M_PI * y) / height - M_PI / 2;
            cv::Mat point3D = (cv::Mat_<double>(3, 1) << radius * cos(phi) * cos(theta), radius * sin(phi), radius * cos(phi) * sin(theta));
            cv::Mat rotatedPoint3D = rotMat * point3D;

            double xPos = (atan2(rotatedPoint3D.at<double>(0), rotatedPoint3D.at<double>(2)) + M_PI) * width / (2 * M_PI);
            double yPos = (asin(rotatedPoint3D.at<double>(1) / radius) + M_PI / 2) * height / M_PI;
            mapX.at<float>(y, x) = static_cast<float>(xPos);
            mapY.at<float>(y, x) = static_cast<float>(yPos);
        }
    }
}
void rotatePanoramaWithLUT(const cv::Mat &input, cv::Mat &output, const cv::Mat &mapX, const cv::Mat &mapY)
{

    cv::remap(input, output, mapX, mapY, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
}

int main()
{
    std::string videoPath = "/home/goslam/Documents/RS全景镜头/全景_VID_20230328_013007.mp4";
    cv::VideoCapture videoCapture(videoPath);
    if (!videoCapture.isOpened())
        return -1;

    int width = static_cast<int>(videoCapture.get(cv::CAP_PROP_FRAME_WIDTH));
    int height = static_cast<int>(videoCapture.get(cv::CAP_PROP_FRAME_HEIGHT));
    double fps = videoCapture.get(cv::CAP_PROP_FPS);

    double pitch = 0.0, yaw = 90.0, roll = 90.0;
    cv::Mat mapX = cv::Mat(height, width, CV_32FC1);
    cv::Mat mapY = cv::Mat(height, width, CV_32FC1);
    createLUT(mapX, mapY, width, height, pitch, yaw, roll);

    std::string outputVideoPath = "/home/goslam/Documents/RS全景镜头/全景_VID_20230328_013007_out.mp4";
    cv::VideoWriter videoWriter(outputVideoPath, cv::VideoWriter::fourcc('a', 'v', 'c', '1'), fps, cv::Size(width, height));

    if (!videoWriter.isOpened())
        return -1;

    cv::Mat frame, rotatedFrame;
    while (videoCapture.read(frame))
    {
        rotatePanoramaWithLUT(frame, rotatedFrame, mapX, mapY);
        videoWriter.write(rotatedFrame);
    }

    videoCapture.release();
    videoWriter.release();

    return 0;
}
