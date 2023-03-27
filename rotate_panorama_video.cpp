#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

// createLUT 和 rotatePanoramaWithLUT 函数
void createLUT(cv::Mat &mapX, cv::Mat &mapY, int width, int height, double pitch, double yaw)
{
    double centerX = width / 2.0, centerY = height / 2.0;
    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            double theta = (2 * M_PI * (x + yaw)) / width - M_PI;
            double phi = (M_PI * (y + pitch)) / height - M_PI / 2;
            double xPos = cos(phi) * cos(theta) * centerX + centerX;
            double yPos = cos(phi) * sin(theta) * centerY + centerY;
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

    double pitch = -90.0, yaw = 180.0;
    cv::Mat mapX = cv::Mat(height, width, CV_32FC1);
    cv::Mat mapY = cv::Mat(height, width, CV_32FC1);
    createLUT(mapX, mapY, width, height, pitch, yaw);

    std::string outputVideoPath = "/home/goslam/Documents/RS全景镜头/全景_VID_20230328_013007_out.mp4";
    cv::VideoWriter videoWriter(outputVideoPath, cv::VideoWriter::fourcc('M', 'P', '4', 'V'), fps, cv::Size(width, height));

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
