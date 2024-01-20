#include <iostream>
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <opencv2/highgui/highgui_c.h>


int main(int argc, char * argv[]) try
{
    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Start streaming with default recommended configuration
    pipe.start();

    using namespace cv;
    using namespace std;

    const auto window_name = "Display Image";
    namedWindow(window_name, WINDOW_AUTOSIZE);

    while (waitKey(1) < 0 && cvGetWindowHandle(window_name))
    {
        rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera

        rs2::frame depth = data.get_depth_frame().apply_filter(color_map);
        rs2::depth_frame depth_point = data.get_depth_frame();

        rs2::frame color = data.get_color_frame().apply_filter(color_map);

        // Query frame size (width and height)
        const int w_d = depth.as<rs2::video_frame>().get_width();
        const int h_d = depth.as<rs2::video_frame>().get_height();

        const int w_c = color.as<rs2::video_frame>().get_width();
        const int h_c = color.as<rs2::video_frame>().get_height();

        // Create OpenCV matrix of size (w,h) from the colorized depth data
        Mat image(Size(w_c, h_c), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);

// デバッグ
        Mat gray;
        cvtColor(image, gray, COLOR_BGR2GRAY);
        medianBlur(gray, gray, 5);  // メディアンフィルタでの平滑化

        std::vector<Vec3f> circles;

        HoughCircles(gray, circles, HOUGH_GRADIENT, 1,
            gray.rows/16,  // change this value to detect circles with different distances to each other
            100,    // エッジ検出器に渡される大きい方の閾値 100
            30,     // -> 小さい方の閾値 30
            1,  // 円の半径の最小値 1
            60  // ->最大値 30
        );

        int count = 0;
        for (auto circle : circles){
            const double x = circle[0], y = circle[1], radius = circle[2];

            // 深度フレームとカラーフレームの比率 -> 深度取得
            const double w_ratio = static_cast<double>(w_d)/w_c, h_ratio = static_cast<double>(h_d)/h_c;
            const double distance = depth_point.get_distance(x*w_ratio, y*h_ratio);

            const std::string text = format("%d:%.2f", count, distance);
            cv::putText(image, text, Point(x,y), FONT_HERSHEY_SIMPLEX, 1, Scalar(255,255,255), 3);

            cv::circle(image, Point(x,y), radius, Scalar(0, 0, 255), 2);
            count++;
        }


// ここまで
        //画像の真ん中のxy座標の距離を取得
        const double pixel_distance_in_meters = depth_point.get_distance(w_d/2,h_d/2);
        // std::cout << pixel_distance_in_meters  << std::endl;

        // Update the window with new data
        imshow(window_name, image);
    }

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
