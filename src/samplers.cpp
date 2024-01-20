#include <iostream>
#include <librealsense2/rs.hpp>

int main()
try {

	rs2::pipeline pipe;

	pipe.start();

	while (true)
	{
		rs2::frameset frames = pipe.wait_for_frames();

		rs2::depth_frame depth = frames.get_depth_frame();

		auto width = depth.get_width();
		auto height = depth.get_height();

		float distance_to_center = depth.get_distance(width / 2, height / 2);

		std::cout<< "The camera is facing an object " << distance_to_center << " meters away \r";
	}

	return EXIT_SUCCESS;

}
catch (const rs2::error& e) {
	std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
	return EXIT_FAILURE;
}
catch (const std::exception& e) {
	std::cerr << e.what() << std::endl;
	return EXIT_FAILURE;
}
