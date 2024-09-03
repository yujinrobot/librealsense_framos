// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp>     // Include RealSense Cross Platform API
#include "example.hpp"              // Include short list of convenience functions for rendering

int main(int argc, char * argv[]) try
{
    // Create librealsense context for managing devices
    rs2::context ctx; 

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;

    // The config allows pipeline users to request filters for the pipeline streams and device selection and configuration
    rs2::config cfg;

    // Declare rates printer for showing streaming rates of the enabled streams
    rs2::rates_printer printer;

    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;

    // Camera serial for enabling found camera
    std::string camera_serial = "";

    bool is_camera_found = false;

    // Find first available D415e or D455e camera
    for (auto&& dev : ctx.query_devices()) {
        std::string camera_name = dev.get_info(RS2_CAMERA_INFO_NAME);

        if (camera_name.find("D415e") != std::string::npos || camera_name.find("D455e") != std::string::npos) {
            camera_serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);

            is_camera_found = true;
            break;
        }
    }

    // Exit if D415e or D455e is not found
    if (!is_camera_found) {
        std::cout << "Please connect D415e or D455e camera!\n";
        return EXIT_FAILURE;
    }

    // Enable D415e or D455 camera
    cfg.enable_device(camera_serial);

    // Enable depth, color from left imager and color streams
    cfg.enable_stream(RS2_STREAM_DEPTH, -1, 640, 480, RS2_FORMAT_Z16, 30);
    // Enable color from left imager
    cfg.enable_stream(RS2_STREAM_INFRARED, -1, 640, 480, RS2_FORMAT_RGB8, 30);
    cfg.enable_stream(RS2_STREAM_COLOR, -1, 640, 480, RS2_FORMAT_RGB8, 30);

    // Start streaming with given configuration
    pipe.start(cfg);

    // Create a simple OpenGL window for rendering:
    window app(1280, 960, "CPP Color-Left-Imager Example");

    while (app) // Application still alive?
    {
        rs2::frameset data = pipe.wait_for_frames().    // Wait for next set of frames from the camera
                             apply_filter(printer).     // Print each enabled stream frame rate
                             apply_filter(color_map);   // Find and colorize the depth data
                               

        // Present collected frames with openGl mosaic
        app.show(data);
    }

    // Stop the pipeline streaming after window is closed
    pipe.stop();

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}