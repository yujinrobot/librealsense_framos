// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.

//
// Textual overview of each api extension is described in d400e_api_extensions.md document.
//

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <iostream>             // for cout
#include <string.h> 

void device_filtering(int argc, const char** argv);
void port_range_example();
void buffer_count_example();
void heartbeat_time_example();
void device_diagnostics_example(rs2::device& dev, const char* d400e_serial_number = "");
void camera_information_example(rs2::device& dev);

int main(int argc, const char** argv) try
{
    // Device Filetering, Port Range, Heartbeat Time and Buffer Count api extensions affect the Framos GigEVision driver and are shared among all D400e cameras
    device_filtering(argc, argv);
    port_range_example();
    heartbeat_time_example();
    buffer_count_example();

    // Device Diagnostics and Camera Information extensions affect only the chosen device, hence creating a context and chosing the device is needed
    rs2::context ctx = rs2::context();
    rs2::device dev = ctx.query_devices()[0];
    device_diagnostics_example(dev, "6CD146030022");
    camera_information_example(dev);

    std::cout << "\nEnter a key to terminate" << std::endl;
    getchar();
    return EXIT_SUCCESS;
}
catch (const rs2::error& e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}

// Filtering file location can be passed to the application through command line arguments, with argument flag dev_filter (e.g. app_name.exe dev_filter=fliter_file_path.json)
// If no argument is passed, meaning that no filter file is defined all devicess will be available
// Filter file syntax can be checked in d400e_api_extensions.md under topic "Device filtering by serial number or/and IP address"
void device_filtering(int argc, const char** argv) {
    rs2_cli_arg carg;
    carg.argc = argc;
    carg.argv = argv;
    rs2_d400e_set_cli_args(&carg, nullptr);
}


// Preferred port range can be set using the example bellow
void port_range_example()
{
    std::cout << "Ports:" << std::endl;
    // Set port range to be used by CameraSuite
    rs2::d400e::set_port_range({ 50000, 65000 });

    // Get used port range
    rs2::d400e::port_range port_range = rs2::d400e::get_port_range();
    std::cout << "Port Range is: " << port_range.min << " to " << port_range.max << std::endl;

    // Querry all open ports used by CameraSuite (with no argument, default method)
    // Ports can be queried by type when rs2_d400e_port_type is passed as argument to  rs2::d400e::query_ports() method    
    //  - RS2_D400E_PORT_TYPE_ALL - returns all used ports, identicall to no arguments being passed
    //  - RS2_D400E_PORT_TYPE_CONTROL - returns only ports used to transfer camera controll signals 
    //  - RS2_D400E_PORT_TYPE_STREAM - returns only ports used to transfer camera streams
    //  - RS2_D400E_PORT_TYPE_MESSAGE - reeturns only ports used to transfer camera messages
    std::vector<uint16_t> all_ports = rs2::d400e::query_ports();

    std::cout << "Ports used by CameraSuite:" << std::endl;
    for (auto port : all_ports)
    {
        std::cout << port << std::endl;
    }
    std::cout << std::endl;
}


// Images acquired from D400e cameras are stored in circular buffer inside the driver
void buffer_count_example()
{
    std::cout << "Buffer count:" << std::endl;
    // Get current buffer count
    int buffer_count = rs2::d400e::get_buffer_count();
    std::cout << "Current buffer count is: " << buffer_count << "." << std::endl;
    int buffer_count_new = 20;
    std::cout << "Setting buffer count to: " << buffer_count_new << "." << std::endl;
    rs2::d400e::set_buffer_count(buffer_count_new);
    // Set default buffer count 
    std::cout << "Returning buffer count to: " << buffer_count << "." << std::endl;
    rs2::d400e::set_buffer_count(buffer_count);
    std::cout << std::endl;
}


// Heartbeat mechanism is used to detect disconnect between the host and a D400e camera.
// Setting the Heartbeat time is usefull if more frequent camera disconnect checks are required or during long debug sessions to prevent unnecessary camera disconnect.
void heartbeat_time_example()
{

    std::string dev_serial_callback = "";
    std::cout << "Heartbeat time:" << std::endl;
    float heartbeat_time_default = rs2::d400e::get_heartbeat_time();
    std::cout << "Default heartbeat time is: " << heartbeat_time_default << " s, which causes camera Heartbeat Timeout every " << 4 * heartbeat_time_default << " s, (Heartbeat time *4)." << std::endl;
    float heartbeat_time_new = 1.0;
    rs2::d400e::set_heartbeat_time(heartbeat_time_new);
    std::cout << "Setting Heartbeat Time to " << heartbeat_time_new << " s." << std::endl;
    rs2::d400e::set_heartbeat_time(heartbeat_time_default);
    std::cout << "Setting Heartbeat Time to default again." << std::endl;
    std::cout << std::endl;
}


// Enable or disable diagnostic packets sent by D400e series devices.
void device_diagnostics_example(rs2::device& dev, const char* d400e_serial_number)
{
    std::cout << "Device Diagnostics:" << std::endl;
    std::cout << "Device diagnostics are used for troubleshooting and can be viewed via wireshark." << std::endl;
    const char* dev_serial_number = NULL;

    // In case no serial number was given seral number of the first (only connected device is read).
    // If the serial number of the camera which diagnostics need to be toogled is known then context doesn't need to be initiated.   
    if (strcmp(d400e_serial_number, "") == 0)
    {
        // Toggle device diagnostics ON
        if (!rs2::d400e::toggle_device_diagnostics(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER), 1))
        {
            std::cout << "Device (SN: " << dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << ") diagnostics are turned ON." << std::endl;

        }
        // Toggle device diagnostics OFF
        if (!rs2::d400e::toggle_device_diagnostics(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER), 0))
        {
            std::cout << "Device (SN: " << dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << ") diagnostics are turned OFF." << std::endl;
        }
    }
    else
    {
        //NOTE: Querry device needs to be called to find available devices even in case the Serial Number of the device for which Device Diagnostics is togled is known.
        // Toggle device diagnostics ON        
        if (!rs2::d400e::toggle_device_diagnostics(d400e_serial_number, 1))
        {
            std::cout << "Device (SN: " << dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << ") diagnostics are turned ON." << std::endl;
        }
        // Toggle device diagnostics OFF
        if (!rs2::d400e::toggle_device_diagnostics(d400e_serial_number, 0))
        {
            std::cout << "Device (SN: " << dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << ") diagnostics are turned OFF." << std::endl;
        }

    }
    std::cout << std::endl;
}


// Get aditional D400e camera information.
void camera_information_example(rs2::device& dev) {
    std::cout << "Device Information: " << std::endl;

    //Get camera hardware and firmware version
    const char* cam_version = dev.get_info(RS2_CAMERA_INFO_DEVICE_VERSION);
    std::cout << "Device version of camera connected: " << cam_version << std::endl;

    //Get camera intel D4 firmware version 
    const char* fw_version = dev.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION);
    std::cout << "Firmware version of camera connected: " << fw_version << std::endl;

    //Get camera IP address
    const char* ip_address = dev.get_info(RS2_CAMERA_INFO_IP_ADDRESS);
    std::cout << "IP address of camera connected:  " << ip_address << std::endl;

    //Get camera subnet mask
    const char* subnet_mask = dev.get_info(RS2_CAMERA_INFO_SUBNET_MASK);
    std::cout << "Subnet mask of camera connected:  " << subnet_mask << std::endl;
    std::cout << std::endl;
}