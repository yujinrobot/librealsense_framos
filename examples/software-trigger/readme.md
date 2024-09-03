# rs-software-trigger sample

## Overview

The software trigger sample demonstrates the ability to use the SDK for executing software trigger on FRAMOS D435e camera.

## Expected Output

The application opens and renders a mosaic view of Depth, Infrared and Color streams provided by the connected devices. Each tile displays an unique stream produced by a specific camera. The stream name appear at the top left.
Stream is acquired by hitting specific keyboard key.

## Code Overview

As with any SDK application we include the Intel RealSense Cross Platform API:

```cpp
#include <librealsense2/rs.hpp>     // Include RealSense Cross Platform API
```

In this example we will also use the auxiliary library of `example.hpp`:

```cpp
#include "example.hpp"              // Include short list of convenience functions for rendering
```

`examples.hpp` lets us easily open a new window and prepare textures for rendering.


The first object we use is a `window`, that will be used to display the images from all the cameras.

```cpp
// Create a simple OpenGL window for rendering:
window app(1280, 960, "CPP Multi-Camera Example");
```

The `window` class resides in `example.hpp` and lets us easily open a new window and prepare textures for rendering.

Next, we define the objects to be used in the example.

```cpp
rs2::context                          ctx;        // Create librealsense context for managing devices

std::map<std::string, rs2::colorizer> colorizers; // Declare map from device serial number to colorizer (utility class to convert depth data RGB colorspace)

std::vector<rs2::pipeline>            pipelines;
```
The `rs2::context` encapsulates all of the devices and sensors, and provides some additional functionalities. We employ the `rs2::colorizer ` to convert depth data to RGB format.
In the example we use multiple `rs2::pipeline` objects, each controlling a lifetime of a single HW device. Similarly, we initialize a separate `rs2::colorizer` object for each device. We keep a mapping from the device's serial number to it's `rs2::colorizer` object, this way we'll be able to apply the correct `rs2::colorizer` to each frame.

The example's flow starts with setting inter cam sync mode and software trigger mode for Framos D435e cameras. Cameras are set to external event operation mode. Read Framos_D435e_External_Event_Camera_Synchronization_AppNote_v1-0-0 for explanation on operating modes for Framos D435e cameras.
```cpp
    if (sensor && sensor.is<rs2::depth_stereo_sensor>()) {
        if ((std::regex_search(name, d435e_regex)) && (std::regex_search(dev_version, fw_version_d435e))) {
            sensor.set_option(RS2_OPTION_INTER_CAM_SYNC_MODE, 259);
            sensor.set_option(RS2_OPTION_SOFTWARE_TRIGGER_ALL_SENSORS, 1);
        }
        else {
            sensor.set_option(RS2_OPTION_INTER_CAM_SYNC_MODE, 3);
            sensor.set_option(RS2_OPTION_SOFTWARE_TRIGGER_ALL_SENSORS, 1);
        }
    }
    else {
        sensor.set_option(RS2_OPTION_INTER_CAM_SYNC_MODE, 1);
    }
```

Optimal Inter Packet Delay value is calculated based on number of parallel streams and packet size.
```cpp
    float interPacketDelay = getOptimalInterPacketDelay(numParallelStreams, packetSize);

    sensor.set_option(RS2_OPTION_PACKET_SIZE, packetSize);
    sensor.set_option(RS2_OPTION_INTER_PACKET_DELAY, interPacketDelay);
```

The example's flow then continues with listing and activating all the connected Intel® RealSense™ devices, with additional syncer mode configuration:
```cpp
// Start a streaming pipe per each connected device
for (auto&& dev : ctx.query_devices())
{
    rs2::pipeline pipe(ctx);
    rs2::config cfg;
    
    // enable only depth and color streams
    cfg.enable_stream(RS2_STREAM_DEPTH, -1, 640, 480, RS2_FORMAT_Z16, 30);
    cfg.enable_stream(RS2_STREAM_INFRARED, 1, 640, 480, RS2_FORMAT_Y8, 30);
    cfg.enable_stream(RS2_STREAM_INFRARED, 2, 640, 480, RS2_FORMAT_Y8, 30);
    cfg.enable_stream(RS2_STREAM_COLOR, -1, 640, 480, RS2_FORMAT_RGB8, 30);

    cfg.enable_device(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
	
	//syncer mode for external event operating mode
	cfg.set_syncer_mode(RS2_SYNCER_MODE_WAIT_FRAMESET);
	
    pipe.start(cfg);
    pipelines.emplace_back(pipe);
    // Map from each device's serial number to a different colorizer
    colorizers[dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)] = rs2::colorizer();
}
```

The example's flow continues with setting the external event to software trigger mode.
```cpp
// Start a streaming pipe per each connected device
for (auto&& sensor : pipe.get_active_profile().get_device().query_sensors()) {
    if (sensor && !sensor.is<rs2::motion_sensor>()) {
        if (sensor && sensor.is<rs2::depth_stereo_sensor>()) {
            sensorDepth = sensor;
            sensor.set_option(RS2_OPTION_EXT_TRIGGER_SOURCE, 2);
        }
        else {
            sensorColor = sensor;
            sensor.set_option(RS2_OPTION_EXT_TRIGGER_SOURCE, 2);
        }
    }
}
```

UpdateScreenInfo function is used for displaying the menu on how to control the software trigger mode change and execution.
```cpp
updateScreenInfo(sensorDepth, sensorColor, sensor_actions, false);
```

First, we allocate `rs2::pipeline` object per recognized device. Note that we share the `rs2::context` object between all `rs2::pipeline` instances.  
```cpp
rs2::pipeline pipe(ctx);
```
To map the specific device to the newly-allocated pipeline we define `rs2::config` object, and assign it with the device's serial number. Only Depth and Color streams with profile 640x480@30fps are enabled. Then we request `rs::pipeline` to start streaming and produce frames.
```cpp
pipe.start(cfg);
```

Since we do not specify explicit stream requests, each device is configured internally to run a set of predefined stream profiles recommended for that specific device.  

After adding the device, we begin our main loop of the application:  
```cpp
while (app)
```

Every application cycle we traverse the registered devices and retrieve all the available frames:

```cpp
// Collect the new frames from all the connected devices
std::vector<rs2::frame> new_frames;
for (auto &&pipe : pipelines)
{
   rs2::frameset fs;
   if (pipe.poll_for_frames(&fs))
   {
       for (rs2::frame& f : fs)
           new_frames.emplace_back(f);
   }
}
```
Each `rs::pipeline` produces a synchronized collection of frames for all streams configured for its allocated device. These are contained in `rs2::frameset` object.
The `rs2::frameset` itself is an wrapper for a `composite_frame`, which can holds more than a single type of frame.  

To minimize UI impact we're using non-blocking frames polling method:
```cpp
    if (pipe.poll_for_frames(&fs))
```
In order to simplify the presentation, we split those `rs2::frameset` containers into separate frames and store them with a standard C++ container for later use:  
```cpp
for (rs2::frame& f : fs)
    new_frames.emplace_back(f);
```

The Depth data is delivered as `uint16_t` type which cannot be rendered directly, therefore we use `rs2::colorizer` to convert the depth representation into human-readable RGB map. We use `rs2::sensor_from_frame` function to retrieve the serial number of the frame's device. Then we can use that device's `rs2::colorizer` to process the frame:
```cpp
// Convert the newly-arrived frames to render-friendly format
for (const auto& frame : new_frames)
{
    // Get the serial number of the current frame's device
    auto serial = rs2::sensor_from_frame(frame)->get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
    // Apply the colorizer of the matching device and store the colorized frame
    render_frames[frame.get_profile().unique_id()] = colorizers[serial].process(frame);
}
```

And finally send the collected frames to update the openGl mosaic:
```cpp
    app.show(render_frames);
```
