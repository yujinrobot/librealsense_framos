# rs-multicam sample

## Overview

The multicam sample demonstrates the ability to use the SDK for streaming and rendering multiple devices simultaneously.
The original sample is modified to work with Framos D400e series cameras in the following way: options RS2_OPTION_INTER_PACKET_DELAY and RS2_OPTION_PACKET_SIZE are set to assure stable streams, and only Depth and Color streams are enabled with profile 640x480@30fps on both streams. The sample expects two D400e series cameras that are streaming to same NIC (Network Interface Card) on PC. The sample will detect and stream from Intel D400 series cameras as well. The sample was tested with two Framos D435e and one Intel D435i camera.

## Expected Output

The application opens and renders a mosaic view of all automatically-selected streams provided by the connected devices. (The selection is device-tailored and takes into account USB-type constrains).
Each tile displays an unique stream produced by a specific camera. The stream name appear at the top left.

In the following snapshot we use five Intel® RealSense™ devices to produce the mosaic: D435i, D415, SR300 and two T265 tracking cameras. Those are responsible to generate:
 - Three Depth streams (D435i, D415, SR300)
 - Three Color streams (D435i, D415, SR300)
 - Two Pose streams (T265x2)
 - Four Fisheye streams (T265x2)
 - Six IMU streams (D435i, T265x2)  

 Alltogether the mosaic comprise of 18 simultaneous live feeds:

<p align="center"><img src="https://raw.githubusercontent.com/wiki/IntelRealSense/librealsense/res/Multicam.gif" alt="screenshot gif"/></p>


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

The example's flow starts with setting packet size and inter packet delay for Framos D435e cameras. Optimal Inter Packet Delay value is calculted based on number of parallel streams and packet size.
```cpp
    float inter_packet_delay = get_optimal_inter_packet_delay(num_parallel_streams, packet_size);

    if (sensor.supports(RS2_OPTION_PACKET_SIZE)) {
		sensor.set_option(RS2_OPTION_PACKET_SIZE, packet_size);
	}
	
	if (sensor.supports(RS2_OPTION_INTER_PACKET_DELAY)) {
		sensor.set_option(RS2_OPTION_INTER_PACKET_DELAY, inter_packet_delay);
	}
```

The example's flow then continues with listing and activating all the connected Intel® RealSense™ devices:
```cpp
// Start a streaming pipe per each connected device
for (auto&& serial : serials)
{
    rs2::pipeline pipe(ctx);
    rs2::config cfg;

    // enable only depth and color streams
    cfg.enable_stream(RS2_STREAM_DEPTH, -1, 640, 480, RS2_FORMAT_Z16, 30);
    cfg.enable_stream(RS2_STREAM_COLOR, -1, 640, 480, RS2_FORMAT_RGB8, 30);

    cfg.enable_device(serial);
    pipe.start(cfg);
    pipelines.emplace_back(pipe);
    // Map from each device's serial number to a different colorizer
    colorizers[serial] = rs2::colorizer();
}
```

First, we allocate `rs2::pipeline` object per recognized device. Note that we share the `rs2::context` object between all `rs2::pipeline` instances.  
```cpp
rs2::pipeline pipe(ctx);
```
To map the specific device to the newly-allocated pipeline we define `rs2::config` object, and assign it with the device's serial number. Only Depth and Color streams with profile 640x480@30fps are enabled.
Then we request `rs::pipeline` to start streaming and produce frames.
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
