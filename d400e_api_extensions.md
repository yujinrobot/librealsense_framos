# D400e librealsense2 API extensions

This readme file provides an overview of extensions to the librealsense2 API implemented to support D400e cameras.

## Heartbeat time

Heartbeat mechanism is used to detect disconnect between the host and a D400e camera. Host sends the heartbeat command to the camera in regular intervals and the camera sends a response. If the camera does not respond in a certain interval, the host considers the camera disconnected. If the camera does not receive a heartbeat command in the same interval, it considers the host disconnected. Length of the interval in which the host sends the heartbeat command is called heartbeat time. Length of the interval after which the host considers the camera disconnected and vice versa is called heartbeat timeout.

Heartbeat time in seconds can be acquired and set using the extended librealsense2 API. Heartbeat timeout is implemented as 4x heartbeat time. All D400e cameras connected to a single application have the same heartbeat time. Setting heartbeat time affects all connected D400e cameras.

C++

```cpp
#include <librealsense2/rs.hpp>
```
```cpp
float heartbeat_time_s = rs2::d400e::get_heartbeat_time();
rs2::d400e::set_heartbeat_time(3.f);
```

C

```c
#include <librealsense2/rs.h>
```
```c
rs2_error* e = 0;
float heartbeat_time_s = rs2_d400e_get_heartbeat_time(&e);
rs2_d400e_set_heartbeat_time(3.f, &e);
```

Python

```python
heartbeat_time_s = rs.d400e.get_heartbeat_time()
rs.d400e.set_heartbeat_time(3)
```

C#

```c#
namespace Intel.Realsense
```

```c#
double heartbeatTimeS = D400e.GetHeartbeatTime();
D400e.SetHeartbeatTime(3);
```

## Buffer count

Images acquired from D400e cameras are stored in circular buffer inside the driver. The library is using images from this buffer and notifies the driver when a buffer is no longer needed. Driver drops images that arrive from camera if this buffer is full because the software is not using the images fast enough.

Buffer count can be acquired and set using the extended librealsense2 API. All D400e cameras connected to a single application have the same number of buffers. The buffer count must be set before instantiating `context` or `pipeline` objects. Setting buffer count after these objects are instantiated may have no effect.

C++

```cpp
#include <librealsense2/rs.hpp>
```
```cpp
int buffer_count = rs2::d400e::get_buffer_count();
rs2::d400e::set_buffer_count(10);
```

C

```c
#include <librealsense2/rs.h>
```
```c
rs2_error* e = 0;
int buffer_count = rs2_d400e_get_buffer_count(&e);
rs2_d400e_set_buffer_count(10, &e);
```

Python

```python
buffer_count = rs.d400e.get_buffer_count()
rs.d400e.set_buffer_count(10)
```

C#

```c#
namespace Intel.Realsense
```

```c#
double bufferCount = D400e.GetBufferCount();
D400e.SetBufferCount(10);
```

## Device diagnostics

Enable or disable diagnostic packets sent by D400e series devices.

C++

```cpp
#include <librealsense2/rs.hpp>
```
```cpp
int status = rs2::d400e::toggle_device_diagnostics("6CD146030D29", 1);
```

C

```c
#include <librealsense2/rs.h>
```
```c
rs2_error* e = 0;
int status;
status = rs2_d400e_toggle_device_diagnostics("6CD146030D29", 1, &e);
```

Python

```python
status = rs.d400e.toggle_device_diagnostics("6CD146030D29", 1);
```

C#

```c#
namespace Intel.Realsense
```

```c#
int status = D400e.ToggleDeviceDiagnostics("6CD146030D29", 1);
```

## Ports

FRAMOS librealsense2 library must open a number of UDP ports to communicate with D400e cameras. Same ports are used by all devices the library communicates with. By default, the port numbers are determined dynamically by the system.

### Port range

Preferred port range can be set using the extended librealsense2 API. Port range must be set before instantiating `context` or `pipeline` objects. Setting port range after these objects are instantiated may have no effect.

Because port range is disabled by default, attempting to get the preferred port range before setting it will cause an error.

If there is not enough free ports in the preferred range, an exception will be thrown.

C++

```cpp
#include <librealsense2/rs.hpp>
```
```cpp
rs2::d400e::set_port_range({ 50000, 65000 });
rs2::d400e::port_range port_range = rs2::d400e::get_port_range();
```

C

```c
#include <librealsense2/rs.h>
```
```c
rs2_error* e = 0;
unsigned short min, max;
rs2_d400e_set_port_range(50000, 65000, &e);
rs2_d400e_get_port_range(&min, &max, &e);
```

Python

```python
rs.d400e.set_port_range(rs.d400e.port_range(50000, 65000))
port_range = rs.d400e.get_port_range()
```

C#

```c#
namespace Intel.Realsense
```

```c#
D400e.SetPortRange(new D400e.PortRange(50000, 65000));
D400e.PortRange portRange = D400e.GetPortRange();
```

### Port list

Opened ports can be listed using the extended librealsense2 API. Opened ports can be queried by type. Possible port types are control ports, stream ports and message ports.

C++

```cpp
#include <librealsense2/rs.hpp>
```
```cpp
std::vector<uint16_t> all_ports = rs2::d400e::query_ports();
```
```cpp
std::vector<uint16_t> all_ports = rs2::d400e::query_ports(RS2_D400E_PORT_TYPE_ALL);
```

C

```c
#include <librealsense2/rs.h>
```
```c
rs2_error* e = 0;
unsigned int port_count;
unsigned short first_port;
rs2_d400e_port_list* all_ports = rs2_d400e_query_ports(&e);
port_count = rs2_d400e_get_port_count(all_ports, &e);
first_port = rs2_d400e_get_port(all_ports, 0, &e);
rs2_d400e_delete_port_list(all_ports);
```
```c
rs2_error* e = 0;
rs2_d400e_port_list* all_ports = rs2_d400e_query_ports_by_type(RS2_D400E_PORT_TYPE_ALL, &e);
rs2_d400e_delete_port_list(all_ports);
```

Python

```python
all_ports = rs.d400e.query_ports();
```
```python
all_ports = rs.d400e.query_ports(rs.d400e_port_type.all);
```

C#

```c#
namespace Intel.Realsense
```
```c#
D400e.PortList allPorts = D400e.QueryPorts();
```
```c#
D400e.PortList allPorts = D400e.QueryPorts(D400e.PortType.All);
```

## Camera information

Available camera information in the librealsense2 API is listed in the `rs2_camera_info`  enumeration available in the `librealsense2/h/rs_sensor.h` header file. This enumeration was extended to provide information specific to D400e cameras.

The `RS2_CAMERA_INFO_DEVICE_VERSION` enumerator represents FRAMOS firmware version on a D400e camera. This value is different from the `RS2_CAMERA_INFO_FIRMWARE_VERSION` enumerator which represents the Intel D4 firmware version.

The `RS2_CAMERA_INFO_IP_ADDRESS` enumerator represents the IP address of a D400e camera.

The `RS2_CAMERA_INFO_SUBNET_MASK` enumerator represents the subnet mask of a D400e camera.

Same API calls are used to obtain information from both normal and extended enumerators.

C++

```cpp
rs2::device device; //obtain rs2::device from rs2::context or rs2::pipeline_profile
const char* ip_address = device.get_info(RS2_CAMERA_INFO_IP_ADDRESS);
```

C

```c
rs2_device* dev; //obtain rs2_device* using rs2_create_device()
rs2_error* e = 0;
const char* ip_address;
ip_address = rs2_get_device_info(device, RS2_CAMERA_INFO_IP_ADDRESS, &e);
```

Python

```python
#obtain device from rs.context() or rs.pipeline()
ip_address = device.get_info(rs.camera_info.ip_address)
```

C#

```c#
namespace Intel.RealSense
```

```c#
Device device; //obtain Device from Context or PipelineProfile
String ipAddress = device.Info[CameraInfo.IpAddress];
```

## Sensor Options

Available sensor options in the librealsense2 API are listed in the `rs2_option` enumeration available in the `librealsense2/h/rs_option.h` header file. This enumeration was extended to provide options specific to D400e cameras.

The `RS2_OPTION_INTER_PACKET_DELAY` enumerator represents the delay in microseconds between stream packets that the camera sends to the host. The library automatically detects optimal value for this option on initialization.

The `RS2_OPTION_PACKET_SIZE` enumerator represents the size of stream packets in bytes that the camera uses to stream images. The library automatically detects optimal value for this option on initialization. This option cannot be set while the sensor is streaming.

The `RS2_OPTION_USER_OUTPUT_LEVEL` enumerator represents output level of M8 pin3 (opto-isolated OUT). 

The `RS2_OPTION_EXT_TRIGGER_SOURCE` enumerator represents external trigger mode. Value 1 represents hardware trigger and value 2 software trigger. See `Framos_D435e_External_Event_Camera_Synchronization_AppNote` for details.

The `RS2_OPTION_SOFTWARE_TRIGGER` enumerator executes software trigger when set to 1.  See `Framos_D435e_External_Event_Camera_Synchronization_AppNote` for details.

The `RS2_OPTION_SOFTWARE_TRIGGER_ALL_SENSORS` enumerator selects which sensors receive the software trigger signal. When set to 1, both stereo and color sensor receive software trigger signal. When set to 0, only the stereo sensor receives the software trigger signal. See `Framos_D435e_External_Event_Camera_Synchronization_AppNote` for details.

The `RS2_OPTION_LINE_DEBOUNCER_TIME` enumerator represents the line debouncer time in microseconds. This option affects the signal applied on M8 pin2 (opto-isolated IN). See `FRAMOS_D400e_UserManual` for details.

The `RS2_OPTION_RGB_LED_TOGGLE` enumerator represents the state of the RGB LED light located on the Color Sensor. When set to true (default), the LED light is turned on. When set to false, the LED light is turned off. This option is only available for the Color Sensor.

The `RS2_OPTION_METADATA_TOGGLE` enumerator represents the state of the Metadata feature on the Camera. When set to false (default), the Metadata transfer is disabled. When set to true, the Metadata transfer is enabled. This option is only available for the Depth Sensor.

Same API calls are used to set both normal and extended options.

C++

```cpp
rs2::sensor sensor; //obtain rs2::sensor from rs2::device or rs2::context
sensor.set_option(RS2_OPTION_INTER_PACKET_DELAY, 65.f);
```

C

```c
rs2_sensor* sensor; //obtain rs2_sensor* using rs2_create_sensor()
rs2_error* e = 0;
rs2_set_option(sensor, RS2_OPTION_INTER_PACKET_DELAY, 65.f, &e);
```

Python

```python
#obtain sensor from device or rs.context()
sensor.set_option(rs.option.inter_packet_delay, 65)
```

C#

```c#
namespace Intel.RealSense
```

```c#
Sensor sensor; //obtain Sensor from Device
sensor.Options[Option.InterPacketDelay].Value = 65;
```

## Syncer Options

D400e cameras have a possibility to synchronize streams to an external event (using external event operating mode). As this is non-continuous working mode syncer module is extended to handle non-continuous events. Available syncer options in the librealsense2 API are listed in the `rs2_syncer_mode` enumeration available in the `librealsense2/h/rs_types.h` header file.

The `RS2_SYNCER_MODE_DEFAULT` enumerator represents the default syncer module (original librealsense2 syncer implementation).

The `RS2_SYNCER_MODE_WAIT_FRAMESET` enumerator represents the modified syncer module with support for external events specific to D400e cameras. Syncer returns synchronized frameset when frames from all enabled streams have arrived. If there is a missing frame within specific external event, syncer will not return frameset and pipeline call wait_for_frames will return timeout for this specific external event.

C++

```cpp
rs2::config cfg; //config object
cfg.set_syncer_mode(RS2_SYNCER_MODE_WAIT_FRAMESET);
```

C

```c
rs2_error* e = 0;
rs2_config* config = rs2_create_config(&e);
check_error(e);
rs2_config_set_syncer_mode(config, RS2_SYNCER_MODE_WAIT_FRAMESET, &e);
```

Python

```python
config = rs.config()
config.set_syncer_mode(rs.syncer_mode.wait_frameset)
```

C#

```c#
var config = new Config();
config.SetSyncerMode(SyncerModes.SYNCER_MODE_WAIT_FRAMESET);
```

## Device filtering by serial number or/and IP address

Device filtering feature enables applications to connect only on cameras that are specified in the filtering list. This way, when multiple librealsense2 based applications exist at the same time, each application can connect and work only with their specific cameras. 

Filtering file location can be passed to the application through command line arguments and application can pass these arguments to the librealsense2 via provided API function (`rs2_d400e_set_cli_args`). If command line arguments are provided, the librealsense2 will try to open filter file and perform device filtration according to the specification in the filter file. 
Additionally, filtering list can be hard-coded in a program and passed to to the librealsense2 via provided API function ('rs2_d400e_set_device_filter_list'). If both mechanism are used, librealsense2 will merge entries and form a single filtering list.

The device filter file (or hard-coded filter list) must be set before instantiating `rs2::context` or `rs2::pipeline` objects. Setting device filter file (or hard-coded filter list) after these objects are instantiated may have no effect.

Syntax of the command line argument which defines device filter file (file name can be arbitrary):
`dev_filter=d400e_filter.json`

Filtering file must be in JSON file format. Only two JSON objects are supported, serial_range and ip_range. Both of them can be defined multiple times.

JSON device filtering file format:
```json
[
    {
        "serial_range":
        {
            "begin": "6C:D1:46:03:0D:00", "end": "6C:D1:46:03:0D:20"
        }
    },

    {
        "serial_range":
        {
            "begin": "6C:D1:46:03:01:33", "end": "6C:D1:46:03:01:33"
        }
    },

    {
        "ip_range":
        {
            "begin": [192, 168, 1, 2], "end": [192, 168, 1, 50]
        }
    }
]
```

C++
```cpp
int main(int argc, const char** argv)
{
    // provide the filtering list via JSON file
    rs2_cli_arg carg;
    carg.argc = argc;
    carg.argv = argv;
    rs2::d400e::set_cli_args(carg);
    
    // or provide the hard-coded filtering list
    rs2_d400e_filter filter;
    filter.serial_range.begin = 0x6CD1460300;
    filter.serial_range.end = 0x6CD1460302;
    std::vector<rs2_d400e_filter> filter_list;
    filter_list.push_back(filter);
    rs2::d400e::set_device_filter_list(filter_list);
    
    return 0;
}
```

C
```c
int main(int argc, const char** argv)
{
    rs2_cli_arg carg;
    rs2_d400e_filter filter;
    rs2_d400e_filter_list* filter_list;
    
    // provide the filtering list via JSON file
    carg.argc = argc;
    carg.argv = argv;
    rs2_d400e_set_cli_args(&carg, 0);
    
    // or provide the hard-coded filtering list
    filter_list = rs2_d400e_new_filter_list(0);
    filter.serial_range.begin = 0x6CD1460300;
    filter.serial_range.end = 0x6CD1460302;
    rs2_d400e_filter_list_insert(&filter, filter_list, 0);
    rs2_d400e_set_device_filter_list(filter_list, 0);
    rs2_d400e_delete_filter_list(filter_list);

    return 0;
}
```

Python
```python
import pyrealsense2 as rs
try:
    # provide the filtering list via JSON file
    rs.d400e.set_cli_args("dev_filter=d400e_filter.json")
    
    # or provide the hard-coded filtering list
    rs_filter = rs.d400e.rs2_d400e_filter()
    rs_filter.serial_range.begin = 0x6CD1460300
    rs_filter.serial_range.end = 0x6CD1460302
    filter_list = [rs_filter]
    rs.d400e.set_device_filter_list(filter_list)
    
except Exception as e:
    print(e)
    pass
```

ROS
```sh
roslaunch realsense2_camera framos_rs_camera.launch dev_filter:="dev_filter=/home/username/d400e_filter.json"
```
To run multiple ROS applications that use multiple D400e cameras it is required ro separate the cameras used by different applications by namespace. If this is not done the publishers of different cameras might be used by a single subscriber, causing the mixing of frames.
```sh
roslaunch realsense2_camera framos_rs_camera.launch namespace:="camera1" dev_filter:="dev_filter=/home/username/Cam1.json"
roslaunch realsense2_camera framos_rs_camera.launch namespace:="camera2" dev_filter:="dev_filter=/home/username/Cam2.json"
```

ROS2
```sh
ros2 launch realsense2_camera d400e_rs_launch.py dev_filter:="dev_filter=/home/username/d400e_filter.json"
```
Similar to ROS, to run multiple ROS2 applications that use multiple D400e cameras it is required ro separate the cameras used by different applications by namespace, the namespace in ROS2 is replaced by "camera_name". If this is not done the publishers of different cameras might be used by a single subscriber, causing the mixing of frames.
```sh
ros2 launch realsense2_camera d400e_rs_launch.py camera_name:="camera1" dev_filter:="dev_filter=/home/username/Cam1.json"
ros2 launch realsense2_camera d400e_rs_launch.py camera_name:="camera2" dev_filter:="dev_filter=/home/username/Cam2.json"
```

