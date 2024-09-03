# rs-d400e-api-extensions Sample

## Overview
RS-D400e API Extensions example demonstrates the use of Framos D400e camera API extensions, which include device Filtering, Heartbeat Time, Device Diagnostics, Buffer Count, Device Diagnostics, Port Range and Camera Information.
Sensor Options and Syncer Options are demonstrated in software trigger example so they are excluded from this example.

## Expected Output
Usage for RS-D400e API is demonstrated. Each API extension use is contained in its own function.

Device Filtering Example: Usage of a device filter file is demonstrated.

Port Range Example: Used ports are printed out.

Heartbeat Time Example: Default Heartbeat Time is printed out, after which default Heartbeat time is set to 1s and then returned to default to not interfere with D400e examples or applications being run later.

Buffer Count Example: Default Buffer Count is printed out, after which default Buffer Count is set to 20 and then returned to default to not interfere with D400e examples or applications being run later.

Device diagnostics Example: If toggling the device diagnostics packets ON/OFF is successful a message is printed out.

Camera Information Example: FRAMOS Camera information is printed out.

## Code Overview 

As with any SDK application we include the Intel RealSense Cross Platform API:

```cpp
#include <librealsense2/rs.hpp>     // Include RealSense Cross Platform API
```

D400e API extensions example functions are called from main.
```cpp
device_filtering(argc, argv);
port_range_example();
heartbeat_time_example();   
buffer_count_example();
```

After those functions are executed context and device objects need to be initialized.
```cpp
rs2::context& ctx = rs2::context();
rs2::device& dev = ctx.query_devices()[0];
```

Example functions with API extensions that require context and device objects are called.
```cpp
device_diagnostics_example(dev, "6CD146030D2C");    
camera_information_example(dev);
```

Function code is explained with comments.
