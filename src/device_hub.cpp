// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp>
#include "source.h"
#include "core/processing.h"
#include "proc/synthetic-stream.h"
#include "device_hub.h"
#ifdef FRAMOS
#include <cs/cs-factory.h>
#endif

namespace librealsense
{
    typedef rs2::devices_changed_callback<std::function<void(rs2::event_information& info)>> hub_devices_changed_callback;

    std::vector<std::shared_ptr<device_info>> filter_by_vid(std::vector<std::shared_ptr<device_info>> devices , int vid)
    {
        std::vector<std::shared_ptr<device_info>> result;
        for (auto dev : devices)
        {
            bool filtered = false;
            auto data = dev->get_device_data();
            for (const auto& usb : data.usb_devices)
            {
                if (usb.vid == vid || vid == 0)
                {
                    result.push_back(dev);
                    filtered = true;
                    break;
                }
            }
            for (const auto& uvc : data.uvc_devices)
            {
                if (uvc.vid == vid || vid == 0)
                {
                    result.push_back(dev);
                    filtered = true;
                    break;
                }
            }
#ifdef FRAMOS

            for (const auto& cs : data.cs_devices)
            {
                if (cs.vid == vid || vid == 0)
                {
                    result.push_back(dev);
                    filtered = true;
                    break;
                }
            }
#endif
        }
        return result;
    }

    device_hub::device_hub(std::shared_ptr<librealsense::context> ctx, int mask, int vid)
        : _ctx(ctx), _vid(vid),
          _device_changes_callback_id(0)
    {
        _device_list = filter_by_vid(_ctx->query_devices(mask), _vid);

        auto cb = new hub_devices_changed_callback([&,mask](rs2::event_information&)
                   {
                        std::unique_lock<std::mutex> lock(_mutex);

                        _device_list = filter_by_vid(_ctx->query_devices(mask), _vid);

                        // Current device will point to the first available device
                        _camera_index = 0;
                        if (_device_list.size() > 0)
                        {
                           _cv.notify_all();
                        }
                    });

        _device_changes_callback_id = _ctx->register_internal_device_callback({ cb, [](rs2_devices_changed_callback* p) { p->release(); } });
    }

    device_hub::~device_hub()
    {
        if (_device_changes_callback_id)
            _ctx->unregister_internal_device_callback(_device_changes_callback_id);

        _ctx->stop();
    }

    std::shared_ptr<device_interface> device_hub::create_device(const std::string& serial, bool cycle_devices)
    {
        std::shared_ptr<device_interface> res = nullptr;
        for(size_t i = 0; ((i< _device_list.size()) && (nullptr == res)); i++)
        {
            // _camera_index is the curr device that the hub will expose
            auto d = _device_list[ (_camera_index + i) % _device_list.size()];
            try
            {
#ifdef FRAMOS
                auto device_group = d->get_device_data();

                if (serial.substr(0, 6) == CS_FRAMOS_SERIAL_BEGIN)
                {
                    if (device_group.cs_devices.size() > 0)
                    {
                        auto cs_device = device_group.cs_devices.at(0);

                        if (cs_device.serial == serial)
                        {
                            res = d->create_device();
                            cycle_devices = false;  // Requesting a device by its serial shall not invoke internal cycling
                        }
                    }
                }
                else if (device_group.cs_devices.size() == 0)
                {
#endif // FRAMOS
                    auto dev = d->create_device();

                    if(serial.size() > 0 )
                    {
                        auto new_serial = dev->get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);

                        if(serial == new_serial)
                        {
                            res = dev;
                            cycle_devices = false;  // Requesting a device by its serial shall not invoke internal cycling
                        }
                    }
                    else // Use the first selected if "any device" pattern was used
                    {
                        res = dev;
                    }
#ifdef FRAMOS
                }
                else if (serial.size() == 0)
                {
                    res = d->create_device();
                }

#endif // FRAMOS 
            }
            catch (const std::exception& ex)
            {
                LOG_WARNING("Could not open device " << ex.what());
            }
        }

        // Advance the internal selection when appropriate
        if (res && cycle_devices)
            _camera_index = (1+_camera_index) % _device_list.size();

        return res;
    }


    /**
     * If any device is connected return it, otherwise wait until next RealSense device connects.
     * Calling this method multiple times will cycle through connected devices
     */
    std::shared_ptr<device_interface> device_hub::wait_for_device(const std::chrono::milliseconds& timeout, bool loop_through_devices, const std::string& serial)
    {
        std::unique_lock<std::mutex> lock(_mutex);

        std::shared_ptr<device_interface> res = nullptr;

        // check if there is at least one device connected
        _device_list = filter_by_vid(_ctx->query_devices(RS2_PRODUCT_LINE_ANY), _vid);
        if (_device_list.size() > 0)
        {
            res = create_device(serial, loop_through_devices);
        }

        if (res) return res;

        // block for the requested device to be connected, or till the timeout occurs
        if (!_cv.wait_for(lock, timeout, [&]()
        {
            if (_device_list.size() > 0)
            {
                res = create_device(serial, loop_through_devices);
            }
            return res != nullptr;
        }))
        {
            throw std::runtime_error("No device connected");
        }
        return res;
    }

    /**
    * Checks if device is still connected
    */
    bool device_hub::is_connected(const device_interface& dev)
    {
        std::unique_lock<std::mutex> lock(_mutex);
        return dev.is_valid();
    }

    std::shared_ptr<librealsense::context> device_hub::get_context()
    {
        return _ctx;
    }
}

