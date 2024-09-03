// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 FRAMOS GmbH.

#include "cs/cs-sensor.h"
#include "cs/cs-options.h"
#include "global_timestamp_reader.h"
#include "stream.h"
#include "device.h"
#include "ds5/ds5-motion.h"

#include <array>
#include <set>
#include <unordered_set>
#include <iomanip>
#include <mutex>
#include <smcs_cpp/CameraSDK.h>

namespace librealsense 
{
    namespace platform
    {
        std::mutex g_mtx_callback;

        void device_event_callback(const smcs_CallbackDeviceEventInfo* dev_event_info, void* forward_arg)
        {
            std::lock_guard<std::mutex> locker(g_mtx_callback); // TODO - nh - verify if it is necessary

            if (forward_arg != nullptr) {
                cs_device* device = static_cast<cs_device*>(forward_arg);
                device->new_event((void*)dev_event_info);
            }
            else {
                // TODO - nh - log error
            }
        }

        bool cs_device::configure_message_channel(smcs::IDevice device)
        {
            bool status = false;

            try {
                INT64 value;

                // Get device information
                _device_serial = device->GetSerialNumber();
                _device_addr = device->GetIpAddress();

                value = 0;
                status = device->GetIntegerNodeValue("EventAccelerometer", value);
                if (status) {
                    _event_acc_id = (uint32_t)value;
                }

                value = 0;
                status &= device->GetIntegerNodeValue("EventGyroscope", value);
                if (status) {
                    _event_gyro_id = (uint32_t)value;
                }
            }
            catch (const std::system_error& e) {
                // TODO - nh
            }

            return status;
        }

        bool cs_device::configure_event(smcs::IDevice device, const cfg_event_stream_list& config_list)
        {
            bool r_status = false;
            bool status = true;

            cfg_event_stream_list_const_it c_it = config_list.begin();
            while (c_it != config_list.end()) {

                if ((*c_it).event_selector == XML_EVENT_SELECTOR_ACC) {
                    status &= device->SetStringNodeValue("EventAccelerometerDataRate", (*c_it).event_data_rate);
                }
                else if ((*c_it).event_selector == XML_EVENT_SELECTOR_GYRO) {
                    status &= device->SetStringNodeValue("EventGyroscopeDataRate", (*c_it).event_data_rate);
                }

                c_it++;
            }

            if (config_list.size() > 0) {
                r_status = status;
            }

            return r_status;
        }

        bool cs_device::toggle_event_acc(smcs::IDevice device, bool state)
        {
            bool status = false;

            if (state == true) {
                status = device->SetStringNodeValue("EventSelector", XML_EVENT_SELECTOR_ACC);
                status &= device->SetStringNodeValue("EventNotification", XML_EVENT_NOTIFICATION_ON);
            }
            else {
                status = device->SetStringNodeValue("EventSelector", XML_EVENT_SELECTOR_ACC);
                status &= device->SetStringNodeValue("EventNotification", XML_EVENT_NOTIFICATION_OFF);
            }

            return status;
        }

        bool cs_device::toggle_event_gyro(smcs::IDevice device, bool state)
        {
            bool status = false;

            if (state == true) {
                status = device->SetStringNodeValue("EventSelector", XML_EVENT_SELECTOR_GYRO);
                status &= device->SetStringNodeValue("EventNotification", XML_EVENT_NOTIFICATION_ON);
            }
            else {
                status = device->SetStringNodeValue("EventSelector", XML_EVENT_SELECTOR_GYRO);
                status &= device->SetStringNodeValue("EventNotification", XML_EVENT_NOTIFICATION_OFF);
            }

            return status;
        }

        void cs_device::open_event(hid_callback callback)
        {
            if (!_is_event_capturing) {
                _event_data_list.clear();
                _callback_event = callback;
            }
            else {
                throw wrong_api_call_sequence_exception("Device already streaming events!");
            }
        }

        void cs_device::close_event()
        {
            // TODO
        }

        void cs_device::start_event(const cfg_event_stream_list& config_list, std::function<void(const notification &n)> error_handler)
        {
            init_event_stream(config_list, error_handler);
        }

        void cs_device::stop_event()
        {
            deinit_event_stream();
        }

        void cs_device::init_event_stream(const cfg_event_stream_list& config_list, std::function<void(const notification& n)> error_handler)
        {
            if (!_is_event_capturing) {

                _error_handler_event = error_handler;

                // TODO - nh - Check if resource "_connected_device" should be protected!

                bool status = configure_event(_connected_device, config_list);
                if (status) {

                    cfg_event_stream_list_const_it c_it = config_list.begin();
                    while (c_it != config_list.end()) {

                        if ((*c_it).event_selector == XML_EVENT_SELECTOR_ACC) {
                            status = status && toggle_event_acc(_connected_device, true);
                            _is_acc_configured = status;
                        }
                        else if ((*c_it).event_selector == XML_EVENT_SELECTOR_GYRO) {
                            status = status && toggle_event_gyro(_connected_device, true);
                            _is_gyro_configured = status;
                        }

                        c_it++;
                    }
                    
                    if (status) {
                        _is_event_capturing = true;
                        _thread_event = std::unique_ptr<std::thread>(new std::thread([this]() { capture_event_loop(); }));
                        _connected_device->RegisterEventCallback(device_event_callback, this);
                    }
                    else {
                        throw wrong_api_call_sequence_exception("Device events configuration error (2)!");
                    }
                }
                else {
                    throw wrong_api_call_sequence_exception("Device events configuration error (1)!");
                }
            }
            else {
                throw wrong_api_call_sequence_exception("Device already streaming events!");
            }
        }

        void cs_device::deinit_event_stream()
        {
            if (_is_event_capturing) {
                _is_event_capturing = false;
                _cv_callback_event.notify_all();
                if (_thread_event->joinable()) {
                    _thread_event->join();
                }
                _thread_event.reset();
                toggle_event_acc(_connected_device, false);
                toggle_event_gyro(_connected_device, false);
                _connected_device->UnregisterEventCallback(device_event_callback, this);
                _is_acc_configured = false;
                _is_gyro_configured = false;
                _event_data_list.clear();
            }
        }

        void cs_device::new_event(void* dev_event_info)
        {
            if (dev_event_info != nullptr) {
                {
                    std::lock_guard<std::mutex> locker(_mtx_callback_event);
                    smcs_CallbackDeviceEventInfo* event_info = static_cast<smcs_CallbackDeviceEventInfo*>(dev_event_info);

                    cs_event_data ed;
                    memcpy(&ed.data[0], &(event_info->packet[0]), GVCP_MAX_UDP_DATA_SIZE);
                    _event_data_list.push_back(ed);
                }
                _cv_callback_event.notify_one();
            }
        }

        void cs_device::capture_event_loop()
        {
            try {
                while (_is_event_capturing) {
                    event_monitor();
                }
            }
            catch (const std::exception& e) {

                LOG_ERROR(e.what());

                librealsense::notification n = { RS2_NOTIFICATION_CATEGORY_UNKNOWN_ERROR, 0, RS2_LOG_SEVERITY_ERROR, e.what() };

                // TODO - nh
                //_error_handler_event(n);
            }
        }

        void cs_device::event_monitor()
        {
            if (_connected_device.IsValid() &&
                _connected_device->IsConnected()) {

                // continue only if stream is configured for accelerometer or gyroscope data
                if ((!_is_acc_configured) && 
                    (!_is_gyro_configured)) {
                    return;
                }

                std::unique_lock<std::mutex> locker(_mtx_callback_event);
                _cv_callback_event.wait(locker, [this] { return (!_event_data_list.empty()) || (!_is_event_capturing); });

                if (!_is_event_capturing) {
                    return;
                }

                cs_event_data ed;
                memcpy(&ed.data[0], &(_event_data_list.front().data[0]), GVCP_MAX_UDP_DATA_SIZE);
                eventdata_cmd_message* event_pkt = (eventdata_cmd_message*)(&ed.data[0]);
                _event_data_list.pop_front();

                locker.unlock();

                float raw_x_os = 0.0f;
                float raw_y_os = 0.0f;
                float raw_z_os = 0.0f;
                metadata_hid_raw meta_data{};
                hid_sensor_data hid_data{};
                sensor_data sd{};
                sd.sensor.name = "";

                // check if gvcp packet
                if (event_pkt->header_gvcp.key_field == GVCP_KEY_VALUE) {
                                
                    // check if gev EVENTDATA packet
                    if (GevSwapWord(event_pkt->header_gvcp.command) == GVCP_EVENTDATA_CMD) {
                                
                        uint32_t tstamp_low = GevSwapDWord(event_pkt->event_timestamp_low);
                        uint32_t tstamp_high = GevSwapDWord(event_pkt->event_timestamp_high);
                        uint64_t tstamp_us = ((uint64_t)(((uint64_t)tstamp_high << 32) | ((uint64_t)tstamp_low)));
                        uint64_t tstamp_ms = tstamp_us * TIMESTAMP_USEC_TO_MSEC;
                                
                        // prepare sensor_data structure
                        meta_data.header.report_type = md_hid_report_type::hid_report_imu;
                        meta_data.header.length = hid_header_size + metadata_imu_report_size;
                        //meta_data.header.timestamp = tstamp_us;
                        meta_data.header.timestamp = tstamp_ms;
                                
                        meta_data.report_type.imu_report.header.md_type_id = md_type::META_DATA_HID_IMU_REPORT_ID;
                        meta_data.report_type.imu_report.header.md_size = metadata_imu_report_size;
                                
                        meta_data.report_type.imu_report.flags = static_cast<uint8_t>(
                            md_hid_imu_attributes::custom_timestamp_attirbute/* |
                            md_hid_imu_attributes::imu_counter_attribute |
                            md_hid_imu_attributes::usb_counter_attribute*/);
                        meta_data.report_type.imu_report.custom_timestamp = tstamp_ms;
                        meta_data.report_type.imu_report.imu_counter = 0;
                        meta_data.report_type.imu_report.usb_counter = 0;
                                                
                        memcpy((void*)&raw_x_os, (void*)(&event_pkt->event_data[EVENTDATA_ACC_XOS_VALUE_OFFSET]), EVENTDATA_ACC_XOS_VALUE_SIZE);
                        memcpy((void*)&raw_y_os, (void*)(&event_pkt->event_data[EVENTDATA_ACC_YOS_VALUE_OFFSET]), EVENTDATA_ACC_YOS_VALUE_SIZE);
                        memcpy((void*)&raw_z_os, (void*)(&event_pkt->event_data[EVENTDATA_ACC_ZOS_VALUE_OFFSET]), EVENTDATA_ACC_ZOS_VALUE_SIZE);
                                
                        hid_data.x = static_cast<int16_t>(raw_x_os);
                        hid_data.y = static_cast<int16_t>(raw_y_os);
                        hid_data.z = static_cast<int16_t>(raw_z_os);
                        //hid_data.ts_low = (uint32_t)tstamp_us;
                        //hid_data.ts_high = (uint32_t)(tstamp_us >> 32);
                        hid_data.ts_low = (uint32_t)tstamp_ms;
                        hid_data.ts_high = (uint32_t)(tstamp_ms >> 32);
                                
                        sd.fo.pixels = &hid_data;
                        sd.fo.frame_size = sizeof(hid_data);
                        sd.fo.metadata = &meta_data;
                        sd.fo.metadata_size = metadata_hid_raw_size;
                        sd.fo.backend_time = tstamp_ms;
                                
                        uint16_t event_id = GevSwapWord(event_pkt->event_identifier);
                                
                        // check if EVENTDATA packet related to device accelerometer
                        if (event_id == _event_acc_id) {
                            // continue only if stream is configured for accelerometer data
                            if (_is_acc_configured) {
                                sd.sensor.name = accel_sensor_name;
                            }
                            else {
                                // drop event (log?)
                            }
                        }
                        // check if EVENTDATA packet related to device gyroscope
                        else if (event_id == _event_gyro_id) {
                            // continue only if stream is configured for gyroscope data
                            if (_is_gyro_configured) {
                                sd.sensor.name = gyro_sensor_name;
                            }
                            else {
                                // drop event (log?)
                            }
                        }
                    }
                }
                                
                // new valid sensor_data
                if (!sd.sensor.name.empty()) {
                    try {
                        _callback_event(sd);
                    }
                    catch (const std::bad_function_call& e) {
                        LOG_ERROR(e.what());                        
                    }
                    catch(...) {                    
                        LOG_ERROR("Motion event callback");
                    }

                }
            }
            else {
                throw camera_disconnected_exception("Waiting on Events from disconnected device!");
            }
        }
    }


    //////////////////////////////////////////////////////
    /////////////////// CS HID Sensor ////////////////////
    //////////////////////////////////////////////////////

    cs_hid_sensor::cs_hid_sensor(
        std::shared_ptr<platform::hid_device> hid_device,
        std::shared_ptr<platform::cs_device> cs_device,
        std::unique_ptr<frame_timestamp_reader> hid_iio_timestamp_reader,
        const std::map<rs2_stream, std::map<unsigned, unsigned>>& fps_and_sampling_frequency_per_rs2_stream,
        const std::vector<std::pair<std::string, stream_profile>>& sensor_name_and_hid_profiles,
        device* dev)
        : sensor_base("Raw Motion Module", dev, (recommended_proccesing_blocks_interface*)this),
        _sensor_name_and_hid_profiles(sensor_name_and_hid_profiles),
        _fps_and_sampling_frequency_per_rs2_stream(fps_and_sampling_frequency_per_rs2_stream),
        _hid_device(hid_device),
        _cs_device(cs_device),
        _is_configured_stream(RS2_STREAM_COUNT),
        _hid_iio_timestamp_reader(move(hid_iio_timestamp_reader))
    {
        std::map<rs2_stream, std::map<uint32_t, uint32_t>>::iterator it1 = _fps_and_sampling_frequency_per_rs2_stream.begin();
        while (it1 != _fps_and_sampling_frequency_per_rs2_stream.end()) {

            std::vector<uint32_t> fps_list;
            std::map<uint32_t, uint32_t>::iterator it2 = it1->second.begin();
            while (it2 != it1->second.end()) {

                uint32_t fps = static_cast<std::underlying_type<IMU_OUTPUT_DATA_RATES>::type>((*it2).first);
                fps_list.push_back(fps);
                it2++;
            }

            _fps_per_rs2_stream.insert(std::pair<rs2_stream, std::vector<uint32_t>>((*it1).first, fps_list));

            it1++;
        }

        register_metadata(RS2_FRAME_METADATA_BACKEND_TIMESTAMP, make_additional_data_parser(&frame_additional_data::backend_timestamp));

        std::map<std::string, uint32_t> frequency_per_sensor;
        for (auto&& elem : sensor_name_and_hid_profiles) {
            frequency_per_sensor.insert(make_pair(elem.first, elem.second.fps));
        }

        std::vector<platform::hid_profile> profiles_vector;
        for (auto&& elem : frequency_per_sensor) {
            profiles_vector.push_back(platform::hid_profile{ elem.first, elem.second });
        }

        _hid_device->register_profiles(profiles_vector);
        for (auto&& elem : _hid_device->get_sensors()) {
            _hid_sensors.push_back(elem);
        }
    }

    cs_hid_sensor::~cs_hid_sensor()
    {
        try
        {
            if (_is_streaming)
                stop();

            if (_is_opened)
                close();
        }
        catch (...)
        {
            LOG_ERROR("An error has occurred while stop_streaming()!");
        }
    }

    stream_profiles cs_hid_sensor::get_sensor_profiles(std::string sensor_name) const
    {
        stream_profiles profiles{};
        for (auto&& elem : _sensor_name_and_hid_profiles)
        {
            if (!elem.first.compare(sensor_name))
            {
                auto&& p = elem.second;
                platform::stream_profile sp{ 1, 1, p.fps, stream_to_fourcc(p.stream) };
                auto profile = std::make_shared<motion_stream_profile>(sp);
                profile->set_stream_index(p.index);
                profile->set_stream_type(p.stream);
                profile->set_format(p.format);
                profile->set_framerate(p.fps);
                profiles.push_back(profile);
            }
        }

        return profiles;
    }

    void cs_hid_sensor::open(const stream_profiles& requests)
    {
        std::lock_guard<std::mutex> lock(_configure_lock);
        if (_is_streaming) {
            throw wrong_api_call_sequence_exception("open(...) failed. CS Motion device is streaming!");
        }
        else if (_is_opened) {
            throw wrong_api_call_sequence_exception("open(...) failed. CS Motion device is already opened!");
        }

        std::vector<platform::hid_profile> configured_hid_profiles;
        for (auto&& request : requests) {
            auto&& sensor_name = rs2_stream_to_sensor_name(request->get_stream_type());
            _configured_profiles.insert(std::make_pair(sensor_name, request));
            _is_configured_stream[request->get_stream_type()] = true;
            configured_hid_profiles.push_back(platform::hid_profile{ sensor_name, fps_to_sampling_frequency(request->get_stream_type(), request->get_framerate()) });
        }
        _hid_device->open(configured_hid_profiles);

        // prepare cs device compatible event stream profiles
        cfg_event_stream cfg_event;
        _cfg_event_list.clear();
        for (auto&& request : requests) {

            // acc event stream profile
            if (request->get_stream_type() == RS2_STREAM_ACCEL) {

                cfg_event.clear();
                cfg_event.event_selector = XML_EVENT_SELECTOR_ACC;

                uint32_t f_rate = static_cast<std::underlying_type<IMU_OUTPUT_DATA_RATES>::type>(request->get_framerate());

                for (auto&& elem1 : _fps_per_rs2_stream) {
                    if (elem1.first == RS2_STREAM_ACCEL) {
                        for (auto elem2 : elem1.second) {
                            if (elem2 == f_rate) {
                                if (elem2 == EVENT_ACC_DATARATE_62_50_TO_RS) {
                                    cfg_event.is_valid = true;
                                    cfg_event.event_data_rate = XML_EVENT_ACC_DATARATE_31_25_HZ;
                                    break;
                                }
                                else if (elem2 == EVENT_ACC_DATARATE_250_TO_RS) {
                                    cfg_event.is_valid = true;
                                    cfg_event.event_data_rate = XML_EVENT_ACC_DATARATE_125_HZ;
                                    break;
                                }
                                else if (elem2 == EVENT_ACC_DATARATE_100_TO_RS) {
                                    cfg_event.is_valid = true;
                                    cfg_event.event_data_rate = XML_EVENT_ACC_DATARATE_100_HZ;
                                    break;
                                }
                                else if (elem2 == EVENT_ACC_DATARATE_200_TO_RS) {
                                    cfg_event.is_valid = true;
                                    cfg_event.event_data_rate = XML_EVENT_ACC_DATARATE_200_HZ;
                                    break;
                                }
                            }
                        }
                        break;
                    }
                }

                if (cfg_event.is_valid) {
                    _cfg_event_list.push_back(cfg_event);
                }
            }
            // gyro event stream profile
            else if (request->get_stream_type() == RS2_STREAM_GYRO) {

                cfg_event.clear();
                cfg_event.event_selector = XML_EVENT_SELECTOR_GYRO;

                uint32_t f_rate = static_cast<std::underlying_type<IMU_OUTPUT_DATA_RATES>::type>(request->get_framerate());

                for (auto&& elem1 : _fps_per_rs2_stream) {
                    if (elem1.first == RS2_STREAM_GYRO) {
                        for (auto elem2 : elem1.second) {
                            if (elem2 == f_rate) {
                                if (elem2 == EVENT_GYRO_DATARATE_200_TO_RS) {
                                    cfg_event.is_valid = true;
                                    cfg_event.event_data_rate = XML_EVENT_GYRO_DATARATE_200_HZ;
                                    break;
                                }
                                else if (elem2 == EVENT_GYRO_DATARATE_400_TO_RS) {
                                    cfg_event.is_valid = true;
                                    cfg_event.event_data_rate = XML_EVENT_GYRO_DATARATE_400_HZ;
                                    break;
                                }
                            }
                        }
                        break;
                    }
                }

                if (cfg_event.is_valid) {
                    _cfg_event_list.push_back(cfg_event);
                }
            }
        }

        unsigned long long last_frame_number = 0;
        rs2_time_t last_timestamp = 0;

        _cs_device->open_event(
            [this, last_frame_number, last_timestamp](const platform::sensor_data& sensor_data) mutable
        {
            const auto&& system_time = environment::get_instance().get_time_service()->get_time();
            auto timestamp_reader = _hid_iio_timestamp_reader.get();
            auto&& sensor_name = sensor_data.sensor.name;
            auto&& request = _configured_profiles[sensor_name];

            if (!this->is_streaming()) {
                auto stream_type = request->get_stream_type();
                LOG_WARNING("CS Motion Frame received when Streaming is not active," << get_string(stream_type) << ",Arrived," << std::fixed << system_time);
                return;
            }

            const auto&& fr = generate_frame_from_data(sensor_data.fo, timestamp_reader, last_timestamp, last_frame_number, request);
            auto&& frame_counter = fr->additional_data.frame_number;
            const auto&& timestamp_domain = timestamp_reader->get_frame_timestamp_domain(fr);
            auto&& timestamp = fr->additional_data.timestamp;
            const auto&& bpp = get_image_bpp(request->get_format());
            auto&& data_size = sensor_data.fo.frame_size;

            LOG_DEBUG(
                "FrameAccepted," << get_string(request->get_stream_type()) <<
                ",Counter," << std::dec << frame_counter << ",Index,0" <<
                ",BackEndTS," << std::fixed << sensor_data.fo.backend_time <<
                ",SystemTime," << std::fixed << system_time <<
                " ,diff_ts[Sys-BE]," << system_time - sensor_data.fo.backend_time <<
                ",TS," << std::fixed << timestamp << ",TS_Domain," << rs2_timestamp_domain_to_string(timestamp_domain) <<
                ",last_frame_number," << last_frame_number << ",last_timestamp," << last_timestamp);

            last_frame_number = frame_counter;
            last_timestamp = timestamp;
            frame_holder frame = _source.alloc_frame(RS2_EXTENSION_MOTION_FRAME, data_size, fr->additional_data, true);
            memcpy((void*)frame->get_frame_data(), fr->data.data(), sizeof(byte)*fr->data.size());

            if (!frame) {
                LOG_INFO("Dropped frame. alloc_frame(...) returned nullptr");
                return;
            }
            frame->set_stream(request);
            frame->set_timestamp_domain(timestamp_domain);
            _source.invoke_callback(std::move(frame));
        });

        if (Is<librealsense::global_time_interface>(_owner)) {
            As<librealsense::global_time_interface>(_owner)->enable_time_diff_keeper(true);
        }

        _is_opened = true;
        set_active_streams(requests);
    }

    void cs_hid_sensor::close()
    {
        std::lock_guard<std::mutex> lock(_configure_lock);
        if (_is_streaming) {
            throw wrong_api_call_sequence_exception("close() failed. CS Motion device is streaming!");
        }
        else if (!_is_opened) {
            throw wrong_api_call_sequence_exception("close() failed. CS Motion device was not opened!");
        }

        _cs_device->stop_event();
        _cs_device->close_event();
        _hid_device->stop_capture();
        _hid_device->close();

        _source.flush();
        _source.reset();
        _hid_iio_timestamp_reader->reset();
        _configured_profiles.clear();
        _is_configured_stream.clear();
        _is_configured_stream.resize(RS2_STREAM_COUNT);
        _is_opened = false;

        if (Is<librealsense::global_time_interface>(_owner)) {
            As<librealsense::global_time_interface>(_owner)->enable_time_diff_keeper(false);
        }
        set_active_streams({});
    }

    void cs_hid_sensor::start(frame_callback_ptr callback)
    {
        std::lock_guard<std::mutex> lock(_configure_lock);
        if (_is_streaming) {
            throw wrong_api_call_sequence_exception("start_streaming(...) failed. CS Motion device is already streaming!");
        }
        else if (!_is_opened) {
            throw wrong_api_call_sequence_exception("start_streaming(...) failed. CS Motion device was not opened!");
        }

        _source.set_callback(callback);
        _source.init(_metadata_parsers);
        _source.set_sensor(_source_owner->shared_from_this());

        unsigned long long last_frame_number = 0;
        rs2_time_t last_timestamp = 0;
        raise_on_before_streaming_changes(true); //Required to be just before actual start allow recording to work

        _cs_device->start_event(
            _cfg_event_list,
            [&](const notification& n)
        {
            _notifications_processor->raise_notification(n);
        });

        _is_streaming = true;
    }

    void cs_hid_sensor::stop()
    {
        std::lock_guard<std::mutex> lock(_configure_lock);
        if (!_is_streaming) {
            throw wrong_api_call_sequence_exception("stop_streaming() failed. CS Motion device is not streaming!");
        }

        _is_streaming = false;
        raise_on_before_streaming_changes(false);
    }

    stream_profiles cs_hid_sensor::init_stream_profiles()
    {
        stream_profiles stream_requests;
        for (auto&& it = _hid_sensors.rbegin(); it != _hid_sensors.rend(); ++it) {
            auto profiles = get_sensor_profiles(it->name);
            stream_requests.insert(stream_requests.end(), profiles.begin(), profiles.end());
        }

        return stream_requests;
    }

    const std::string& cs_hid_sensor::rs2_stream_to_sensor_name(rs2_stream stream) const
    {
        for (auto&& elem : _sensor_name_and_hid_profiles) {
            if (stream == elem.second.stream) {
                return elem.first;
            }
        }

        throw invalid_value_exception("rs2_stream not found!");
    }

    uint32_t cs_hid_sensor::stream_to_fourcc(rs2_stream stream) const
    {
        uint32_t fourcc;
        try {
            fourcc = stream_and_fourcc.at(stream);
        }
        catch (std::out_of_range) {
            throw invalid_value_exception(to_string() << "fourcc of stream " << rs2_stream_to_string(stream) << " not found!");
        }

        return fourcc;
    }

    uint32_t cs_hid_sensor::fps_to_sampling_frequency(rs2_stream stream, uint32_t fps) const
    {
        // TODO: Add log prints
        auto it = _fps_and_sampling_frequency_per_rs2_stream.find(stream);
        if (it == _fps_and_sampling_frequency_per_rs2_stream.end()) {
            return fps;
        }

        auto fps_mapping = it->second.find(fps);
        if (fps_mapping != it->second.end()) {
            return fps_mapping->second;
        }
        else {
            return fps;
        }
    }
}