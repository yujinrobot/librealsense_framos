// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 FRAMOS GmbH.

#include "cs/cs-sensor.h"
#include "cs/cs-options.h"
#include "cs/cs-options-default.h"
#include "global_timestamp_reader.h"
#include "stream.h"
#include "device.h"

//#include "../../../third-party/json.hpp"

#include <array>
#include <set>
#include <unordered_set>
#include <iomanip>

namespace librealsense {

    cs_firmware_version::cs_firmware_version(smcs::IDevice &device)
        : _major(0), _minor(0), _patch(0), _build(0)
    {
        auto device_version = device->GetDeviceVersion();        
        std::string delimiter = "FW:";
        device_version.erase(0, device_version.find(delimiter) + delimiter.length());

        std::vector<std::string> token;
        delimiter = ".";
        size_t pos = 0;
        while ((pos = device_version.find(delimiter)) != std::string::npos) {
            token.push_back(device_version.substr(0, pos));
            device_version.erase(0, pos + delimiter.length());
        }
        token.push_back(device_version);
        
        try
        {
            _major = std::stoi(token[0]);
            _minor = std::stoi(token[1]);
            _patch = std::stoi(token[2]);
            _build = std::stoi(token[3]);
        }
        catch (...)
        {

        }
    }

    cs_sensor::cs_sensor(std::string name,
                         std::shared_ptr<platform::cs_device> cs_device,
                         std::unique_ptr<frame_timestamp_reader> timestamp_reader,
                         device* dev,
                         cs_stream stream)
            : sensor_base(name, dev, (recommended_proccesing_blocks_interface*)this),
              _device(std::move(cs_device)),
              _timestamp_reader(std::move(timestamp_reader)),
              _cs_stream(stream),
              _user_count(0),
              _external_trigger_mode(false)
    {
        register_metadata(RS2_FRAME_METADATA_BACKEND_TIMESTAMP,     make_additional_data_parser(&frame_additional_data::backend_timestamp));
    }

    cs_sensor::~cs_sensor()
    {
        try
        {
            if (_is_streaming)
                cs_sensor::stop();

            if (_is_opened)
                cs_sensor::close();
        }
        catch(...)
        {
            LOG_ERROR("An error has occurred while stop_streaming()!");
        }
    }

    void cs_sensor::open(const stream_profiles& requests)
    {
        std::lock_guard<std::mutex> lock(_configure_lock);

        _device->toggle_raw16_flag(false);
        _device->toggle_calibration_stream_flag(false);
        for (auto&& r_profile : requests) {
            if (r_profile->get_stream_type() == RS2_STREAM_COLOR) {
                if (r_profile->get_format() == RS2_FORMAT_RAW16) {
                    _device->toggle_raw16_flag(true);
                    break;
                }
            }
            if (r_profile->get_stream_type() == RS2_STREAM_INFRARED) {
                if (r_profile->get_format() == RS2_FORMAT_Y12I) {
                    if (auto vp = dynamic_cast<video_stream_profile_interface*>(r_profile.get())) {
                        if (vp->get_height() == 800 && vp->get_width() == 1280) {
                            _device->toggle_calibration_stream_flag(true);
                            break;
                        }
                        if (vp->get_height() == 1080 && vp->get_width() == 1920) {
                            _device->toggle_calibration_stream_flag(true);
                            break;
                        }
                    }
                }
            }
        }

        if (_is_streaming)
            throw wrong_api_call_sequence_exception("open(...) failed. CS device is streaming!");
        else if (_is_opened)
            throw wrong_api_call_sequence_exception("open(...) failed. CS device is already opened!");

        auto on = std::unique_ptr<power>(new power(std::dynamic_pointer_cast<cs_sensor>(shared_from_this())));

        if (!smcs::GetCameraAPI()->IsUsingKernelDriver())
            LOG_WARNING("GEV filter driver not loaded! Socket driver will be used.");

        _source.init(_metadata_parsers);
        _source.set_sensor(_source_owner->shared_from_this());

        std::vector<platform::stream_profile> commited;

        _cs_selected_streams.clear();

        for (auto&& req_profile : requests)
        {
            auto&& req_profile_base = std::dynamic_pointer_cast<stream_profile_base>(req_profile);
            auto selected_stream = get_stream(req_profile->get_stream_type(), req_profile->get_stream_index());

            try
            {
                auto infrared_stream = selected_stream == CS_STREAM_IR_LEFT || selected_stream == CS_STREAM_IR_RIGHT || selected_stream == CS_STREAM_IR_LEFT_COLOR;
                if (infrared_stream && !_device->is_infrared_supported())
                    throw wrong_api_call_sequence_exception("Device does not support infrared streams!");

                unsigned long long last_frame_number = 0;
                rs2_time_t last_timestamp = 0;
                _device->probe_and_commit(req_profile_base->get_backend_profile(),
                                          [this, req_profile_base, req_profile, last_frame_number, last_timestamp](platform::stream_profile p, platform::frame_object f, std::function<void()> continuation) mutable
                {
                    const auto&& system_time = environment::get_instance().get_time_service()->get_time();
                    const auto&& fr = generate_frame_from_data(f, _timestamp_reader.get(), last_timestamp, last_frame_number, req_profile_base);
                    const auto&& requires_processing = true; // TODO - Ariel add option
                    const auto&& timestamp_domain = _timestamp_reader->get_frame_timestamp_domain(fr);
                    const auto && bpp = get_image_bpp(req_profile_base->get_format());
                    auto&& frame_counter = fr->additional_data.frame_number;
                    auto&& timestamp = fr->additional_data.timestamp;

                    if (!this->is_streaming())
                    {
                        LOG_WARNING("Frame received with streaming inactive,"
                                            << librealsense::get_string(req_profile_base->get_stream_type())
                                            << req_profile_base->get_stream_index()
                                            << ", Arrived," << std::fixed << f.backend_time << " " << system_time);
                        return;
                    }

                    frame_continuation release_and_enqueue(continuation, f.pixels);

                    LOG_DEBUG("SN:" << this->get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)
                                               << "\n    FrameAccepted, " << librealsense::get_string(req_profile_base->get_stream_type())
                                               << "\n    Counter, " << std::dec << fr->additional_data.frame_number
                                               << "\n    Index, " << req_profile_base->get_stream_index()
                                               << "\n    BackEndTS, " << std::fixed << f.backend_time
                                               << "\n    SystemTime, " << std::fixed << system_time
                                               << "\n    diff_ts[Sys-BE], " << system_time - f.backend_time
                                               << "\n    TS, " << std::fixed << timestamp
                                               << "\n    TS_Domain," << rs2_timestamp_domain_to_string(timestamp_domain)
                                               << "\n    last_frame_number, " << last_frame_number
                                               << "\n    last_timestamp, " << last_timestamp);
               
                    last_frame_number = frame_counter;
                    last_timestamp = timestamp;

                    const auto&& vsp = As<video_stream_profile, stream_profile_interface>(req_profile);
                    int width = vsp ? vsp->get_width() : 0;
                    int height = vsp ? vsp->get_height() : 0;
                    frame_holder fh = _source.alloc_frame(stream_to_frame_types(req_profile_base->get_stream_type()), (size_t)(width * height * bpp / 8.0), fr->additional_data, requires_processing);
                    if (fh.frame)
                    {
                        memcpy((void*)fh->get_frame_data(), fr->data.data(), sizeof(byte)*fr->data.size());
                        auto&& video = (video_frame*)fh.frame;
                        video->assign(width, height, width * bpp / 8, bpp);
                        video->set_timestamp_domain(timestamp_domain);
                        fh->set_stream(req_profile_base);
                    }
                    else
                    {
                        LOG_INFO("Dropped frame. alloc_frame(...) returned nullptr");
                        return;
                    }

                    if (!requires_processing)
                    {
                        fh->attach_continuation(std::move(release_and_enqueue));
                    }

                    if (fh->get_stream().get())
                    {
                        _source.invoke_callback(std::move(fh));
                    }

                }, selected_stream);
            }
            catch (...)
            {
                _device->close(_cs_selected_streams);
                throw;
            }
            _cs_selected_streams.push_back(selected_stream);
            commited.push_back(req_profile_base->get_backend_profile());
        }

        _internal_config = commited;

        if (_on_open)
            _on_open(_internal_config);

        _power = move(on);
        _is_opened = true;

        try {
            _device->stream_on([&](const notification& n)
                               {
                                   _notifications_processor->raise_notification(n);
                               }, _cs_selected_streams, _internal_config);
        }
        catch (...)
        {
            std::stringstream error_msg;
            error_msg << "\tFormats: \n";
            for (auto&& profile : _internal_config)
            {
                rs2_format fmt = fourcc_to_rs2_format(profile.format);
                error_msg << "\t " << std::string(rs2_format_to_string(fmt)) << std::endl;
                try {
                    _device->close(_cs_selected_streams);
                }
                catch (...) {}
            }
            error_msg << std::endl;
            reset_streaming();
            _power.reset();
            _is_opened = false;

            throw std::runtime_error(error_msg.str());
        }

        if (Is<librealsense::global_time_interface>(_owner))
        {
            As<librealsense::global_time_interface>(_owner)->enable_time_diff_keeper(true);
        }
        set_active_streams(requests);
    }

    stream_profiles cs_sensor::init_stream_profiles()
    {
        auto lock = environment::get_instance().get_extrinsics_graph().lock();

        std::unordered_set<std::shared_ptr<video_stream_profile>> profiles;
        power on(std::dynamic_pointer_cast<cs_sensor>(shared_from_this()));

        _uvc_profiles = _device->get_profiles(_cs_stream);

        for (auto&& p : _uvc_profiles)
        {
            const auto&& rs2_fmt = fourcc_to_rs2_format(p.format);
            if (rs2_fmt == RS2_FORMAT_ANY)
                continue;

            auto&& profile = std::make_shared<video_stream_profile>(p);
            profile->set_dims(p.width, p.height);
            profile->set_stream_type(fourcc_to_rs2_stream(p.format));
            profile->set_stream_index(0);
            profile->set_format(rs2_fmt);
            profile->set_framerate(p.fps);
            profiles.insert(profile);
        }

        stream_profiles result{ profiles.begin(), profiles.end() };
        return result;
    }

    rs2_extension cs_sensor::stream_to_frame_types(rs2_stream stream)
    {
        // TODO: explicitly return video_frame for relevant streams and default to an error?
        switch (stream)
        {
            case RS2_STREAM_DEPTH:  return RS2_EXTENSION_DEPTH_FRAME;
            default:                return RS2_EXTENSION_VIDEO_FRAME;
        }
    }

    void cs_sensor::close()
    {
        std::lock_guard<std::mutex> lock(_configure_lock);
        if (_is_streaming)
            throw wrong_api_call_sequence_exception("close() failed. CS device is streaming!");
        else if (!_is_opened)
            throw wrong_api_call_sequence_exception("close() failed. CS device was not opened!");

        for (auto&& profile : _internal_config)
        {
            try // Handle disconnect event
            {
                _device->close(_cs_selected_streams);
            }
            catch (...) 
            {
                LOG_ERROR("Error while closing stream!");
                throw;
            }
        }
        reset_streaming();
        if (Is<librealsense::global_time_interface>(_owner))
        {
            As<librealsense::global_time_interface>(_owner)->enable_time_diff_keeper(false);
        }
        _power.reset();
        _is_opened = false;
        set_active_streams({});
    }

    void cs_sensor::start(frame_callback_ptr callback)
    {
        std::lock_guard<std::mutex> lock(_configure_lock);
        if (_is_streaming)
            throw wrong_api_call_sequence_exception("start_streaming(...) failed. CS device is already streaming!");
        else if(!_is_opened)
            throw wrong_api_call_sequence_exception("start_streaming(...) failed. CS device was not opened!");

        raise_on_before_streaming_changes(true); //Required to be just before actual start allow recording to work
        _source.set_callback(callback);
        _is_streaming = true;
    }

    void cs_sensor::stop()
    {
        std::lock_guard<std::mutex> lock(_configure_lock);
        if (!_is_streaming)
            throw wrong_api_call_sequence_exception("stop_streaming() failed. CS device is not streaming!");

        _is_streaming = false;
        raise_on_before_streaming_changes(false);
    }

    void cs_sensor::acquire_power()
    {
        std::lock_guard<std::mutex> lock(_power_lock);
        if (_user_count.fetch_add(1) == 0)
        {
            if (_device->set_power_state(platform::D0) != platform::D0)
                throw wrong_api_call_sequence_exception("set power state(...) failed. CS device cannot be turned on!");
            //for (auto&& xu : _xus) _device->init_xu(xu);
        }
    }
	

    void cs_sensor::release_power()
    {
        std::lock_guard<std::mutex> lock(_power_lock);
        if (_user_count.fetch_add(-1) == 1)
        {
            if (_device->set_power_state(platform::D3) != platform::D3)
                throw wrong_api_call_sequence_exception("set power state(...) failed. CS device cannot be turned off!");
        }
    }

    void cs_sensor::reset_streaming()
    {
        _source.flush();
        _source.reset();
        _timestamp_reader->reset();
    }

    cs_stream cs_sensor::get_stream(rs2_stream type, int index)
    {
        switch (type)
        {
        case RS2_STREAM_DEPTH:
            return CS_STREAM_DEPTH;
        case RS2_STREAM_COLOR:
            return CS_STREAM_COLOR;
        case RS2_STREAM_INFRARED:
            if (index == 0)
                return CS_STREAM_IR_LEFT_COLOR;
            else
                return index == 1 ? CS_STREAM_IR_LEFT : CS_STREAM_IR_RIGHT;
        default:
            throw wrong_api_call_sequence_exception("unable to map type and index to stream ID!");
        }
    }

    void cs_sensor::register_pu(rs2_option id)
    {
        register_option(id, std::make_shared<cs_pu_option>(*this, id, _cs_stream));
    }

    void cs_sensor::try_register_pu(rs2_option id)
    {
        auto opt = std::make_shared<cs_pu_option>(*this, id, _cs_stream);
        try
        {
            auto range = opt->get_range();
            if (range.max <= range.min || range.step <= 0 || range.def < range.min || range.def > range.max) return;

            auto val = opt->query();
            if (val < range.min || val > range.max) return;
            opt->set(val);

            register_option(id, opt);
        }
        catch (...)
        {
            LOG_WARNING("Exception was thrown when inspecting " << this->get_info(RS2_CAMERA_INFO_NAME) << " property " << opt->get_description());
        }
    }

	void cs_sensor::set_inter_cam_sync_mode(float value, bool gs)
	{
        if (_is_streaming)
            throw wrong_api_call_sequence_exception("Unable to set Inter Cam Sync Mode while streaming!");

        _device->update_external_trigger_mode_flag(_cs_stream, value);
        if (gs)
            _device->set_trigger_mode_gs(value, _cs_stream);
        else
            _device->set_trigger_mode(value, _cs_stream);

	}

    float cs_sensor::get_inter_cam_sync_mode(bool gs)
    {
        float inter_cam_sync_mode = 0.0;
        if (gs)
            inter_cam_sync_mode = _device->get_trigger_mode_gs(_cs_stream);
        else
            inter_cam_sync_mode = _device->get_trigger_mode(_cs_stream);

        _device->update_external_trigger_mode_flag(_cs_stream, inter_cam_sync_mode);

        return inter_cam_sync_mode;
    }

    bool cs_sensor::query_inter_cam_sync_mode(bool gs)
    {
        return _device->get_intercam_mode(_cs_stream);
    }

    namespace platform
    {
        std::map<std::string, int> cs_device::_cs_device_num_objects_sn;
        std::map<std::string, bool> cs_device::_cs_device_initialized_sn;
        std::map<std::string, bool> cs_device::_cs_device_option_sw_trigger_all_flag_sn;
        double cs_device::_statistics_last_call_time_driver;

        cs_device::cs_device(cs_device_info hwm)
                : _device_info(std::move(hwm)),
                    _power_state(D3),
                    _connected_device(NULL),
                    _rgb_pixel_format(RS2_FORMAT_ANY),
                    _infrared_supported(false),
                    _temperature_supported_checked(false),
                    _temperature_supported(false),
                    _software_trigger_supported_checked(false),
                    _software_trigger_supported(false),
                    _line_debouncer_time_supported_checked(false),
                    _line_debouncer_time_supported(false),
                    _rgb_raw16_supported_checked(false),
                    _rgb_raw16_supported(false),
                    _imu_supported_checked(false),
                    _imu_supported(false),
                    _metadata_supported_checked(false),
                    _metadata_supported(false),
                    _selected_source(0),
                    _selected_source_initialized(false),
                    _is_calibration_stream(false),
                    _is_raw16(false),
                    _hwm_tx_once_flag(false),
                    _hwm_tx_reg_addr(0),
                    _hwm_tx_reg_len(0),
                    _hwm_rx_once_flag(false),
                    _hwm_rx_reg_addr(0),
                    _hwm_rx_reg_len(0),
                    _calib_tx_once_flag(false),
                    _calib_tx_reg_addr(0),
                    _calib_tx_reg_len(0),
                    _is_acc_configured(false),
                    _is_gyro_configured(false),
                    _burst_mode_supported_checked(false),
                    _burst_mode_supported(false),
                    _color_genlock_mode_supported(false),
                    _color_genlock_mode_supported_checked(false),
                    _is_full_slave_mode(false),
                    _rgb_ae_roi_hwm_size(6 * sizeof(uint32_t)),
                    _rgb_led_toggle_supported_checked(false) {
            _smcs_api = smcs::GetCameraAPI();
            auto devices = _smcs_api->GetAllDevices();

            _metadata_toggle_value = false;

            if (_smcs_api->IsUsingKernelDriver()) {
                LOG_INFO("GEV filter driver initial api packet statistics: " << get_api_packet_statistics());
            }
            else {
                LOG_INFO("GEV socket driver initial api packet statistics: " << get_api_packet_statistics());
            }

            _md_size = FRAMOS_METADATA_SIZE;
            uint32_t md_sz = sizeof(md_capture_timing) + sizeof(md_capture_stats) + sizeof(md_depth_control) + sizeof(md_configuration);
            if (_md_size > md_sz) {
                _md_size = md_sz;
            }

            for (int i = 0; i < devices.size(); i++) {
                auto serial = devices[i]->GetSerialNumber();
                if (!serial.compare(_device_info.serial)) {
                    _connected_device = devices[i];

                    if (_connected_device == NULL) {
                        break;
                    } else {
                        if (!_connected_device->IsConnected()) {
                            // try to connect to camera if not already connected
                            if (!_connected_device->Connect()) {
                                throw wrong_api_call_sequence_exception(std::string("Could not connect to device with serial number ") + _connected_device->GetSerialNumber());
                            }
                            else {
                                // initialize this part (inter-packet delay) only once
                                if (get_device_init_flag_sn(_device_info.serial) == false) { 
                                    INT64 stream_count, link_speed_mbitps;
                                    _connected_device->GetIntegerNodeValue("GevStreamChannelCount", stream_count);
                                    _connected_device->GetIntegerNodeValue("GevLinkSpeed", link_speed_mbitps);
                                    for (int i = 0; i < stream_count; i++)
                                    {
                                        INT64 packet_size_bytes;
                                        select_channel((cs_stream)i);

                                        _connected_device->GetIntegerNodeValue("GevSCPSPacketSize", packet_size_bytes);
                                        INT64 inter_packet_delay_us = get_optimal_inter_packet_delay(packet_size_bytes, link_speed_mbitps);
                                        _connected_device->SetIntegerNodeValue("GevSCPD", inter_packet_delay_us);
                                    }                                
                                    set_device_init_flag_sn(_device_info.serial, true);
                                }
                            }
                        }

                        _number_of_streams = CS_STREAM_COUNT;
                        _threads = std::vector<std::unique_ptr <std::thread>>(_number_of_streams);
                        _is_capturing = std::vector<std::atomic<bool>>(_number_of_streams);
                        _callbacks = std::vector<frame_callback>(_number_of_streams);
                        _error_handler = std::vector<std::function<void(const notification &n)>>(_number_of_streams);
                        _profiles = std::vector<stream_profile>(_number_of_streams);
                        _cs_firmware_version = cs_firmware_version(_connected_device);

                        constexpr double s_to_ms_factor = 1000;
                        INT64 frequency;
                        if (!_connected_device->GetIntegerNodeValue("GevTimestampTickFrequency", frequency))
                            throw io_exception("Unable to read GevTimestampTickFrequency");
                        _timestamp_to_ms_factor = s_to_ms_factor / frequency;

                        for (int i = 0; i < _number_of_streams; i++)
                        {
                            _threads[i] = nullptr;
                            _is_capturing[i] = false;
                            _callbacks[i] = nullptr;

                            // make sure stream parameters are unlocked
                            stream_params_unlock((cs_stream)i);
                        }

                        _is_event_capturing = false;
                        if (is_imu_supported()) {
                            if (!configure_message_channel(_connected_device)) {
                                throw io_exception("Unable to configure Device message channel");
                            }
                        }

                        get_addresses_of_frequent_commands(_connected_device, _reg_map);

                        // increment device counter for device with current SN
                        inc_device_count_sn(_device_info.serial);
                    }
                    // found device with SN
                    break;
                }
            }
            if (_connected_device == NULL) {    // Could not find device with given SN
                throw wrong_api_call_sequence_exception("Could not create CS device with given SN!");
            }
        }

        cs_device::~cs_device() {
            dec_device_count_sn(_device_info.serial);
            for (int i = 0; i < _number_of_streams; i++) {
                deinit_stream((cs_stream)i);
            }
            
            if (get_device_count_sn(_device_info.serial) == 0) {
                if (_connected_device->IsConnected()) {
                    try {
                        //TODO - close all streams
                        close({ CS_STREAM_DEPTH , CS_STREAM_COLOR }); // -> Source0, Source1
                        stop_event();
                        close_event();
                    }
                    catch (const std::exception& ex) {
                        LOG_WARNING(ex.what());
                    }
                    _connected_device->Disconnect();
                }
            }
            if (_smcs_api->IsUsingKernelDriver()) {
                LOG_INFO("GEV filter driver api packet statistics: " << get_api_packet_statistics());
            }
            else {
                LOG_INFO("GEV socket driver api packet statistics: " << get_api_packet_statistics());
            }
        }

        void cs_device::get_addresses_of_frequent_commands(smcs::IDevice& connected_device, RegMap& reg_map)
        {
            /* Cache register adresses of commands that are frequently called.
            *  This is done so that genicam mechanism is unburdened from frequent calls. */
            const std::vector<std::pair<std::string, std::string>> frequentlyUsedNodes = {
                { "GEV_BOOTSTRAP_GevTimestampControl", "GevTimestampControlLatch" },
                { "GEV_BOOTSTRAP_GevTimestampValueHigh", "GevTimestampValueHigh" },
                { "GEV_BOOTSTRAP_GevTimestampValueLow", "GevTimestampValueLow" },
                { "MetadataToggleReg", "MetadataToggle" },
                { "IsMetadataToggleAvailable", "IsMetadataToggleAvailable" }
            };

            for (const std::pair<std::string, std::string> nodeNamePair : frequentlyUsedNodes) {
                UINT64 address;
                bool getAddrStatus = _connected_device->GetNode(nodeNamePair.first)->GetRegisterNodeAddress(address);

                if (getAddrStatus) {
                    reg_map.insert({ nodeNamePair.second, address });
                }
            }
        }

        bool cs_device::is_reg_address_cached(const std::string& name, const RegMap& reg_map)
        {
            return (reg_map.find(name) != reg_map.end());
        }

        std::vector<stream_profile> cs_device::get_profiles(cs_stream stream)
        {
            std::vector<stream_profile> all_stream_profiles;
            stream_profile profile;
            INT64 int64_value;
            bool is_successful = true;

            if (!set_source_locked(stream, false))
                throw wrong_api_call_sequence_exception("Unable to read profiles!");
            
            if (!disable_source_regions(stream))
                throw wrong_api_call_sequence_exception("Unable to read profiles!");
            
            if (!set_region(stream, true))
                throw wrong_api_call_sequence_exception("Unable to read profiles!");

            if (stream == CS_STREAM_COLOR) {
                std::string pixel_format;
                if (_connected_device->GetStringNodeValue("PixelFormat", pixel_format)
                    && pixel_format.compare("YUV422Packed") == 0)
                    _rgb_pixel_format = RS2_FORMAT_BGR8;
                else
                    _rgb_pixel_format = RS2_FORMAT_RGB8;
            }

            if (stream == CS_STREAM_DEPTH) {
                smcs::StringList regions;
                _infrared_supported =
                    _connected_device->GetEnumNodeValuesList("RegionSelector", regions)
                    && regions.size() >= 4;
            }

            if (stream == CS_STREAM_DEPTH) {
                smcs::StringList regions;
                _infrared_left_color_supported =
                    _connected_device->GetEnumNodeValuesList("RegionSelector", regions)
                    && regions.size() >= 5;
            }

            smcs::StringList resolution_list;
            is_successful = is_successful & _connected_device->GetEnumNodeValuesList("Resolution", resolution_list);

            for (const auto& resolution : resolution_list) {

                // Resolution is read-only on D435e with FW 1.3.4.0
                std::string old_resolution;
                if (_connected_device->GetStringNodeValue("Resolution", old_resolution) && old_resolution != resolution)
                    is_successful = is_successful & _connected_device->SetStringNodeValue("Resolution", resolution);

                is_successful = is_successful & _connected_device->GetIntegerNodeValue("Width", int64_value);
                profile.width = (uint32_t) int64_value;

                is_successful = is_successful & _connected_device->GetIntegerNodeValue("Height", int64_value);
                profile.height = (uint32_t) int64_value;

                std::string pixelFormat;
                is_successful = is_successful & _connected_device->GetStringNodeValue("PixelFormat", pixelFormat);

                for (auto frameRate : get_frame_rates()) {

                    profile.fps = frameRate;

                    if (is_successful) {
                        profile.format = cs_pixel_format_to_native_pixel_format(pixelFormat);
                        all_stream_profiles.push_back(profile);
            
                        /**
                         * Adding profiles for IR streams.
                         * GREY is for the left IR stream.
                         * Y8I is for the right IR stream.
                         * GREY must be added before Y8I. Order is important because 
                         * only the first compatible format is selected.
                         */
                        if (is_successful && get_stream_source(stream) == 0) {
                            profile.format = rs_fourcc('G', 'R', 'E', 'Y');
                            all_stream_profiles.push_back(profile);
                            profile.format = rs_fourcc('Y', '8', 'I', ' ');
                            all_stream_profiles.push_back(profile);
                        }
                    }

                    // IR1 left stream
                    if ((stream == CS_STREAM_DEPTH) && is_infrared_left_color_supported()) {
                        profile.format = rs_fourcc('Y', 'U', 'Y', 'V');
                        all_stream_profiles.push_back(profile);
                    }
                    
                    all_stream_profiles.push_back(profile);
                }
            }

            set_region(stream, false);

            // rgb raw16
            if ((stream == CS_STREAM_COLOR) && 
                (is_rgb_raw16_supported())) {
                    if (_device_info.id == CS_CAMERA_MODEL_D455e) {
                        profile.width = 1280;
                        profile.height = 720;
                    } else {
                        profile.width = 1920;
                        profile.height = 1080;
                    }
                    profile.format = rs_fourcc('B', 'Y', 'R', '2');
                    /**/
                    profile.fps = 30;
                    all_stream_profiles.push_back(profile);
                    /**/
                    profile.fps = 15;
                    all_stream_profiles.push_back(profile);
                    /**/
                    profile.fps = 6;
                    all_stream_profiles.push_back(profile);
            }

            //Calibration streams
            if (stream == CS_STREAM_DEPTH) {
                if (_device_info.id == CS_CAMERA_MODEL_D455e ||
                    _device_info.id == CS_CAMERA_MODEL_D435e) {
                    profile.width = 1280;
                    profile.height = 800;

                    /*Right IR stream*/
                    profile.format = rs_fourcc('Y', '1', '2', 'I');
                    profile.fps = 25;
                    all_stream_profiles.push_back(profile);

                    /**/
                    profile.fps = 15;
                    all_stream_profiles.push_back(profile);
                }
                else if (_device_info.id == CS_CAMERA_MODEL_D415e) {
                    profile.width = 1920;
                    profile.height = 1080;

                    /*Right IR stream*/
                    profile.format = rs_fourcc('Y', '1', '2', 'I');
                    profile.fps = 25;
                    all_stream_profiles.push_back(profile);
                }
            }

            return all_stream_profiles;
        }

        std::vector<uint32_t> cs_device::get_frame_rates()
        {
            // FrameRate does not exist on D435e with FW 1.3.4.0
            std::vector<uint32_t> frame_rates;
            if (get_frame_rates_from_control(frame_rates))
                return frame_rates;
            else
                return std::vector<uint32_t> {30};
        }

        bool cs_device::get_frame_rates_from_control(std::vector<uint32_t> &frame_rates)
        {
            std::string frameRateValue;
            if (!_connected_device->GetStringNodeValue("FrameRate", frameRateValue))
                return false;

            smcs::StringList frameRates;
            if (!_connected_device->GetEnumNodeValuesList("FrameRate", frameRates))
                return false;

            std::vector<uint32_t> acquisitionFrameRates;
            for (const auto& frameRate : frameRates) {
                if (!_connected_device->SetStringNodeValue("FrameRate", frameRate))
                    return false;

                double acquisitionFrameRate;
                if (_connected_device->GetFloatNodeValue("AcquisitionFrameRate", acquisitionFrameRate))
                    frame_rates.push_back(static_cast<uint32_t>(acquisitionFrameRate));
                else
                    return false;
            }

            if (!_connected_device->SetStringNodeValue("FrameRate", frameRateValue))
                return false;

            return true;
        }

        bool cs_device::is_profile_format(UINT32 width, UINT32 height, UINT32 format, const stream_profile& profile)
        {
            // Unrectified depth calibration stream width is displayed as double
            if (width == 2560 && height == 800 || (width == 3840 && height == 1080)) {
                width /= 2;
            }
            return ((width == profile.width) &&
                    (height == profile.height) &&
                    (format == native_pixel_format_to_cs_pixel_format(profile.format)));
        }

        uint32_t cs_device::cs_pixel_format_to_native_pixel_format(std::string cs_format)
        {
            uint32_t npf;
            if (cs_format.compare("YUV422") == 0)
                npf = rs_fourcc('Y', 'U', 'Y', 'V');
            else if (cs_format.compare("YUV422Packed") == 0)
                npf = rs_fourcc('U', 'Y', 'V', 'Y');
            else if (cs_format.compare("Mono16") == 0)
                npf = rs_fourcc('Z', '1', '6', ' ');
            else if (cs_format.compare("Mono8") == 0)
                npf = rs_fourcc('G', 'R', 'E', 'Y');
            else 
                throw wrong_api_call_sequence_exception("Unsuported image format!");

            return npf;
        }

        uint32_t cs_device::native_pixel_format_to_cs_pixel_format(uint32_t native_format)
        {
            if (native_format == rs_fourcc('Y', '8', 'I', ' ') || native_format == rs_fourcc('Z', '1', '6', ' ') || native_format == rs_fourcc('B', 'Y', 'R', '2'))
                return GVSP_PIX_MONO16;
            else if (native_format == rs_fourcc('G', 'R', 'E', 'Y'))
                return GVSP_PIX_MONO8;
            else if (native_format == rs_fourcc('Y', 'U', 'Y', 'V'))
                return GVSP_PIX_YUV422_YUYV_PACKED;
            else if (native_format == rs_fourcc('U', 'Y', 'V', 'Y'))
                return GVSP_PIX_YUV422_PACKED;
            else if (native_format = rs_fourcc('Y', '1', '2', 'I'))
                return GVSP_PIX_MONO12_PACKED;
            else
                throw wrong_api_call_sequence_exception("Unable to map Realsense pixel format to CameraSuite pixel format!");
        }

        bool cs_device::is_option_supported(rs2_option opt, cs_stream stream)
        {
            switch (opt)
            {
                case RS2_OPTION_ASIC_TEMPERATURE:
                case RS2_OPTION_PROJECTOR_TEMPERATURE:
                    return is_temperature_supported();
                case RS2_OPTION_RGB_LED_TOGGLE:
                    return is_rgb_led_toggle_supported();
                default: return false;
            }
        }

        bool cs_device::get_pu(rs2_option opt, int32_t& value, cs_stream stream)
        {
            return get_cs_param_value(opt, value, stream);
        }

        bool cs_device::set_pu(rs2_option opt, int32_t value, cs_stream stream)
        {
            return set_cs_param(opt, value, stream);
        }

        control_range cs_device::get_pu_range(rs2_option option, cs_stream stream)
        {
            // Auto controls range is trimmed to {0,1} range
            if((option == RS2_OPTION_ENABLE_AUTO_EXPOSURE) || 
               (option == RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE) ||
               (option == RS2_OPTION_EMITTER_ENABLED) || 
               (option == RS2_OPTION_AUTO_EXPOSURE_PRIORITY) || 
               (option == RS2_OPTION_RGB_LED_TOGGLE) ||
               (option == RS2_OPTION_METADATA_TOGGLE))
            {
                static const int32_t min = 0, max = 1, step = 1, def = 1;
                control_range range(min, max, step, def);

                return range;
            }
            if ((option == RS2_OPTION_OUTPUT_TRIGGER_ENABLED) || 
                (option == RS2_OPTION_SOFTWARE_TRIGGER_ALL_SENSORS) || 
                (option == RS2_OPTION_BACKLIGHT_COMPENSATION))
            {
                static const int32_t min = 0, max = 1, step = 1, def = 0;
                control_range range(min, max, step, def);

                return range;
            }

            //range copied from asic_and_projector_temperature_options::get_range()
            if (option == RS2_OPTION_ASIC_TEMPERATURE || option == RS2_OPTION_PROJECTOR_TEMPERATURE)
                return control_range{ -40, 125, 0, 0 };

            //range for burst count manually decided to 1000000
            if (option == RS2_OPTION_EXTERNAL_EVENT_BURST_COUNT)
                return control_range{ 1, 1000000, 1, 1 };

            int32_t min, max, step, value, def;
            if (!get_cs_param_min(option, min, stream)) min = 0;
            if (!get_cs_param_max(option, max, stream)) max = 0;
            if (!get_cs_param_step(option, step, stream)) step = 1;
            if (!get_cs_param_default(option, def, stream)) def = 0;
           
            control_range range(min, max, step, def);

            return range;
        }

        bool cs_device::set_cs_param(rs2_option option, int32_t value, cs_stream stream)
        {
            switch(option)
            {
                case RS2_OPTION_WHITE_BALANCE:
                case RS2_OPTION_EXPOSURE:
                case RS2_OPTION_POWER_LINE_FREQUENCY:
                case RS2_OPTION_GAMMA:
                case RS2_OPTION_SHARPNESS:
                case RS2_OPTION_SATURATION:
                case RS2_OPTION_BRIGHTNESS:
                case RS2_OPTION_CONTRAST:
                case RS2_OPTION_GAIN:
                case RS2_OPTION_LASER_POWER:
                case RS2_OPTION_HUE:
                    return _connected_device->SetIntegerNodeValue(
                        get_cs_param_name(option, stream), 
                        round_cs_param(option, value, stream)
                    );
                case RS2_OPTION_RGB_LED_TOGGLE:
                // case RS2_OPTION_METADATA_TOGGLE:
                {
                    /* Cache _metadata_toggle_value.
                    *  This is done so that genicam mechanism is unburdened from frequent calls. */

                    bool status = _connected_device->SetBooleanNodeValue(get_cs_param_name(option, stream), (value == 1));

                    if (status && option == RS2_OPTION_METADATA_TOGGLE) {
                        _metadata_toggle_value = (value == 1);
                    }

                    return status;
                }
                
                case RS2_OPTION_METADATA_TOGGLE:
                {
                    _metadata_toggle_value = false;
                    return true;
                }


                case RS2_OPTION_ENABLE_AUTO_EXPOSURE:
                case RS2_OPTION_BACKLIGHT_COMPENSATION:
                case RS2_OPTION_EMITTER_ENABLED:
                case RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE:
                case RS2_OPTION_AUTO_EXPOSURE_PRIORITY:
                {
                    if (value == 1) return _connected_device->SetStringNodeValue(get_cs_param_name(option, stream), "On");
                    else if (value == 0) return _connected_device->SetStringNodeValue(get_cs_param_name(option, stream), "Off");
                }
                case RS2_OPTION_PACKET_SIZE:
                {
                    if (_is_capturing[stream])
                        throw wrong_api_call_sequence_exception("Unable to set Packet Size while streaming!");

                    auto enabled = value == 0;

                    auto node = _connected_device->GetStatisticsNode("DetectOptimalPacketSize");
                    if (node != nullptr)
                        node->SetBooleanNodeValue(enabled);

                    if (enabled)
                        return true;
                }
                case RS2_OPTION_INTER_PACKET_DELAY:
                {
                    for (auto member_stream : get_stream_group(stream)) {
                        if (!select_channel(member_stream))
                            return false;

                        if (!_connected_device->SetIntegerNodeValue(get_cs_param_name(option, member_stream), (int)value))
                            return false;
                    }
                    
                    return true;
                }
                case RS2_OPTION_OUTPUT_TRIGGER_ENABLED:
                {
                    if (!select_source(stream))
                        return false;

                    if (!_connected_device->SetStringNodeValue("LineSelector", "Line1"))
                        return false;

                    if (value == 1) return _connected_device->SetStringNodeValue(get_cs_param_name(option, stream), "VSync");
                    else if (value == 0) return _connected_device->SetStringNodeValue(get_cs_param_name(option, stream), "UserOutput1");
                }                
				case RS2_OPTION_SOFTWARE_TRIGGER:
                {
                    if (!select_source(stream))
                        return false;

                    if (!_connected_device->CommandNodeExecute("TriggerSoftware"))
                        return false;
                        
                    return true;
                }
                case RS2_OPTION_SOFTWARE_TRIGGER_ALL_SENSORS:
                {
                    if (!select_source(stream))
                        return false;

                    if (value == 1) return _connected_device->SetBooleanNodeValue("TriggerSoftwareAllSources", true);
                    else if (value == 0) return _connected_device->SetBooleanNodeValue("TriggerSoftwareAllSources", false);
                }
                case RS2_OPTION_EXT_TRIGGER_SOURCE:
                {
                    if (!select_source(stream))
                        return false;

                    std::string trigger_mode;
                    _connected_device->GetStringNodeValue("TriggerMode", trigger_mode);
                    if (trigger_mode == "Off") 
                        return false;

                    if (value == 1) return _connected_device->SetStringNodeValue("TriggerSource", "Line1");
                    else if (value == 2) return _connected_device->SetStringNodeValue("TriggerSource", "Software");

                }
                case RS2_OPTION_USER_OUTPUT_LEVEL:
                {
                    if (!select_source(stream))
                        return false;

                    if (value == CS_USER_OUTPUT_LEVEL_LOW) return _connected_device->SetBooleanNodeValue("UserOutputValue", false);
                    else if (value == CS_USER_OUTPUT_LEVEL_HIGH) return _connected_device->SetBooleanNodeValue("UserOutputValue", true);
                }
                case RS2_OPTION_LINE_DEBOUNCER_TIME:
                {
                    if (!select_source(stream))
                        return false;
                    
                    if (!_connected_device->SetStringNodeValue("LineSelector", "Line2"))
                        return false;

                    return _connected_device->SetFloatNodeValue(get_cs_param_name(option, stream), (double)value);
                }
                case RS2_OPTION_EXTERNAL_EVENT_BURST_COUNT:
                {
                    if (!select_source(stream))
                        return false;
                    
                    std::string trig_sel, trig_mode, trig_type;
                    _connected_device->GetStringNodeValue("TriggerSelector", trig_sel);
                    _connected_device->GetStringNodeValue("TriggerMode", trig_mode);
                    _connected_device->GetStringNodeValue("TriggerType", trig_type);

                    if (trig_sel != "FrameBurstStart" || trig_mode != "On" || trig_type != "ExternalEvent")
                        return true;
                    
                    return _connected_device->SetIntegerNodeValue(get_cs_param_name(option, stream), (double)value);
                }

                default: throw linux_backend_exception(to_string() << "no CS cid for option " << option);
            }
        }

        int32_t cs_device::round_cs_param(rs2_option option, int32_t value, cs_stream stream)
        {
            int32_t min, max, step;
            if (get_cs_param_min(option, min, stream)
                && get_cs_param_max(option, max, stream)
                && get_cs_param_step(option, step, stream)) 
            {
                auto rounded = step * std::round(value / static_cast<double>(step));
                if (rounded < min) return min;
                else if (rounded > max) return max;
                else return static_cast<int32_t>(rounded);
            }
            else
            {
                return value;
            }
        }

        std::string cs_device::get_cs_param_name(rs2_option option, cs_stream stream)
        {
            switch(option)
            {
                case RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE:
                    if (stream == CS_STREAM_COLOR) return std::string("RGB_WhiteBalanceTempAuto");
                case RS2_OPTION_WHITE_BALANCE:
                    if (stream == CS_STREAM_COLOR) return std::string("RGB_WhiteBalanceTemp");
                case RS2_OPTION_ENABLE_AUTO_EXPOSURE:
                    if (stream == CS_STREAM_COLOR) return std::string("RGB_ExposureAuto");
                    else if (stream == CS_STREAM_DEPTH) return std::string("STR_ExposureAuto");
                case RS2_OPTION_EXPOSURE:
                    if (stream == CS_STREAM_COLOR) return std::string("RGB_Exposure");
                    else if (stream == CS_STREAM_DEPTH) return std::string("STR_Exposure");
                case RS2_OPTION_BACKLIGHT_COMPENSATION:
                    if (stream == CS_STREAM_COLOR) return std::string("RGB_BacklightComp");
                case RS2_OPTION_AUTO_EXPOSURE_PRIORITY:
                    if (stream == CS_STREAM_COLOR) return std::string("RGB_LowLightComp");
                case RS2_OPTION_POWER_LINE_FREQUENCY:
                    if (stream == CS_STREAM_COLOR) return std::string("RGB_PowerLineFrequency");
                case RS2_OPTION_GAMMA:
                    if (stream == CS_STREAM_COLOR) return std::string("RGB_Gamma");
                case RS2_OPTION_SHARPNESS:
                    if (stream == CS_STREAM_COLOR) return std::string("RGB_Sharpness");
                case RS2_OPTION_SATURATION:
                    if (stream == CS_STREAM_COLOR) return std::string("RGB_Saturation");
                case RS2_OPTION_BRIGHTNESS:
                    if (stream == CS_STREAM_COLOR) return std::string("RGB_Brightness");
                case RS2_OPTION_CONTRAST:
                    if (stream == CS_STREAM_COLOR) return std::string("RGB_Contrast");
                case RS2_OPTION_GAIN:
                    if (stream == CS_STREAM_COLOR) return std::string("RGB_Gain");
                    if (stream == CS_STREAM_DEPTH) return std::string("STR_Gain");
                case RS2_OPTION_HUE:
                    if (stream == CS_STREAM_COLOR) return std::string("RGB_Hue");
                case RS2_OPTION_EMITTER_ENABLED:
                    if (stream == CS_STREAM_DEPTH) return std::string("STR_LaserEnable");
                case RS2_OPTION_LASER_POWER:
                    if (stream == CS_STREAM_DEPTH) return std::string("STR_LaserPower");
                case RS2_OPTION_INTER_PACKET_DELAY: return std::string("GevSCPD");
                case RS2_OPTION_PACKET_SIZE: return std::string("GevSCPSPacketSize");
                case RS2_OPTION_ASIC_TEMPERATURE: return std::string("IntelASIC");
                case RS2_OPTION_PROJECTOR_TEMPERATURE: return std::string("DepthModule");
                case RS2_OPTION_OUTPUT_TRIGGER_ENABLED: return std::string("LineSource");
                case RS2_OPTION_LINE_DEBOUNCER_TIME: return std::string("LineDebouncerTime");
                case RS2_OPTION_EXTERNAL_EVENT_BURST_COUNT: return std::string("AcquisitionBurstFrameCount");
                case RS2_OPTION_RGB_LED_TOGGLE:
                    if (stream == CS_STREAM_COLOR) return std::string("RGB_LedToggle");
                case RS2_OPTION_METADATA_TOGGLE:
                    if (stream == CS_STREAM_DEPTH) return std::string("MetadataToggle");
                default: throw linux_backend_exception(to_string() << "no CS cid for option " << option);
            }
        }

        bool cs_device::get_cs_param_min(rs2_option option, int32_t &value, cs_stream stream)
        {
            smcs::StringList node_value_list;
            INT64 int_value;
            double float_value;
            bool status;

            switch(option)
            {
                case RS2_OPTION_WHITE_BALANCE:
                case RS2_OPTION_EXPOSURE:
                case RS2_OPTION_GAMMA:
                case RS2_OPTION_SHARPNESS:
                case RS2_OPTION_SATURATION:
                case RS2_OPTION_BRIGHTNESS:
                case RS2_OPTION_CONTRAST:
                case RS2_OPTION_LASER_POWER:
                case RS2_OPTION_GAIN:
                case RS2_OPTION_HUE:
                case RS2_OPTION_EXTERNAL_EVENT_BURST_COUNT:
                {
                    status = _connected_device->GetIntegerNodeMin(get_cs_param_name(option, stream), int_value);
                    value = static_cast<int32_t>(int_value);
                    return status;
                }
                case RS2_OPTION_POWER_LINE_FREQUENCY:
                    status = _connected_device->GetEnumNodeValuesList(get_cs_param_name(option, stream), node_value_list);
                    value = 0;
                    return status;
                case RS2_OPTION_INTER_PACKET_DELAY:
                case RS2_OPTION_PACKET_SIZE:
                {
                    if (!select_channel(stream))
                        return false;

                    status = _connected_device->GetIntegerNodeMin(get_cs_param_name(option, stream), int_value);
                    value = static_cast<int32_t>(int_value);
                    return status;
                }
                case RS2_OPTION_OUTPUT_TRIGGER_ENABLED:
                    status = _connected_device->GetEnumNodeValuesList(get_cs_param_name(option, stream), node_value_list);
                    value = 0;
                    return status;
                case RS2_OPTION_LINE_DEBOUNCER_TIME: 
                {
                    status = _connected_device->GetFloatNodeMin(get_cs_param_name(option, stream), float_value);
                    value = static_cast<int32_t>(float_value);
                    return status;
                }
                default: throw linux_backend_exception(to_string() << "no CS cid for option " << option);
            }
        }

        bool cs_device::get_cs_param_max(rs2_option option, int32_t &value, cs_stream stream)
        {
            smcs::StringList node_value_list;
            INT64 int_value;
            double float_value;
            bool status;

            switch(option)
            {
                case RS2_OPTION_WHITE_BALANCE:
                case RS2_OPTION_EXPOSURE:
                case RS2_OPTION_GAMMA:
                case RS2_OPTION_SHARPNESS:
                case RS2_OPTION_SATURATION:
                case RS2_OPTION_BRIGHTNESS:
                case RS2_OPTION_LASER_POWER:
                case RS2_OPTION_CONTRAST:
                case RS2_OPTION_GAIN:
                case RS2_OPTION_HUE:
                {
                    status = _connected_device->GetIntegerNodeMax(get_cs_param_name(option, stream), int_value);
                    value = static_cast<int32_t>(int_value);
                    return status;
                }
                case RS2_OPTION_POWER_LINE_FREQUENCY:
                    status = _connected_device->GetEnumNodeValuesList(get_cs_param_name(option, stream), node_value_list);
                    value = static_cast<int32_t>(node_value_list.size()-1);
                    return status;
                case RS2_OPTION_INTER_PACKET_DELAY:
                case RS2_OPTION_PACKET_SIZE:
                {
                    if (!select_channel(stream))
                        return false;

                    status = _connected_device->GetIntegerNodeMax(get_cs_param_name(option, stream), int_value);
                    value = static_cast<int32_t>(int_value);
                    return status;
                }
                case RS2_OPTION_OUTPUT_TRIGGER_ENABLED:
                    status = _connected_device->GetEnumNodeValuesList(get_cs_param_name(option, stream), node_value_list);
                    value = 1;
                    return status;
                case RS2_OPTION_LINE_DEBOUNCER_TIME:
                {
                    status = _connected_device->GetFloatNodeMax(get_cs_param_name(option, stream), float_value);
                    value = static_cast<int32_t>(float_value);
                    return status;
                }
                case RS2_OPTION_EXTERNAL_EVENT_BURST_COUNT:
                {
                    status = true;
                    value = static_cast<int32_t>(CS_EXT_EVENT_BURST_MAX);
                    return status;
                }
                default: throw linux_backend_exception(to_string() << "no CS cid for option " << option);
            }
        }

        bool cs_device::get_cs_param_step(rs2_option option, int32_t& step, cs_stream stream)
        {
            INT64 int_value;
            double float_value; 

            switch(option)
            {
                case RS2_OPTION_WHITE_BALANCE:
                case RS2_OPTION_EXPOSURE:
                case RS2_OPTION_GAMMA:
                case RS2_OPTION_SHARPNESS:
                case RS2_OPTION_SATURATION:
                case RS2_OPTION_LASER_POWER:
                case RS2_OPTION_BRIGHTNESS:
                case RS2_OPTION_CONTRAST:
                case RS2_OPTION_GAIN:
                case RS2_OPTION_HUE:
                case RS2_OPTION_EXTERNAL_EVENT_BURST_COUNT:
                {
                    auto result = _connected_device->GetIntegerNodeIncrement(get_cs_param_name(option, stream), int_value);
                    step = static_cast<int32_t>(int_value);
                    return result;
                }
                case RS2_OPTION_POWER_LINE_FREQUENCY:
                {
                    step = 1;
                    return true;
                }
                case RS2_OPTION_INTER_PACKET_DELAY:
                case RS2_OPTION_PACKET_SIZE:
                {
                    if (!select_channel(stream)) {
                        step = 1;
                        return true;
                    }

                    auto result = _connected_device->GetIntegerNodeIncrement(get_cs_param_name(option, stream), int_value);
                    step = static_cast<int32_t>(int_value);
                    return true;
                }
                case RS2_OPTION_OUTPUT_TRIGGER_ENABLED:
                {
                    step = 1;
                    return true;
                }
                case RS2_OPTION_LINE_DEBOUNCER_TIME:
                {
                    auto result = _connected_device->GetFloatNodeIncrement(get_cs_param_name(option, stream), float_value);
                    step = static_cast<int32_t>(float_value);
                    return result;
                }
                default: throw linux_backend_exception(to_string() << "no CS cid for option " << option);
            }
        }

        bool cs_device::get_cs_param_default(rs2_option option, int32_t& def, cs_stream stream)
        {
            std::string string_value;
            INT64 int_value;
            bool status = true;

            //get device name, used to look up the correct map
            std::string camera_name = _connected_device->GetModelName();
                status = false;
                if (camera_name == "D415e")
                {
                    if (stream == CS_STREAM_COLOR)
                    {
                        auto search = d415e_rgb_option_default.find(option);
                        if (search != d415e_rgb_option_default.end())
                        {
                            def = static_cast<int32_t>(search->second);
                            status = true;
                        }
                    }
                    else {
                        auto search = d415e_stereo_option_default.find(option);
                        if (search != d415e_stereo_option_default.end())
                        {
                            def = static_cast<int32_t>(search->second);
                            status = true;
                        }
                    }
                }
                else if (camera_name == "D435e")
                {
                    if (stream == CS_STREAM_COLOR)
                    {
                        auto search = d435e_rgb_option_default.find(option);
                        if (search != d435e_rgb_option_default.end())
                        {
                            def = static_cast<int32_t>(search->second);
                            status = true;
                        }
                    }
                    else {
                        auto search = d435e_stereo_option_default.find(option);
                        if (search != d435e_stereo_option_default.end())
                        {
                            def = static_cast<int32_t>(search->second);
                            status = true;
                        }
                    }
                }
                else if (camera_name == "D455e")
                {
                    if (stream == CS_STREAM_COLOR)
                    {
                        auto search = d455e_rgb_option_default.find(option);
                        if (search != d455e_rgb_option_default.end())
                        {
                            def = static_cast<int32_t>(search->second);
                            status = true;
                        }
                    }
                    else {
                        auto search = d455e_stereo_option_default.find(option);
                        if (search != d455e_stereo_option_default.end())
                        {
                            def = static_cast<int32_t>(search->second);
                            status = true;
                        }
                    }
                }
            return status;
        }

        bool cs_device::get_cs_param_value(rs2_option option, int32_t &value, cs_stream stream)
        {
            std::string string_value;
            INT64 int_value;
            bool status;

            switch(option)
            {
                case RS2_OPTION_WHITE_BALANCE:
                case RS2_OPTION_EXPOSURE:
                case RS2_OPTION_POWER_LINE_FREQUENCY:
                case RS2_OPTION_GAMMA:
                case RS2_OPTION_SHARPNESS:
                case RS2_OPTION_SATURATION:
                case RS2_OPTION_BRIGHTNESS:
                case RS2_OPTION_CONTRAST:
                case RS2_OPTION_LASER_POWER:
                case RS2_OPTION_GAIN:
                case RS2_OPTION_HUE:
                {
                    status = _connected_device->GetIntegerNodeValue(get_cs_param_name(option, stream), int_value);
                    value = static_cast<int32_t>(int_value);
                    return status;
                }
                case RS2_OPTION_RGB_LED_TOGGLE:
                case RS2_OPTION_METADATA_TOGGLE:
                {
                    status = _connected_device->GetStringNodeValue(get_cs_param_name(option, stream), string_value);
                    if (string_value == "0") value = 0;
                    else value = 1;
                    return status;
                }
                case RS2_OPTION_ENABLE_AUTO_EXPOSURE:
                case RS2_OPTION_BACKLIGHT_COMPENSATION:
                case RS2_OPTION_EMITTER_ENABLED:
                case RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE:
                case RS2_OPTION_AUTO_EXPOSURE_PRIORITY:
                {
                    status = _connected_device->GetStringNodeValue(get_cs_param_name(option, stream), string_value);
                    if (string_value == "Off") value = 0;
                    else value = 1;
                    return status;
                }
                case RS2_OPTION_INTER_PACKET_DELAY:
                case RS2_OPTION_PACKET_SIZE:
                {
                    if (!select_channel(stream))
                        return false;

                    status = _connected_device->GetIntegerNodeValue(get_cs_param_name(option, stream), int_value);
                    value = static_cast<int32_t>(int_value);
                    return status;
                }
                case RS2_OPTION_ASIC_TEMPERATURE:
                case RS2_OPTION_PROJECTOR_TEMPERATURE:
                {
                    if (_connected_device->SetStringNodeValue("DeviceTemperatureSelector", get_cs_param_name(option, stream))) {
                        double temperature;
                        if (_connected_device->GetFloatNodeValue("DeviceTemperature", temperature)) {
                            value = static_cast<int32_t> (temperature);
                            return true;
                        }
                        else {
                            // Fix for potential hang (firmware side) of setting temperature selector
                            std::string s;
                            _connected_device->GetStringNodeValue("DeviceTemperatureSelector", s);
                        }
                    }
                    return false;
                }
                case RS2_OPTION_OUTPUT_TRIGGER_ENABLED:
                {
                    if (!select_source(stream))
                        return false;
                        
                    if (!_connected_device->SetStringNodeValue("LineSelector", "Line1"))
                        return false;

                    status = _connected_device->GetStringNodeValue(get_cs_param_name(option, stream), string_value);
                    if (string_value == "VSync") value = 1;
                    else value = 0;
                    return status;
                }
                case RS2_OPTION_SOFTWARE_TRIGGER:
                {
                    if (!select_source(stream))
                        return false;
                    value = 1;
                    return true;
                }
                case RS2_OPTION_SOFTWARE_TRIGGER_ALL_SENSORS:
                {
                    if (!select_source(stream))
                        return false;

                    bool trigger_all_sensors;
                    status = _connected_device->GetBooleanNodeValue("TriggerSoftwareAllSources", trigger_all_sensors);
                    if (trigger_all_sensors == false) value = 0;
                    else value = 1;
                    return status;
                }
                case RS2_OPTION_EXT_TRIGGER_SOURCE:
                {
                    if (!select_source(stream))
                        return false;

                    std::string trigger_mode, trigger_source;
                    status = _connected_device->GetStringNodeValue("TriggerMode", trigger_mode);
                    if (trigger_mode == "Off")
                    {
                        value = 1;
                        return status;
                    }
                    else
                    {
                        status = _connected_device->GetStringNodeValue("TriggerSource", trigger_source);
                        if (trigger_source == "Line1")
                            value = 1;
                        else if (trigger_source == "Software")
                            value = 2;
                        else
                            value = 1;
                        return status;
                    }
                }
                case RS2_OPTION_USER_OUTPUT_LEVEL:
                {
                    if (!select_source(stream))
                        return false;

                    bool user_output_level;

                    status = _connected_device->GetBooleanNodeValue("UserOutputValue", user_output_level);
                    if (user_output_level == false) value = CS_USER_OUTPUT_LEVEL_LOW;
                    else value = CS_USER_OUTPUT_LEVEL_HIGH;

                    return status;
                }
                case RS2_OPTION_LINE_DEBOUNCER_TIME:
                {
                    if (!select_source(stream))
                        return false;
                    
                    if (!_connected_device->SetStringNodeValue("LineSelector", "Line2"))
                        return false;

                    double line_debouncer_time;
                    status = _connected_device->GetFloatNodeValue(get_cs_param_name(option, stream), line_debouncer_time);
                    value = static_cast<int32_t>(line_debouncer_time);
                    return status;
                }
                case RS2_OPTION_EXTERNAL_EVENT_BURST_COUNT:
                {
                    if (!select_source(stream))
                        return false;

                    std::string trig_sel, trig_mode, trig_type;
                    _connected_device->GetStringNodeValue("TriggerSelector", trig_sel);
                    _connected_device->GetStringNodeValue("TriggerMode", trig_mode);
                    _connected_device->GetStringNodeValue("TriggerType", trig_type);
                    
                    if (trig_sel != "FrameBurstStart" || trig_mode != "On" || trig_type != "ExternalEvent")
                    {
                        value = 1;
                        return true;
                    }

                    INT64 burst_count;
                    status = _connected_device->GetIntegerNodeValue(get_cs_param_name(option, stream), burst_count);
                    value = static_cast<int32_t>(burst_count);
                    return status;
                }

                default: throw linux_backend_exception(to_string() << "no CS cid for option " << option);
            }
        }

        void cs_device::start_acquisition(cs_stream stream)
        {   
            // set continuous acquisition mode
            _connected_device->SetStringNodeValue("AcquisitionMode", "Continuous");

            if (!_connected_device->CommandNodeExecute("AcquisitionStart"))
                throw wrong_api_call_sequence_exception("Unable to start acquisition!");
        }

        void cs_device::stop_acquisition(cs_stream stream)
        {
            if (!select_source(stream))
                throw wrong_api_call_sequence_exception("Unable to select source!");

            if (!_connected_device->CommandNodeExecute("AcquisitionStop"))
                LOG_WARNING("Unable to stop acquisition!");

            if (!set_region(stream, false))
                throw wrong_api_call_sequence_exception("Unable to set_region!");
        }

        bool cs_device::select_source(cs_stream stream)
        {
            auto source = get_stream_source(stream);
            if (_selected_source_initialized && (source == _selected_source))
                return true;
            _selected_source = source;
            _selected_source_initialized = true;
            return _connected_device->SetIntegerNodeValue("SourceControlSelector", source);
        }

        bool cs_device::set_source_locked(cs_stream stream, bool locked)
        {
            if (select_source(stream))
                return _connected_device->SetIntegerNodeValue("TLParamsLocked", locked ? 1 : 0);

            return false;
        }

        bool cs_device::set_region(cs_stream stream, bool enable)
        {
            if (select_region(stream))
                return _connected_device->SetStringNodeValue("RegionMode", enable ? "On" : "Off");

            return false;
        }

        bool cs_device::disable_source_regions(cs_stream stream)
        {
            if (!select_source(stream))
                return false;

            smcs::StringList regions;
            if (!_connected_device->GetEnumNodeValuesList("RegionSelector", regions))
                return false;

            for (const auto& region : regions) {
                if (!_connected_device->SetStringNodeValue("RegionSelector", region))
                    return false;
                if (!_connected_device->SetStringNodeValue("RegionMode", "Off"))
                    return false;
            }

            return true;
        }

        bool cs_device::select_region(cs_stream stream)
        {
            if (select_source(stream))
                return _connected_device->SetIntegerNodeValue("RegionSelector", get_stream_region(stream));

            return false;
        }

        bool cs_device::select_channel(cs_stream stream)
        {
            UINT32 channel;
            if (get_stream_channel(stream, channel))
                return _connected_device->SetIntegerNodeValue("GevStreamChannelSelector", static_cast<INT64>(channel));

            return false;
        }

        INT64 cs_device::get_stream_source(cs_stream stream)
        {
            switch (stream) {
            case CS_STREAM_DEPTH:
            case CS_STREAM_IR_LEFT:
            case CS_STREAM_IR_RIGHT:
            case CS_STREAM_IR_LEFT_COLOR:
                return 0;
            case CS_STREAM_COLOR:
                return 1;
            // TODO - nh
            /*case CS_STREAM_IMU:
                return 0;*/
            default:
                throw wrong_api_call_sequence_exception("Unknown stream ID!");
            }
        }

        INT64 cs_device::get_stream_region(cs_stream stream)
        {
            switch (stream) {
            case CS_STREAM_DEPTH:
                return 0;
            case CS_STREAM_COLOR:
                // In Color stream, single Region can be active at the same time, so this should be ok
                if (_is_raw16) {
                    return 2;
                }
                else {
                    return 0;
                }
            case CS_STREAM_IR_LEFT:
                if (_is_calibration_stream) {
                    return 1;
                }
                else {
                    return 3;
                }
            case CS_STREAM_IR_RIGHT:
                if (_is_calibration_stream) {
                    return 1;
                }
                else {
                    return 2;
                }
            case CS_STREAM_IR_LEFT_COLOR:
                return 4;
            // TODO - nh
            /*case CS_STREAM_IMU:
                return 0;*/
            default:
                throw wrong_api_call_sequence_exception("Unknown stream ID!");
            }
        }

        bool cs_device::get_stream_channel(cs_stream stream, UINT32& channel)
        {
            if (_stream_channels.find(stream) == _stream_channels.end() || (stream == CS_STREAM_IR_LEFT  || stream == CS_STREAM_IR_RIGHT)) {
                
                if (!select_source(stream))
                    return false;

                if (!select_region(stream))
                    return false;

                std::string region_destination;
                if (!_connected_device->GetStringNodeValue("RegionDestination", region_destination))
                    return false;

                if (!_connected_device->SetStringNodeValue("TransferSelector", region_destination))
                    return false;

                INT64 transfer_stream_channel;
                if (!_connected_device->GetIntegerNodeValue("TransferStreamChannel", transfer_stream_channel))
                    return false;

                _stream_channels[stream] = static_cast<UINT32>(transfer_stream_channel);
            }

            channel = _stream_channels[stream];
            return true;
        }

        std::vector<cs_stream> cs_device::get_stream_group(cs_stream stream)
        {
            std::vector<cs_stream> group { stream };

            switch (stream) {
            case CS_STREAM_DEPTH:
                group.push_back(CS_STREAM_IR_LEFT);
                group.push_back(CS_STREAM_IR_RIGHT);
                break;
            default:
                break;
            }

            return group;
        }

        void cs_device::stream_on(std::function<void(const notification &n)> error_handler, std::vector<cs_stream> streams, std::vector<platform::stream_profile> profiles)
        {
            if (streams.empty()) return;

            for (auto i = 0; i < streams.size(); ++i)
                set_format(profiles[i], streams[i]);

            if (!select_source(streams[0]))
                throw wrong_api_call_sequence_exception("Unable to select source!");

            // Temporary fix for D435e not setting the format correctly 
            // when programmed in a specific order.
            // Issue is reproducible when depth and left IR are selected 
            // in realsense-viewer.
            // Issue exists only in firmware 1.5.1.0 and earlier. 
            // It is fixed in later firmwares versions.
            auto left_ir = std::find(streams.begin(), streams.end(), CS_STREAM_IR_LEFT);
            if (left_ir != streams.end() && !this->is_calibration_stream_toggled())
                _connected_device->SetStringNodeValue("RegionSelector", "Region3");

            stream_params_lock(streams[0]);
            start_acquisition(streams[0]);

            for (auto i = 0; i < streams.size(); ++i)
                init_stream(error_handler, streams[i]);
        }

        void cs_device::init_stream(std::function<void(const notification& n)> error_handler, cs_stream stream)
        {
            if (stream < _number_of_streams)
            {
                if (!_is_capturing[stream])
                {
                    UINT32 channel;
                    if (!get_stream_channel(stream, channel))
                        throw wrong_api_call_sequence_exception("Unable to get stream channel!");

                    _error_handler[stream] = error_handler;
                    _is_capturing[stream] = true;
                    _threads[stream] = std::unique_ptr<std::thread>(new std::thread([this, stream, channel]() { capture_loop(stream, channel); }));
                }
            }
            else throw wrong_api_call_sequence_exception("Unsuported streaming type!");
        }

        void cs_device::close(std::vector<cs_stream> streams)
        {
            if (streams.empty()) return;

            for (auto stream : streams)
                deinit_stream(stream);

            for (auto stream : streams)
            {
               stream_params_unlock(stream);
               stop_acquisition(stream);
               set_region(stream, false);
            }
        }

        void cs_device::deinit_stream(cs_stream stream)
        {
            if (stream < _number_of_streams)
            {
                if (_is_capturing[stream])
                {
                    _is_capturing[stream] = false;
                    _threads[stream]->join();
                    _threads[stream].reset();
                }
                if (_callbacks[stream]) _callbacks[stream] = nullptr;
            }
            else throw wrong_api_call_sequence_exception("Unsuported streaming type!");
        }

        power_state cs_device::set_power_state(power_state state)
        {
            std::lock_guard<std::mutex> lock(_power_lock);

            std::string serial;

            if (state == D0 && _power_state == D3)
            {
                _power_state = D0;
            }
            if (state == D3 && _power_state == D0)
            {
                _power_state = D3;
            }

            return _power_state;
        }

        bool cs_device::reset(void)
        {
            set_device_init_flag_sn(_device_info.serial, false);
            return _connected_device->CommandNodeExecute("DeviceReset");
        }

        std::vector<byte> cs_device::send_hwm(std::vector<byte>& buffer)
        {
            std::lock_guard<std::mutex> lock(_hwm_lock);

            const uint8_t unknown_opcode = 0x00;
            const uint8_t opcode_index = cs::d4_hwm_cmd_opcode_offset;
            const uint8_t param2_index = cs::d4_hwm_cmd_param2_offset;

            // check enum fw_cmd cs-private.h
            const uint8_t setrgbaeroi_opcode = 0x75;
            const uint8_t getrgbaeroi_opcode = 0x76;

            // 0x50 and 0x0208 - values discovered in D435i imu calibration process
            const uint8_t imucalibdata_opcode = cs::d4_hwm_cmd_opcode_imu_calib;
            const uint16_t imucalibdata_param2 = 0x0208;

            const uint8_t opcode = buffer.size() > opcode_index ? buffer[opcode_index] : unknown_opcode;

            // SETRGBAEROI
            if ((opcode == setrgbaeroi_opcode) && (buffer.size() >= _rgb_ae_roi_hwm_size)) {
                uint32_t param_index = 8;
                uint32_t param1 = read_from_buffer(buffer, param_index);
                uint32_t param2 = read_from_buffer(buffer, param_index + 4);
                uint32_t param3 = read_from_buffer(buffer, param_index + 8);
                uint32_t param4 = read_from_buffer(buffer, param_index + 12);
                set_rgb_ae_roi(param1, param2, param3, param4);
                return std::vector<byte> {setrgbaeroi_opcode, 0, 0, 0};
            }
            // GETRGBAEROI
            else if (opcode == getrgbaeroi_opcode) {
                INT64 top, left, bott, right;
                top = left = bott = right = 0;
                std::vector<byte> vec{ getrgbaeroi_opcode, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
            
                _connected_device->GetIntegerNodeValue("RGB_ExposureAutoROITop", top);
                _connected_device->GetIntegerNodeValue("RGB_ExposureAutoROILeft", left);
                _connected_device->GetIntegerNodeValue("RGB_ExposureAutoROIBottom", bott);
                _connected_device->GetIntegerNodeValue("RGB_ExposureAutoROIRight", right);
            
                memcpy(&vec[4], &top, 2);
                memcpy(&vec[6], &bott, 2);
                memcpy(&vec[8], &left, 2);
                memcpy(&vec[10], &right, 2);
            
                return vec;
            }
            // IMU CALIBRATION DATA
            else if ((opcode == imucalibdata_opcode)) {

                bool is_opcode_ok = true;
                if (buffer.size() > param2_index + 1) {
                    uint16_t param2 = *((uint16_t*)(&buffer[param2_index]));
                    if (param2 == imucalibdata_param2) {
                        if (!send_calib_data(buffer)) {
                            is_opcode_ok = false;
                        }
                    }
                    else {
                        is_opcode_ok = false;
                    }
                }
                else {
                    is_opcode_ok = false;
                }

                if (is_opcode_ok) {
                    return buffer;
                }
                else {
                    std::vector<byte> empty_buffer;
                    return empty_buffer;
                }
            }
            // EVERYTHING ELSE
            else {
                return send_hwm_to_device(buffer);
            }
        }

        uint32_t cs_device::read_from_buffer(std::vector<byte>& buffer, uint32_t index)
        {
            uint32_t result = 0;

            for (int i = 0; i < sizeof(uint32_t); ++i) {
                result += static_cast<uint32_t>(buffer[index + i]) << (i * 8);
            }

            return result;
        }

        std::vector<byte> cs_device::send_hwm_to_device(std::vector<byte>& buffer)
        {
            std::vector<byte> out_vec;
        
            if (send_hwm_cmd(buffer)) {
                if (!receive_hwm_cmd(out_vec)) {
                    // TODO - nh - log warning
                }
            }
        
            return out_vec;
        }

        bool cs_device::send_hwm_cmd(std::vector<byte>& hwm_cmd, bool verify)
        {
            // SetMemory function is limited to 536 bytes per transfer, therefore "buffer.size()" need to be divided into 536 byte chunks + leftover bytes
            // (TODO - nh - buffer.size() must be divisible by 4. Append some dummy bytes in case if is not and update D400e FW to handle dummy bytes correctly)S            
            uint64_t address, regLength;
            uint32_t restSize = hwm_cmd.size();
            uint8_t* sendBuffer = (UINT8*)(&hwm_cmd[0]);
            uint8_t readBuffer[GVCP_WRITEMEM_MAX_COUNT];
            uint8_t resendCount = 3;
            double maxWaitTime = 5.2;
            uint16_t size = 0;
            GEV_STATUS gevStatus;
            bool status = true;

            //std::call_once(_hwm_tx_once_flag, [&, this]()
            //{
            //    // Verify HWm command TX buffer size
            //    status = _connected_device->GetNode("STR_HWmTxBuffer")->GetRegisterNodeLength(_hwm_tx_reg_len);
            //    if (status) {
            //        // Get HWm command TX buffer address
            //        status = _connected_device->GetNode("STR_HWmTxBuffer")->GetRegisterNodeAddress(_hwm_tx_reg_addr);
            //    }
            //});
            //
            //if (!status) {
            //    return status;
            //}

            if (!_hwm_tx_once_flag) {
                // Verify HWm command TX buffer size
                UINT64 reg_len, reg_addr;
                status = _connected_device->GetNode("STR_HWmTxBuffer")->GetRegisterNodeLength(reg_len);
                if (status) {
                    // Get HWm command TX buffer address
                    status = _connected_device->GetNode("STR_HWmTxBuffer")->GetRegisterNodeAddress(reg_addr);
                    if (status) {
                        _hwm_tx_reg_len = reg_len;
                        _hwm_tx_reg_addr = reg_addr;
                        _hwm_tx_once_flag = true;
                    }
                }
            }

            if ((_hwm_tx_reg_len == 0) || (restSize > _hwm_tx_reg_len) || (_hwm_tx_reg_addr == 0)) {
                status = false;
                return status;
            }

            address = _hwm_tx_reg_addr;

            // Append dummy bytes, GiGE Visione - buffer.size() must be divisible by 4
            std::vector<byte> hwm_cmd_tmp;
            if (_connected_device->IsAvailable("STR_HWmTxBufferSendSize")) {

                // Update HWM Tx Buffer with a real command size
                INT64 val = (INT64)restSize;
                bool ss = _connected_device->SetIntegerNodeValue("STR_HWmTxBufferSendSize", val);

                if ((restSize % 4) != 0) {

                    while ((restSize % 4) != 0) {
                        restSize++;
                    }
                    hwm_cmd_tmp = hwm_cmd;
                    hwm_cmd_tmp.resize(restSize, 0); // TODO consider "reserve"
                    sendBuffer = (UINT8*)(&hwm_cmd_tmp[0]);
                }
            }

            // Send HWm command
            while (restSize > 0) {

                size = (restSize > GVCP_WRITEMEM_MAX_COUNT) ? GVCP_WRITEMEM_MAX_COUNT : restSize;

                int tryCount = resendCount;
                bool ok;

                do {
                    ok = true;
                    tryCount--;
                    ok = ok && _connected_device->SetMemory(address, size, sendBuffer, &gevStatus, maxWaitTime);
                    if (verify) {
                        ok = ok && _connected_device->GetMemory(address, size, (UINT8*)(&readBuffer[0]), &gevStatus, maxWaitTime);
                        if ((ok) && (memcmp(sendBuffer, readBuffer, size) != 0)) {
                            ok = false;
                        }
                    }
                } while ((tryCount > 0) && (!ok));

                if (!ok) {
                    status = false;
                    break;
                }

                restSize -= size;
                address += size;
                sendBuffer += size;
            }

            // For the HWM commands, waiting period for camera to respond must be extended, because hwm commmands execution can last
            // more than second. Using CS API calls to increase timeout. Timeouts must be restored after the hwm command execution.
            double cwt; // command wait time
            double mwt; // max commmand wait time

            // get current timeouts
            _smcs_api->GetCommandWaitTime(&mwt, &cwt);
            status &= _smcs_api->SetCommandWaitTime(CS_HWM_MAX_WAIT_TIME, CS_HWM_COMMAND_WAIT_TIME);

            // Execute HWm TX command
            status &= _connected_device->CommandNodeExecute("STR_HWmTxBufferSend");

            // restore original timeouts
            _smcs_api->SetCommandWaitTime(mwt, cwt);

            return status;
        }

        bool cs_device::receive_hwm_cmd(std::vector<byte>& buffer_out)
        {
            uint64_t address, regLength;
            uint32_t restSize;
            uint8_t readBuffer[GVCP_WRITEMEM_MAX_COUNT];
            uint8_t resendCount = 3;
            double maxWaitTime = 0.2;
            uint16_t size = 0;
            GEV_STATUS gevStatus;
            bool status = true;

            //std::call_once(_hwm_rx_once_flag, [&, this]()
            //{
            //    // Verify HWm command TX buffer size
            //    status = _connected_device->GetNode("STR_HWmRxBuffer")->GetRegisterNodeLength(_hwm_rx_reg_len);
            //    if (status) {
            //
            //        if (_hwm_rx_reg_len > IVCAM_MONITOR_MAX_BUFFER_SIZE) {
            //            _hwm_rx_reg_len = IVCAM_MONITOR_MAX_BUFFER_SIZE;
            //            // TODO - nh - handle this
            //        }
            //
            //        // Get HWm command TX buffer address
            //        status = _connected_device->GetNode("STR_HWmRxBuffer")->GetRegisterNodeAddress(_hwm_rx_reg_addr);
            //    }
            //});
            //
            //if (!status) {
            //    return status;
            //}

            if (!_hwm_rx_once_flag) {
                // Verify HWm command RX buffer size
                UINT64 reg_len, reg_addr;
                status = _connected_device->GetNode("STR_HWmRxBuffer")->GetRegisterNodeLength(reg_len);
                if (status) {
                
                    if (reg_len > IVCAM_MONITOR_MAX_BUFFER_SIZE) {
                        reg_len = IVCAM_MONITOR_MAX_BUFFER_SIZE;
                        // TODO - nh - handle this
                    }
                
                    // Get HWm command RX buffer address
                    status = _connected_device->GetNode("STR_HWmRxBuffer")->GetRegisterNodeAddress(reg_addr);
                    if (status) {
                        _hwm_rx_reg_len = reg_len;
                        _hwm_rx_reg_addr = reg_addr;
                        _hwm_rx_once_flag = true;
                    }
                }
            }

            if ((_hwm_rx_reg_len == 0) || (_hwm_rx_reg_addr == 0)) {
                status = false;
                return status;
            }

            address = _hwm_rx_reg_addr;
            restSize = _hwm_rx_reg_len;
            buffer_out.reserve(_hwm_rx_reg_len);

            // For the HWM commands, waiting period for camera to respond must be extended, because hwm commmands execution can last
            // more than second. Using CS API calls to increase timeout. Timeouts must be restored after the hwm command execution.
            double cwt; // command wait time
            double mwt; // max commmand wait time

            // get current timeouts
            _smcs_api->GetCommandWaitTime(&mwt, &cwt);
            status &= _smcs_api->SetCommandWaitTime(CS_HWM_MAX_WAIT_TIME, CS_HWM_COMMAND_WAIT_TIME);

            // Execute HWm RX command
            status &= _connected_device->CommandNodeExecute("STR_HWmRxBufferReceive");

            // restore original timeouts
            _smcs_api->SetCommandWaitTime(mwt, cwt);

            if (status) {

                // Receive HWm command
                while (restSize > 0) {

                    size = (restSize > GVCP_WRITEMEM_MAX_COUNT) ? GVCP_WRITEMEM_MAX_COUNT : restSize;

                    int tryCount = resendCount;
                    bool ok;

                    do {
                        ok = true;
                        tryCount--;
                        ok = ok && _connected_device->GetMemory(address, size, (UINT8*)(&readBuffer[0]), &gevStatus, maxWaitTime);
                        if (ok) {
                            // TODO use insert or std::copy instead
                            for (uint16_t i = 0; i < size; i++) {
                                buffer_out.push_back(readBuffer[i]);
                            }
                        }
                    } while ((tryCount > 0) && (!ok));

                    if (!ok) {
                        status = false;
                        break;
                    }

                    restSize -= size;
                    address += size;
                }
            }

            // Trim receive buffer size according to the "STR_HWmRxBufferReceiveSize" register
            if (_connected_device->IsAvailable("STR_HWmRxBufferReceiveSize")) {
                INT64 buffSize = 0;
                if (_connected_device->GetIntegerNodeValue("STR_HWmRxBufferReceiveSize", buffSize)) {
                    buffer_out.resize(buffSize);
                }
            }

            return status;
        }

        void cs_device::set_rgb_ae_roi(uint32_t top, uint32_t bottom, uint32_t left, uint32_t right)
        {
            _connected_device->SetIntegerNodeValue("RGB_ExposureAutoROITop", top);
            _connected_device->SetIntegerNodeValue("RGB_ExposureAutoROILeft", left);
            _connected_device->SetIntegerNodeValue("RGB_ExposureAutoROIBottom", bottom);
            _connected_device->SetIntegerNodeValue("RGB_ExposureAutoROIRight", right);
            _connected_device->CommandNodeExecute("RGB_ExposureAutoROISet");
        }
        
        bool cs_device::send_calib_data(std::vector<uint8_t>& hwm_cmd, bool verify)
        {
            // This is a private function and verification if "hwm_cmd" is "IMU calibration command" is done in cs_device::send_hwm

            uint64_t address, regLength;
            uint8_t readBuffer[GVCP_WRITEMEM_MAX_COUNT];
            uint8_t resendCount = 3;
            double maxWaitTime = 5.2;
            uint16_t size = 0;
            GEV_STATUS gevStatus;
            bool status = true;
            cs::d4_hwm_cmd* hwm = nullptr;

            if (hwm_cmd.size() > cs::d4_hwm_cmd_data_offset) {
                hwm = (cs::d4_hwm_cmd*)(hwm_cmd.data());
            }
            else {
                status = false;
                return status;
            }

            uint32_t restSize = hwm_cmd.size() - cs::d4_hwm_cmd_data_offset;
            uint8_t* sendBuffer = (UINT8*)(&hwm->Data[0]);

            if (!_calib_tx_once_flag) {
                // Verify HWm command TX buffer size
                status = _connected_device->GetNode("ImuCalibTxBuffer")->GetRegisterNodeLength(_calib_tx_reg_len);
                if (status) {
                    // Get HWm command TX buffer address
                    status = _connected_device->GetNode("ImuCalibTxBuffer")->GetRegisterNodeAddress(_calib_tx_reg_addr);
                    if (status) {
                        _calib_tx_once_flag = true;
                    }
                }
            }

            if ((_calib_tx_reg_len == 0) || (restSize > _calib_tx_reg_len) || (_calib_tx_reg_addr == 0)) {
                status = false;
                return status;
            }

            address = _calib_tx_reg_addr;

            // Send HWm command
            while (restSize > 0) {

                size = (restSize > GVCP_WRITEMEM_MAX_COUNT) ? GVCP_WRITEMEM_MAX_COUNT : restSize;

                int tryCount = resendCount;
                bool ok;

                do {
                    ok = true;
                    tryCount--;
                    ok = ok && _connected_device->SetMemory(address, size, sendBuffer, &gevStatus, maxWaitTime);
                    if (verify) {
                        ok = ok && _connected_device->GetMemory(address, size, (UINT8*)(&readBuffer[0]), &gevStatus, maxWaitTime);
                        if ((ok) && (memcmp(sendBuffer, readBuffer, size) != 0)) {
                            ok = false;
                        }
                    }
                } while ((tryCount > 0) && (!ok));

                if (!ok) {
                    status = false;
                    break;
                }

                restSize -= size;
                address += size;
                sendBuffer += size;
            }

            // Execute HWm TX command
            status &= _connected_device->CommandNodeExecute("ImuCalibTxBufferSend");

            // D435i imu calibration process - on write success, return data must be 4 byte long and first byte must be imu calib data hwm opcode
            if (status) {
                hwm_cmd.clear();
                hwm_cmd.resize(4);
                hwm_cmd[0] = cs::d4_hwm_cmd_opcode_imu_calib;
            }

            return status;
        }

        void cs_device::stream_params_lock(cs_stream stream)
        {
            if (!set_source_locked(stream, true))
                throw wrong_api_call_sequence_exception("Unable to lock source!");
        }

        void cs_device::stream_params_unlock(cs_stream stream)
        {
            if (!set_source_locked(stream, false))
                throw wrong_api_call_sequence_exception("Unable to unlock source!");
        }

        std::string cs_device::get_device_version()
        {
            return _connected_device->GetDeviceVersion();
        }

        std::string cs_device::get_ip_address()
        {
            return ip_address_to_string(_connected_device->GetIpAddress());
        }

        std::string cs_device::get_subnet_mask()
        {
            return ip_address_to_string(_connected_device->GetSubnetMask());
        }

        std::string cs_device::ip_address_to_string(uint32_t ip_address)
        {
            std::stringstream stream;
            stream << ((ip_address >> 24) & 0xFF) << "." 
                << ((ip_address >> 16) & 0xFF) << "." 
                << ((ip_address >> 8) & 0xFF) << "." 
                << ((ip_address) & 0xFF);
            return stream.str();
        }

        enum rs2_format cs_device::get_rgb_format()
        {
            return _rgb_pixel_format;
        }

        bool cs_device::is_infrared_supported()
        {
            return _infrared_supported;
        }

        bool cs_device::is_infrared_left_color_supported()
        {
            return _infrared_left_color_supported;
        }

        bool cs_device::is_temperature_supported()
        {
            if (!_temperature_supported_checked) {
                _temperature_supported = _connected_device->GetNode("DeviceTemperatureSelector") != nullptr;
                _temperature_supported_checked = true;
            }

            return _temperature_supported;
        }

        bool cs_device::is_rgb_led_toggle_supported()
        {
            if (!_rgb_led_toggle_supported_checked) {
                _rgb_led_toggle_supported = _connected_device->GetNode("RGB_LedToggle") != nullptr;
                _rgb_led_toggle_supported_checked = true;
            }
            return _rgb_led_toggle_supported;
        }

        double cs_device::get_device_timestamp_ms()
        {
            // DEBUG LOG for packet statistics, DEBUG LOG is outputed every 10000 miliseconds
            double this_call_time = std::chrono::duration<double, std::milli>(std::chrono::system_clock::now().time_since_epoch()).count();
            {
                // DEBUG LOG for driver statistics, called once per 10000 milliseconds, shared between devices,
                // mutex protected to ensure it is not called for every device 
                std::lock_guard<std::mutex> lck(_statistics_lock);
                double milisec_passed_device = this_call_time - _statistics_last_call_time_driver;
                if (milisec_passed_device >= 10000) {
                    if (_smcs_api->IsUsingKernelDriver()) {
                        LOG_DEBUG("GEV filter driver api packet statistics: " << get_api_packet_statistics());
                    }
                    else {
                        LOG_DEBUG("GEV socket driver api packet statistics: " << get_api_packet_statistics());
                    }
                    _statistics_last_call_time_driver = this_call_time;
                }
            }
            // DEBUG LOG for each device, called every 10000 milliseconds
            {
                double milisec_passed_device = this_call_time - _statistics_last_call_time_device;
                if (milisec_passed_device >= 10000) {
                    if (_smcs_api->IsUsingKernelDriver()) {
                        LOG_DEBUG("GEV filter driver device (SN:" << _device_info.serial << ") packet statistics: " << get_dev_packet_statistics());
                    }
                    else {
                        LOG_DEBUG("GEV socket driver device (SN:" << _device_info.serial << ") packet statistics: " << get_dev_packet_statistics());
                    }
                    _statistics_last_call_time_device = this_call_time;
                }
            }
            // End of DEBUG LOG code

            GEV_STATUS gev_status;
            
            bool status_latch = _connected_device->SetRegister((UINT32)_reg_map.at("GevTimestampControlLatch"), 2, &gev_status, 3.0, GVCP_FLAG_ACK_BIT);
            if (!status_latch) {
                throw io_exception("Unable to execute GevTimestampControlLatch");
            }

            INT64 timestamp = 0;
            UINT32 timestamp_high, timestamp_low;
            bool status_get_value_high = _connected_device->GetRegister((UINT32)_reg_map.at("GevTimestampValueHigh"), &timestamp_high, &gev_status, 3.0, GVCP_FLAG_ACK_BIT);
            bool status_get_value_low = _connected_device->GetRegister((UINT32)_reg_map.at("GevTimestampValueLow"), &timestamp_low, &gev_status, 3.0, GVCP_FLAG_ACK_BIT);

            if (status_get_value_high && status_get_value_low) {
                timestamp = (UINT64)timestamp_high << 32;
                timestamp = timestamp | (UINT64)timestamp_low;
            }
            else {
                throw io_exception("Unable to read GevTimestampValue");
            }

            /*
            if (!_connected_device->CommandNodeExecute("GevTimestampControlLatch"))
                throw io_exception("Unable to execute GevTimestampControlLatch");

            INT64 timestamp;
            if (!_connected_device->GetIntegerNodeValue("GevTimestampValue", timestamp))
                throw io_exception("Unable to read GevTimestampValue");    
            */

            return timestamp * _timestamp_to_ms_factor;
        }

        bool cs_device::is_software_trigger_supported()
        {
            if (!_software_trigger_supported_checked) {
                _software_trigger_supported = 
                    (_connected_device->GetNode("TriggerSoftwareAllSources") != nullptr) && 
                    (_connected_device->GetNode("TriggerSoftware") != nullptr);
                _software_trigger_supported_checked = true;
            }

            return _software_trigger_supported;
        }

        bool cs_device::is_line_debouncer_time_supported()
        {
            if (!_line_debouncer_time_supported_checked) {
                _line_debouncer_time_supported =
                ((_cs_firmware_version >= cs_firmware_version(1, 8, 0, 2) && (_device_info.id == CS_CAMERA_MODEL_D435e)) ||
                (_cs_firmware_version >= cs_firmware_version(1, 3, 0, 2) && (_device_info.id == CS_CAMERA_MODEL_D415e)) ||
                (_cs_firmware_version >= cs_firmware_version(1, 0, 0, 0) && (_device_info.id == CS_CAMERA_MODEL_D455e)));
                _line_debouncer_time_supported_checked = true;
            }
            return _line_debouncer_time_supported;
        }

        bool cs_device::is_rgb_raw16_supported() 
        {
            if (!_rgb_raw16_supported_checked) {
                _rgb_raw16_supported =
                    ((_cs_firmware_version > cs_firmware_version(1, 9, 0, 0) && (_device_info.id == CS_CAMERA_MODEL_D435e)) ||
                    (_cs_firmware_version > cs_firmware_version(1, 4, 0, 0) && (_device_info.id == CS_CAMERA_MODEL_D415e)) ||
                    (_cs_firmware_version > cs_firmware_version(1, 0, 0, 0) && (_device_info.id == CS_CAMERA_MODEL_D455e)));
                _rgb_raw16_supported_checked = true;
            }
            return _rgb_raw16_supported;
        }

        bool cs_device::is_burst_mode_supported()
        {
            if (!_burst_mode_supported_checked) {
                smcs::StringList trigger_selector_list;
                _connected_device->GetEnumNodeValuesList("TriggerSelector", trigger_selector_list);
                for (const auto& trigger_selector : trigger_selector_list) {
                    if (trigger_selector == "FrameBurstStart")
                        _burst_mode_supported = true;
                }
                _burst_mode_supported_checked = true;
            }

            return _burst_mode_supported;
        }

        bool cs_device::is_color_genlock_mode_supported()
        {
            if (!_color_genlock_mode_supported_checked) {
                INT64 val;
                _connected_device->GetIntegerNodeValue("SourceControlSelector", val);
                select_source(CS_STREAM_COLOR);
                smcs::StringList trigger_type_list;
                _connected_device->GetEnumNodeValuesList("TriggerType", trigger_type_list);
                for (const auto& trigger_type : trigger_type_list) {
                    if (trigger_type == "GenLock")
                        _color_genlock_mode_supported = true;
                }
                _color_genlock_mode_supported_checked = true;
                select_source((librealsense::cs_stream)val);
            }
            return _color_genlock_mode_supported;
        }

        bool cs_device::is_imu_supported()
        {
            if (!_imu_supported_checked) {
                /*_imu_supported =
                    (_cs_firmware_version > cs_firmware_version(1, 10, 0, 0) && (_device_info.id == CS_CAMERA_MODEL_D435e) ||
                    (_cs_firmware_version > cs_firmware_version(1, 5, 0, 0) && (_device_info.id == CS_CAMERA_MODEL_D415e)));*/

                if ((_connected_device->GetNode("EventAccelerometer")) && 
                    (_connected_device->GetNode("EventGyroscope"))) {
                    _imu_supported = true;
                }
                else {
                    _imu_supported = false;
                }

                _imu_supported_checked = true;
            }
            return _imu_supported;
        }


        int cs_device::get_imu_id()
        {
            int id = IMU_INVALID_ID;
            if (_imu_supported) {
                id = BMI055_IDENTIFIER;
                
                if (_connected_device->IsAvailable("ImuIdentifierReg")) {
                    INT64 val;
                    if (_connected_device->GetIntegerNodeValue("ImuIdentifierReg", val)) {
                        id = val;
                    }
                }
            }
            return id;
        }

        void cs_device::capture_loop(cs_stream stream, UINT32 channel)
        {
            try
            {
                while(_is_capturing[stream])
                {
                    image_poll(stream, channel);
                }
            }
            catch (const std::exception& ex)
            {
                LOG_ERROR(ex.what());

                librealsense::notification n = {RS2_NOTIFICATION_CATEGORY_UNKNOWN_ERROR, 0, RS2_LOG_SEVERITY_ERROR, ex.what()};

                _error_handler[stream](n);
            }
        }

        void cs_device::image_poll(cs_stream stream, UINT32 channel)
        {
            smcs::IImageInfo image_info_ = nullptr;

            if (_connected_device.IsValid() && _connected_device->IsConnected()) {
                if (_connected_device->WaitForImage(1, channel))
                {
                    _connected_device->GetImageInfo(&image_info_, channel);

                    if (image_info_ != nullptr) {

                        UINT32 width, height;
                        image_info_->GetSize(width, height);

                        UINT32 pixel_type;
                        image_info_->GetPixelType(pixel_type);
                        float Bpp = GvspGetBitsPerPixel((GVSP_PIXEL_TYPES)pixel_type) / 8.0;

                        UINT32 image_size = (UINT32)(width * height * Bpp);

                        bool is_meta_valid = false;
                        UINT8* meta_start = nullptr;

                        // check if metadata is supported and enabled
                        if (_metadata_supported && _metadata_toggle_value) {
                            // currently metadata is available only on StreamChannel0 (depth, calib) and StreamChannel2 (IR l/r, IR l, IR lcolor)
                            if ((channel == CS_GEV_STREAM_CHANNEL0) || (channel == CS_GEV_STREAM_CHANNEL2)) {
                                // metadata is appended to each image, therefore image is bigger for 2 additional lines
                                height -= CS_METADATA_SIZE;
                                // re-calc image size
                                image_size = width * height * Bpp;
                            }
                        }

                        if (!is_profile_format(width, height, pixel_type, _profiles[stream])) {
                            _connected_device->PopImage(image_info_);
                            return;
                        }

                        if (_metadata_toggle_value) {
                            meta_start = image_info_->GetRawData() + image_size;
                            is_meta_valid = check_metadata(meta_start);
                        }

                        auto frame_counter = image_info_->GetImageID();
                        auto timestamp_us = (uint64_t)image_info_->GetCameraTimestamp();
                        double timestamp = timestamp_us * TIMESTAMP_USEC_TO_MSEC;

                        auto im = image_info_->GetRawData();
                        {
                            std::lock_guard<std::mutex> lock(_stream_lock);

                            clear_metadata(_md);

                            // use data from metadata
                            if (is_meta_valid) {
                                handle_metadata(_md, meta_start);
                            }

                            // use frame_counter data from smcs::IImageInfo for every stream
                            // (preserve consistency with color stream from camera (no metadata for color stream))
                            _md.capture_timing.frame_counter = frame_counter;

                            frame_object fo{ image_size, sizeof(metadata_framos_basic), im, &_md, timestamp };
                            try {
                                _callbacks[stream](_profiles[stream], fo, []() {});
                            }
                            catch (const std::bad_function_call& e) {
                                LOG_ERROR(e.what());
                            }
                            catch (...) {
                                LOG_ERROR("Image poll callback");
                            }
                        }

                        _connected_device->PopImage(image_info_);
                    }
                }
            }
            else {
                throw camera_disconnected_exception("Polling images from disconnected device!");
            }
        }

        bool cs_device::is_metadata_supported()
        {
            if (!_metadata_supported_checked) {
                if (_connected_device->IsAvailable("MetadataToggle")) {
                    _metadata_supported = true;

                    /* Cache _metadata_toggle_value.
                    *  This is done so that genicam mechanism is unburdened from frequent calls. */
                    _connected_device->GetBooleanNodeValue("MetadataToggle", _metadata_toggle_value);
                }
                else {
                    _metadata_supported = false;
                }

                _metadata_supported_checked = true;
            }

            return _metadata_supported;
        }

        bool cs_device::check_metadata(UINT8* metadata)
        {
            bool status = false;

            if (metadata != nullptr) {
                md_depth_y_normal_mode* meta_data = (md_depth_y_normal_mode*)metadata;

                if ((meta_data->intel_capture_timing.header.md_type_id == md_type::META_DATA_INTEL_CAPTURE_TIMING_ID) &&
                    (meta_data->intel_capture_stats.header.md_type_id == md_type::META_DATA_CAPTURE_STATS_ID) &&
                    (meta_data->intel_depth_control.header.md_type_id == md_type::META_DATA_INTEL_DEPTH_CONTROL_ID) &&
                    (meta_data->intel_configuration.header.md_type_id == md_type::META_DATA_INTEL_CONFIGURATION_ID)) {

                    status = true;
                }
                else {
                    // TODO NH - Log Error!
                }
            }

            return status;
        }

        void cs_device::handle_metadata(metadata_framos_basic& md, UINT8* metadata)
        {
            if (metadata != nullptr) {

                md_depth_y_normal_mode* meta_data = (md_depth_y_normal_mode*)metadata;

                md.header.info = 0x00;
                md.header.length = _md_size;
                md.header.source_clock[0] = 0; // TODO NH
                md.header.source_clock[1] = 0; // TODO NH
                md.header.source_clock[2] = 0; // TODO NH
                md.header.source_clock[3] = 0; // TODO NH
                md.header.source_clock[4] = 0; // TODO NH
                md.header.source_clock[5] = 0; // TODO NH

                memcpy(&md.capture_timing.header.md_type_id, meta_data, md.header.length);

                // NH - from Intel's description file - more on xwiki report
                md.header.timestamp = md.capture_stats.reserved;
            }
            else {
                // TODO NH - Log Error!
            }
        }

        void cs_device::clear_metadata(metadata_framos_basic& md)
        {
            memset(&md.capture_timing.header.md_type_id, 0, _md_size);
        }

        void cs_device::probe_and_commit(stream_profile profile, frame_callback callback, cs_stream stream)
        {
            if (stream < _number_of_streams)
            {
                if (!_is_capturing[stream])
                {
                    _profiles[stream] = profile;
                    _callbacks[stream] = callback;
                }
                else {
                    throw wrong_api_call_sequence_exception("Device already streaming!");
                }
            }
            else throw wrong_api_call_sequence_exception("Unsuported streaming type!");
        }

        void cs_device::set_format(stream_profile profile, cs_stream stream)
        {
            if (!set_region(stream, true))
                throw wrong_api_call_sequence_exception("Failed to set region!");

            // Resolution is read-only on D435e with FW 1.3.4.0
            std::string new_resolution = "Res_" + std::to_string(profile.width) + "x" + std::to_string(profile.height);
            std::string old_resolution;
            if (_connected_device->GetStringNodeValue("Resolution", old_resolution) && old_resolution != new_resolution)
                if (!_connected_device->SetStringNodeValue("Resolution", new_resolution))
                    throw wrong_api_call_sequence_exception("Failed to set resolution!");

            _connected_device->GetStringNodeValue("Resolution", old_resolution);

            // FrameRate does not exist on D435e with FW 1.3.4.0
            if (_connected_device->GetNode("FrameRate") != nullptr 
                && !_connected_device->SetStringNodeValue("FrameRate", "FPS_" + std::to_string(profile.fps)))
                throw wrong_api_call_sequence_exception("Failed to set framerate!");
        }

        void cs_device::set_trigger_mode(float mode, cs_stream stream)
        {
            auto sync_mode = static_cast<int>(mode);

            bool operating_mode_supported = _connected_device->GetNode("STR_OperatingMode") != nullptr;

            select_source(stream);

            smcs::StringList trigger_selector_list;
            _connected_device->GetEnumNodeValuesList("TriggerSelector", trigger_selector_list);
            for (const auto& trigger_selector : trigger_selector_list) {
                _connected_device->SetStringNodeValue("TriggerSelector", trigger_selector);
                _connected_device->SetStringNodeValue("TriggerMode", "Off");
            }

            if (stream == CS_STREAM_COLOR) {
                switch (sync_mode) {
                case CS_INTERCAM_SYNC_EXTERNAL_COLOR:
                    _connected_device->SetStringNodeValue("TriggerSelector", "FrameStart");
                    _connected_device->SetStringNodeValue("TriggerType", "ExternalEvent");
                    _connected_device->SetStringNodeValue("TriggerMode", "On");
                    break;
                case CS_INTERCAM_SYNC_EXTERNAL_BURST_COLOR:
                    _connected_device->SetStringNodeValue("TriggerSelector", "FrameBurstStart");
                    _connected_device->SetStringNodeValue("TriggerType", "ExternalEvent");
                    _connected_device->SetStringNodeValue("TriggerMode", "On");
                    break;
                default:
                    _connected_device->SetStringNodeValue("TriggerSelector", "FrameStart");
                    _connected_device->SetStringNodeValue("TriggerType", "MultiCam_Sync");
                    _connected_device->SetStringNodeValue("TriggerMode", "Off");
                }
            }
            else {
                switch (sync_mode) {
                case CS_INTERCAM_SYNC_SLAVE:
                    _connected_device->SetStringNodeValue("TriggerSelector", "FrameStart");
                    _connected_device->SetStringNodeValue("TriggerType", "MultiCam_Sync");
                    if (operating_mode_supported) {
                        _connected_device->SetStringNodeValue("STR_OperatingMode", "Slave");
                    }
                    else {
                        _connected_device->SetStringNodeValue("LineSelector", "Line1");
                        _connected_device->SetStringNodeValue("LineSource", "UserOutput1");
                    }
                    _connected_device->SetStringNodeValue("TriggerMode", "On");
                    break;
                case CS_INTERCAM_SYNC_MASTER:
                    _connected_device->SetStringNodeValue("TriggerSelector", "FrameStart");
                    _connected_device->SetStringNodeValue("TriggerType", "MultiCam_Sync");
                    if (operating_mode_supported) {
                        _connected_device->SetStringNodeValue("STR_OperatingMode", "Master");
                    }
                    else {
                        _connected_device->SetStringNodeValue("LineSelector", "Line1");
                        _connected_device->SetStringNodeValue("LineSource", "VSync");
                    }
                    _connected_device->SetStringNodeValue("TriggerMode", "Off");

                    break;
                case CS_INTERCAM_SYNC_EXTERNAL:
                    _connected_device->SetStringNodeValue("TriggerSelector", "FrameStart");
                    _connected_device->SetStringNodeValue("TriggerType", "ExternalEvent");
                    if (operating_mode_supported) {
                        _connected_device->SetStringNodeValue("STR_OperatingMode", "Slave");
                    }
                    else {
                        _connected_device->SetStringNodeValue("LineSelector", "Line1");
                        _connected_device->SetStringNodeValue("LineSource", "VSync");
                    }
                    _connected_device->SetStringNodeValue("TriggerMode", "On");
                    break;

                case CS_INTERCAM_SYNC_EXTERNAL_BURST:
                    _connected_device->SetStringNodeValue("TriggerSelector", "FrameBurstStart");
                    _connected_device->SetStringNodeValue("TriggerType", "ExternalEvent");
                    if (operating_mode_supported) {
                        _connected_device->SetStringNodeValue("STR_OperatingMode", "Slave");
                    }
                    else {
                        _connected_device->SetStringNodeValue("LineSelector", "Line1");
                        _connected_device->SetStringNodeValue("LineSource", "VSync");
                    }
                    _connected_device->SetStringNodeValue("TriggerMode", "On");
                    break;
                default:
                    _connected_device->SetStringNodeValue("TriggerSelector", "FrameStart");
                    _connected_device->SetStringNodeValue("TriggerType", "MultiCam_Sync");
                    if (operating_mode_supported) {
                        _connected_device->SetStringNodeValue("STR_OperatingMode", "Default");
                    }
                    else {
                        _connected_device->SetStringNodeValue("LineSelector", "Line1");
                        _connected_device->SetStringNodeValue("LineSource", "UserOutput1");
                    }
                    _connected_device->SetStringNodeValue("TriggerMode", "Off");
                }
            }
        }

        void cs_device::set_trigger_mode_gs(float mode, cs_stream stream)
        {
            auto sync_mode = static_cast<int>(mode);

            bool operating_mode_supported = _connected_device->GetNode("STR_OperatingMode") != nullptr;

            select_source(stream);

            smcs::StringList trigger_selector_list;
            _connected_device->GetEnumNodeValuesList("TriggerSelector", trigger_selector_list);
            for (const auto& trigger_selector : trigger_selector_list) {
                _connected_device->SetStringNodeValue("TriggerSelector", trigger_selector);
                _connected_device->SetStringNodeValue("TriggerMode", "Off");
            }

            if (stream == CS_STREAM_COLOR) {
                switch (sync_mode) {
                case CS_INTERCAM_SYNC_EXTERNAL_COLOR_GS:
                    _connected_device->SetStringNodeValue("TriggerSelector", "FrameStart");
                    _connected_device->SetStringNodeValue("TriggerType", "ExternalEvent");
                    _connected_device->SetStringNodeValue("TriggerMode", "On");
                    break;
                case CS_INTERCAM_SYNC_EXTERNAL_BURST_COLOR_GS:
                    _connected_device->SetStringNodeValue("TriggerSelector", "FrameBurstStart");
                    _connected_device->SetStringNodeValue("TriggerType", "ExternalEvent");
                    _connected_device->SetStringNodeValue("TriggerMode", "On");
                    break;
                case CS_INTERCAM_SYNC_GENLOCK_COLOR_GS:
                    _connected_device->SetStringNodeValue("TriggerSelector", "FrameBurstStart");
                    _connected_device->SetStringNodeValue("TriggerType", "GenLock");
                    _connected_device->SetStringNodeValue("TriggerMode", "On");
                    break;
                default:
                    _connected_device->SetStringNodeValue("TriggerSelector", "FrameStart");
                    _connected_device->SetStringNodeValue("TriggerType", "MultiCam_Sync");
                    _connected_device->SetStringNodeValue("TriggerMode", "Off");
                }
            }
            else {
                switch (sync_mode) {
                case CS_INTERCAM_SYNC_SLAVE_GS:
                case CS_INTERCAM_SYNC_FULL_SLAVE_GS:
                    _connected_device->SetStringNodeValue("TriggerSelector", "FrameStart");
                    _connected_device->SetStringNodeValue("TriggerType", "MultiCam_Sync");
                    if (operating_mode_supported) {
                        _connected_device->SetStringNodeValue("STR_OperatingMode", "Slave");
                    }
                    else {
                        _connected_device->SetStringNodeValue("LineSelector", "Line1");
                        _connected_device->SetStringNodeValue("LineSource", "UserOutput1");
                    }
                    _connected_device->SetStringNodeValue("TriggerMode", "On");
                    if (sync_mode == CS_INTERCAM_SYNC_FULL_SLAVE_GS)
                        _is_full_slave_mode = true;
                    else
                        _is_full_slave_mode = false;
                    break;
                case CS_INTERCAM_SYNC_MASTER_GS:
                    _connected_device->SetStringNodeValue("TriggerSelector", "FrameStart");
                    _connected_device->SetStringNodeValue("TriggerType", "MultiCam_Sync");
                    if (operating_mode_supported) {
                        _connected_device->SetStringNodeValue("STR_OperatingMode", "Master");
                    }
                    else {
                        _connected_device->SetStringNodeValue("LineSelector", "Line1");
                        _connected_device->SetStringNodeValue("LineSource", "VSync");
                    }
                    _connected_device->SetStringNodeValue("TriggerMode", "Off");

                    break;
                case CS_INTERCAM_SYNC_EXTERNAL_GS:
                    _connected_device->SetStringNodeValue("TriggerSelector", "FrameStart");
                    _connected_device->SetStringNodeValue("TriggerType", "ExternalEvent");
                    if (operating_mode_supported) {
                        _connected_device->SetStringNodeValue("STR_OperatingMode", "Slave");
                    }
                    else {
                        _connected_device->SetStringNodeValue("LineSelector", "Line1");
                        _connected_device->SetStringNodeValue("LineSource", "VSync");
                    }
                    _connected_device->SetStringNodeValue("TriggerMode", "On");
                    break;
                case CS_INTERCAM_SYNC_EXTERNAL_BURST_GS:
                    _connected_device->SetStringNodeValue("TriggerSelector", "FrameBurstStart");
                    _connected_device->SetStringNodeValue("TriggerType", "ExternalEvent");
                    if (operating_mode_supported) {
                        _connected_device->SetStringNodeValue("STR_OperatingMode", "Slave");
                    }
                    else {
                        _connected_device->SetStringNodeValue("LineSelector", "Line1");
                        _connected_device->SetStringNodeValue("LineSource", "VSync");
                    }
                    _connected_device->SetStringNodeValue("TriggerMode", "On");
                    break;
                default:
                    if (sync_mode > CS_INTERCAM_SYNC_FULL_SLAVE_GS && sync_mode < CS_INTERCAM_SYNC_EXTERNAL_GS)
                    {
                        _connected_device->SetStringNodeValue("TriggerSelector", "FrameBurstStart");
                        _connected_device->SetStringNodeValue("TriggerType", "GenLock");

                        if (operating_mode_supported) {
                            _connected_device->SetStringNodeValue("STR_OperatingMode", "Slave");
                        }
                        else {
                            _connected_device->SetStringNodeValue("LineSelector", "Line1");
                            _connected_device->SetStringNodeValue("LineSource", "VSync");
                        }
                        _connected_device->SetStringNodeValue("TriggerMode", "On");
                        auto burst_count = sync_mode - 3;
                        _connected_device->SetIntegerNodeValue("AcquisitionBurstFrameCount", burst_count);
                    }
                    else
                    {
                        _connected_device->SetStringNodeValue("TriggerType", "MultiCam_Sync");
                        if (operating_mode_supported) {
                            _connected_device->SetStringNodeValue("STR_OperatingMode", "Default");
                        }
                        else {
                            _connected_device->SetStringNodeValue("LineSelector", "Line1");
                            _connected_device->SetStringNodeValue("LineSource", "UserOutput1");
                        }
                        _connected_device->SetStringNodeValue("TriggerMode", "Off");
                    }
                }
            }
        }

        float cs_device::get_trigger_mode(cs_stream stream)
        {
            select_source(stream);

            bool operating_mode_supported = _connected_device->GetNode("STR_OperatingMode") != nullptr;
            std::map < std::string, std::map<std::string, std::string>> trigger_options;

            std::string trigger_type, trigger_source, line_source, trigger_mode, operating_mode;
            _connected_device->GetStringNodeValue("TriggerType", trigger_type);

            _connected_device->SetStringNodeValue("LineSelector", "Line1");
            _connected_device->GetStringNodeValue("LineSource", line_source);
            _connected_device->GetStringNodeValue("TriggerMode", trigger_mode);
            if (trigger_mode == "On") {
                _connected_device->GetStringNodeValue("TriggerSource", trigger_source);
            }
            if (operating_mode_supported) {
                _connected_device->GetStringNodeValue("STR_OperatingMode", operating_mode);
            }

            trigger_options = get_trigger_properties(stream);

            switch (stream) {
            case CS_STREAM_COLOR:
                if (trigger_options["FrameStart"]["TriggerType"] == "ExternalEvent" && trigger_options["FrameStart"]["TriggerMode"] == "On")
                    return CS_INTERCAM_SYNC_EXTERNAL_COLOR;
                else if (trigger_options["FrameBurstStart"]["TriggerType"] == "ExternalEvent" && trigger_options["FrameBurstStart"]["TriggerMode"] == "On")
                    return CS_INTERCAM_SYNC_EXTERNAL_BURST_COLOR;
                else
                    return CS_INTERCAM_SYNC_DEFAULT_COLOR;
            default:
                if (operating_mode_supported) {
                    if (trigger_options["FrameStart"]["TriggerType"] == "MultiCam_Sync" && operating_mode == "Slave" && trigger_options["FrameStart"]["TriggerMode"] == "On")
                        return CS_INTERCAM_SYNC_SLAVE;
                    else if (trigger_options["FrameStart"]["TriggerType"] == "MultiCam_Sync" && operating_mode == "Master" && trigger_options["FrameStart"]["TriggerMode"] == "Off" && (is_burst_mode_supported() ? trigger_options["FrameBurstStart"]["TriggerMode"] == "Off" : 1))
                        return CS_INTERCAM_SYNC_MASTER;
                    else if (trigger_options["FrameStart"]["TriggerType"] == "ExternalEvent" && trigger_options["FrameStart"]["TriggerMode"] == "On")
                        return CS_INTERCAM_SYNC_EXTERNAL;
                    else if (trigger_options["FrameBurstStart"]["TriggerType"] == "ExternalEvent" && trigger_options["FrameBurstStart"]["TriggerMode"] == "On")
                        return CS_INTERCAM_SYNC_EXTERNAL_BURST;
                    else
                        return CS_INTERCAM_SYNC_DEFAULT;
                }
                else {
                    if (trigger_type == "MultiCam_Sync" && trigger_source == "Line1" && line_source == "UserOutput1" && trigger_mode == "On")
                        return CS_INTERCAM_SYNC_SLAVE;
                    else if (trigger_type == "MultiCam_Sync" && line_source == "VSync" && trigger_mode == "Off")
                        return CS_INTERCAM_SYNC_MASTER;
                    else if (trigger_type == "ExternalEvent" && (trigger_source == "Line1" || trigger_source == "Software") && (line_source == "VSync" || line_source == "UserOutput1") && trigger_mode == "On")
                        return CS_INTERCAM_SYNC_EXTERNAL;
                    else
                        return CS_INTERCAM_SYNC_DEFAULT;
                }
            }
        }

        float cs_device::get_trigger_mode_gs(cs_stream stream)
        {
            select_source(stream);

            bool operating_mode_supported = _connected_device->GetNode("STR_OperatingMode") != nullptr;
            std::map < std::string, std::map<std::string, std::string>> trigger_options;

            std::string trigger_type, trigger_source, line_source, trigger_mode, operating_mode, trigger_selector;
            _connected_device->GetStringNodeValue("TriggerType", trigger_type);

            _connected_device->SetStringNodeValue("LineSelector", "Line1");
            _connected_device->GetStringNodeValue("LineSource", line_source);
            _connected_device->GetStringNodeValue("TriggerMode", trigger_mode);
            if (trigger_mode == "On") {
                _connected_device->GetStringNodeValue("TriggerSource", trigger_source);
            }
            if (operating_mode_supported) {
                _connected_device->GetStringNodeValue("STR_OperatingMode", operating_mode);
            }

            trigger_options = get_trigger_properties(stream);

            switch (stream) {
            case CS_STREAM_COLOR:
                if (trigger_options["FrameStart"]["TriggerType"] == "ExternalEvent" && trigger_options["FrameStart"]["TriggerMode"] == "On")
                    return CS_INTERCAM_SYNC_EXTERNAL_COLOR_GS;
                else if (trigger_options["FrameBurstStart"]["TriggerType"] == "ExternalEvent" && trigger_options["FrameBurstStart"]["TriggerMode"] == "On")
                    return CS_INTERCAM_SYNC_EXTERNAL_BURST_COLOR_GS;
                else if (trigger_options["FrameBurstStart"]["TriggerType"] == "GenLock" && trigger_options["FrameBurstStart"]["TriggerMode"] == "On" && trigger_options["FrameStart"]["TriggerMode"] == "Off")
                    return CS_INTERCAM_SYNC_GENLOCK_COLOR_GS;
                else
                    return CS_INTERCAM_SYNC_DEFAULT_COLOR_GS;

            default:
                if (operating_mode_supported) {
                    if (trigger_options["FrameStart"]["TriggerType"] == "MultiCam_Sync" && trigger_options["FrameStart"]["TriggerSource"] == "Line1" && operating_mode == "Slave" && trigger_options["FrameStart"]["TriggerMode"] == "On" && trigger_options["FrameBurstStart"]["TriggerMode"] == "Off")
                        if (!_is_full_slave_mode)
                            return CS_INTERCAM_SYNC_SLAVE_GS;
                        else 
                            return CS_INTERCAM_SYNC_FULL_SLAVE_GS;
                    else if (trigger_options["FrameStart"]["TriggerType"] == "MultiCam_Sync" && operating_mode == "Master" && trigger_options["FrameStart"]["TriggerMode"] == "Off" && trigger_options["FrameBurstStart"]["TriggerMode"] == "Off")
                        return CS_INTERCAM_SYNC_MASTER_GS;
                    else if (trigger_options["FrameStart"]["TriggerType"] == "ExternalEvent" && trigger_options["FrameStart"]["TriggerMode"] == "On")
                        return CS_INTERCAM_SYNC_EXTERNAL_GS;
                    else if (trigger_options["FrameBurstStart"]["TriggerType"] == "ExternalEvent" && trigger_options["FrameBurstStart"]["TriggerMode"] == "On")
                        return CS_INTERCAM_SYNC_EXTERNAL_BURST_GS;
                    else if (trigger_options["FrameBurstStart"]["TriggerType"] == "GenLock" && trigger_options["FrameBurstStart"]["TriggerMode"] == "On" && trigger_options["FrameStart"]["TriggerMode"] == "Off") 
                    {
                        INT64 burst_count;
                        _connected_device->GetIntegerNodeValue("AcquisitionBurstFrameCount", burst_count);
                        return burst_count + 3;
                    }
                    else
                        return CS_INTERCAM_SYNC_DEFAULT_GS;
                }
                else {
                    if (trigger_type == "MultiCam_Sync" && trigger_source == "Line1" && line_source == "UserOutput1" && trigger_mode == "On")
                        return CS_INTERCAM_SYNC_SLAVE_GS;
                    else if (trigger_type == "MultiCam_Sync" && line_source == "VSync" && trigger_mode == "Off")
                        return CS_INTERCAM_SYNC_MASTER_GS;
                    else if (trigger_type == "ExternalEvent" && (trigger_source == "Line1" || trigger_source == "Software") && (line_source == "VSync" || line_source == "UserOutput1") && trigger_mode == "On")
                        return CS_INTERCAM_SYNC_EXTERNAL_GS;
                    else
                        return CS_INTERCAM_SYNC_DEFAULT_GS;
                }
            }
        }

        std::map < std::string, std::map<std::string, std::string>> cs_device::get_trigger_properties(cs_stream stream)
        {
            bool operating_mode_supported = _connected_device->GetNode("STR_OperatingMode") != nullptr;
            std::string trigger_type, trigger_source, line_source, trigger_mode, operating_mode;
            std::string trigger_frame = "TriggerType";

            std::map < std::string, std::map<std::string, std::string>> trigger_options;
            std::map<std::string, std::string> custom_trigger_options;

            std::string trigger_selector_keep;
            _connected_device->GetStringNodeValue("TriggerSelector", trigger_selector_keep);
            

            smcs::StringList trigger_selector_list;
            _connected_device->GetEnumNodeValuesList("TriggerSelector", trigger_selector_list);
            for (const auto& trigger_selector : trigger_selector_list) {
                _connected_device->SetStringNodeValue("TriggerSelector", trigger_selector);
                
                _connected_device->GetStringNodeValue("TriggerType", trigger_type);
                custom_trigger_options["TriggerType"] = trigger_type;

                _connected_device->SetStringNodeValue("LineSelector", "Line1");
                _connected_device->GetStringNodeValue("LineSource", line_source);
                custom_trigger_options["LineSource"] = line_source;
                _connected_device->GetStringNodeValue("TriggerMode", trigger_mode);
                custom_trigger_options["TriggerMode"] = trigger_mode;
                _connected_device->GetStringNodeValue("TriggerSource", trigger_source);
                custom_trigger_options["TriggerSource"] = trigger_source;

                trigger_options[trigger_selector] = custom_trigger_options;
            }

            _connected_device->SetStringNodeValue("TriggerSelector", trigger_selector_keep);

            return trigger_options;
        }

        void cs_device::update_external_trigger_mode_flag(cs_stream stream, float value)
        {
            switch (stream)
            {
            case CS_STREAM_COLOR:
                if (value == 1.f) set_device_option_sw_trigger_all_flag_sn(_device_info.serial + "_RGB", true); else set_device_option_sw_trigger_all_flag_sn(_device_info.serial + "_RGB", false);
                break;
            case CS_STREAM_DEPTH:
            case CS_STREAM_IR_LEFT:
            case CS_STREAM_IR_RIGHT:
            case CS_STREAM_IR_LEFT_COLOR:
                if (value == 3.f) set_device_option_sw_trigger_all_flag_sn(_device_info.serial + "_Stereo", true); else set_device_option_sw_trigger_all_flag_sn(_device_info.serial + "_Stereo", false);
                break;
            default: throw linux_backend_exception(to_string() << "wrong stream cid ");
            }

        }

        bool cs_device::get_intercam_mode(cs_stream stream)
        {
            switch (stream) {
            case CS_STREAM_COLOR:
                return get_device_option_sw_trigger_all_flag_sn(_device_info.serial + "_RGB");
            default:
                return get_device_option_sw_trigger_all_flag_sn(_device_info.serial + "_Stereo");;
            }
        }

        INT64 cs_device::get_optimal_inter_packet_delay(INT64 packet_size_bytes, INT64 link_speed_mbitps)
        {
            constexpr int eth_packet_overhead = 38;
            double eth_packet_size = packet_size_bytes + eth_packet_overhead;
            constexpr float mbitps_to_nspbyte_factor = 0.008;
            float ns_per_byte = link_speed_mbitps * mbitps_to_nspbyte_factor;
            constexpr float ns_to_ms_factor = 0.001;
            double time_to_transfer_packet_us = eth_packet_size * ns_per_byte * ns_to_ms_factor;
            time_to_transfer_packet_us = ceil(time_to_transfer_packet_us + 0.5);

            return static_cast<INT64>(time_to_transfer_packet_us);
        }

        std::string cs_device::get_api_packet_statistics()
        {
            std::stringstream ss;
            auto stat_names = {
                "PacketResendsAmount",
                "LostPackets",
                "LostImages",
                "IgnoredPackets",
                "IncompleteImages",
                "AllPackets",
                "LeaderPackets",
                "PayloadPackets",
                "TrailerPackets",
                "FrameTimeExceeded",
                "ResendTimeExceeded",
                "ResendRetriesExceeded",
                "UnknownDevice",
                "UnknownPackets"
            };
            for (auto const& stat_name : stat_names) {
                auto stat_node = _smcs_api->GetApiParametersNode(stat_name);
                INT64 stat_value;
                if (stat_node && stat_node->GetIntegerNodeValue(stat_value))
                    ss << std::endl << "    " << stat_name << ": " << stat_value;
            }
            return ss.str();
        }

        std::string cs_device::get_dev_packet_statistics()
        {
            smcs::StringList ch_selector_list;
            std::stringstream ss;

            auto dev_stat_names = {
                "PacketResendsAmount",
                "LostPackets",
                "LostImages",
                "IgnoredPackets",
                "IncompleteImages"
            };

            auto stat_node_ch = _connected_device->GetStatisticsNode("ChannelSelector");
            stat_node_ch->GetEnumNodeValuesList(ch_selector_list);

            for (const auto& ch_selector : ch_selector_list) {
                ss << std::endl << "    " << ch_selector << ": ";
                stat_node_ch->SetStringNodeValue(ch_selector);

                for (auto const& dev_stat_name : dev_stat_names) {
                    auto stat_node = _connected_device->GetStatisticsNode(dev_stat_name);
                    INT64 stat_value;
                    if (stat_node && stat_node->GetIntegerNodeValue(stat_value))
                        ss << std::endl << "        " << dev_stat_name << ": " << stat_value;
                }
            }
            return ss.str();
        }

        bool cs_device::inc_device_count_sn(std::string serial_num)
        {
            bool result = true;

            auto it = _cs_device_num_objects_sn.find(serial_num);
            if (it == _cs_device_num_objects_sn.end()) {    // does not exist
                _cs_device_num_objects_sn.insert({serial_num, 1});
            } 
            else {
                it->second++;
            }

            return result;
        }

        bool cs_device::dec_device_count_sn(std::string serial_num)
        {
            bool result = true;

            auto it = _cs_device_num_objects_sn.find(serial_num);
            if (it == _cs_device_num_objects_sn.end()) {    // does not exist
                result = false;
            }
            else {
                it->second--;
            }

            return result;
        }

        int cs_device::get_device_count_sn(std::string serial_num)
        {
            int devCount = -1;
            
            auto it = _cs_device_num_objects_sn.find(serial_num);
            if (it == _cs_device_num_objects_sn.end()) {    // does not exist
                devCount = -1;
            }
            else {
                devCount = it->second;
            }

            return devCount;
        }

        bool cs_device::set_device_init_flag_sn(std::string serial_num, bool set_init_flag)
        {
            bool result = true;

            auto it = _cs_device_initialized_sn.find(serial_num);
            if (it == _cs_device_initialized_sn.end()) {    // does not exist
                _cs_device_initialized_sn.insert({serial_num, set_init_flag});
            }
            else {
                it->second = set_init_flag;
            }

            return result;
        }

        bool cs_device::get_device_init_flag_sn(std::string serial_num)
        {
            bool flag = false;

            auto it = _cs_device_initialized_sn.find(serial_num);
            if (it == _cs_device_initialized_sn.end()) {    // does not exist
                flag = false;
            }
            else {
                flag = it->second;
            }

            return flag;
        }

        bool cs_device::set_device_option_sw_trigger_all_flag_sn(std::string serial_num, bool set_trigger_all_flag)
        {
            bool result = true;

            auto it = _cs_device_option_sw_trigger_all_flag_sn.find(serial_num);
            if (it == _cs_device_option_sw_trigger_all_flag_sn.end()) {    // does not exist
                _cs_device_option_sw_trigger_all_flag_sn.insert({ serial_num, set_trigger_all_flag });
            }
            else {
                it->second = set_trigger_all_flag;
            }

            return result;
        }

        bool cs_device::get_device_option_sw_trigger_all_flag_sn(std::string serial_num)
        {
            bool flag = false;

            auto it = _cs_device_option_sw_trigger_all_flag_sn.find(serial_num);
            if (it == _cs_device_option_sw_trigger_all_flag_sn.end()) {    // does not exist
                flag = false;
            }
            else {
                flag = it->second;
            }

            return flag;
        }

        std::string cs_device::get_serial() const
        {
            return _connected_device->GetSerialNumber();
        }

        smcs::IDevice cs_device::get_cs_device() const
        {
            return _connected_device;
        }
    }

    std::vector<uint8_t> cs_command_transfer::send_receive(const std::vector<uint8_t>& data, int, bool require_response)
    {
        if (data.size() > HW_MONITOR_BUFFER_SIZE)
        {
            LOG_ERROR("HW monitor command size is invalid");
            throw invalid_value_exception(to_string() << "Requested HW monitor command size " <<
                                                      std::dec << data.size() << " exceeds permitted limit " << HW_MONITOR_BUFFER_SIZE);
        }

        std::vector<uint8_t> transmit_buf(data);

        // Changed: 15.12.2021.
        // Instead of returning value directly check first if the value is valid.
        // This replaces "Incomplete bulk USB transfer!" error with an error appropriate for FRAMOS cameras.
        std::vector<byte> buffer = _device->send_hwm(transmit_buf);

        if (buffer.size() < static_cast<int>(sizeof(uint32_t)))
        {
            const uint8_t opcode_index = cs::d4_hwm_cmd_opcode_offset;
            const uint8_t unknown_opcode = 0x00;
            const uint8_t opcode = data.size() > opcode_index ? data[opcode_index] : unknown_opcode;

            std::stringstream stream;
            stream << "Hardware monitor command failed: 0x" << std::hex << (int)opcode;
            throw invalid_value_exception(stream.str());
        }

        return static_cast<std::vector<uint8_t>>(buffer);
    }


}
