// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2016 Intel Corporation. All Rights Reserved.

#include "cs-motion.h"
#include "cs-timestamp.h"
#include "ds5/ds5-private.h"
#include "ds5/ds5-timestamp.h"
#include "ds5/ds5-options.h"
//#include "proc/motion-transform.h"
#include "cs-motion-transform.h"
#include "cs-factory.h"
#include "proc/auto-exposure-processor.h"

namespace librealsense
{
    class LRS_EXTENSION_API cs_global_time_option_motion : public global_time_option
    {
    public:
        cs_global_time_option_motion(cs_motion* cs_motion) : m_cs_motion(cs_motion) {}

        void set(float value) override
        {
            if (!is_valid(value))
                throw invalid_value_exception(to_string() << "set(...) failed! " << value << " is not a valid value");
            _value = value;

            m_cs_motion->enable_time_diff_keeper(value != 0);
        };

    private:
        cs_motion* m_cs_motion;
    };

    std::map<uint32_t, rs2_format> cs_motion_fourcc_to_rs2_format = {
            {rs_fourcc('G','Y','R','O'), RS2_FORMAT_MOTION_XYZ32F},
            {rs_fourcc('A','C','C','L'), RS2_FORMAT_MOTION_XYZ32F}

    };

    std::map<uint32_t, rs2_stream> cs_motion_fourcc_to_rs2_stream = {
            {rs_fourcc('G','Y','R','O'), RS2_STREAM_GYRO},
            {rs_fourcc('A','C','C','L'), RS2_STREAM_ACCEL}
    };

    cs_mm_calib_handler::cs_mm_calib_handler(std::shared_ptr<platform::cs_device> cs_device, std::shared_ptr<hw_monitor> hw_monitor, ds::d400_caps dev_cap) :
        _device(cs_device),
        _hw_monitor(hw_monitor), 
        _dev_cap(dev_cap),
        _calib_tx_once_flag(false),
        _calib_tx_reg_addr(0),
        _calib_tx_reg_len(0),
        _calib_rx_once_flag(false),
        _calib_rx_reg_addr(0),
        _calib_rx_reg_len(0)
    {
        _connected_device = _device->get_cs_device();

        _imu_eeprom_raw = [this]() { return get_imu_eeprom_raw(); };

        _calib_parser = [this]() {

            std::vector<uint8_t> raw(ds::tm1_eeprom_size);
            uint16_t calib_id = ds::dm_v2_eeprom_id; //assume DM V2 IMU as default platform
            bool valid = false;

            try
            {
                raw = *_imu_eeprom_raw;
                calib_id = *reinterpret_cast<uint16_t*>(raw.data());
                valid = true;
            }
            catch (const std::exception&)
            {
                LOG_WARNING("CS IMU Calibration is not available, see the previous message");
            }

            // TODO NH - verify this when D455e will be available
            uint16_t pid = ds::RS435I_PID;
            if (_connected_device->GetModelName() == "D455e") {
                pid = ds::RS455_PID;
            }

            std::shared_ptr<mm_calib_parser> prs = nullptr;
            switch (calib_id)
            {
                case ds::dm_v2_eeprom_id: // DM V2 id
                    
                    // NH - this works in commit: SHA-1: 3ac1e6fb2cf4c163690b874573a67b198ab56dfb, in "fature/finalizing_merge" dm_v2_imu_calib_parser constructor is changed
                    //prs = std::make_shared<dm_v2_imu_calib_parser>(raw, _dev_cap, valid);
                    prs = std::make_shared<dm_v2_imu_calib_parser>(raw, pid, valid);

                    break;
                /*case ds::tm1_eeprom_id: // TM1 id
                    prs = std::make_shared<tm1_imu_calib_parser>(raw); 
                    break;*/
                default: {
                    /*throw recoverable_exception(to_string() << "CS - Motion Intrinsics unresolved - "
                    << ((valid) ? "device is not calibrated" : "invalid calib type "),
                    RS2_EXCEPTION_TYPE_BACKEND);*/

                    LOG_WARNING("CS - Motion Intrinsics unresolved");

                    std::vector<uint8_t> raw(ds::tm1_eeprom_size);
                    valid = false;
                    raw = *_imu_eeprom_raw;

                    // NH - this works in commit: SHA-1: 3ac1e6fb2cf4c163690b874573a67b198ab56dfb, in "fature/finalizing_merge" dm_v2_imu_calib_parser constructor is changed
                    //prs = std::make_shared<dm_v2_imu_calib_parser>(raw, _dev_cap, valid);
                    prs = std::make_shared<dm_v2_imu_calib_parser>(raw, pid, valid);
                }
            }
            return prs;
        };
    }

    std::vector<uint8_t> cs_mm_calib_handler::get_imu_eeprom_raw()
    {
        std::vector<uint8_t> buffer;
        if (!receive_calib_data(buffer)) {
            buffer.clear();
        }
        buffer.resize(ds::eeprom_imu_table_size);

        return buffer;
    }

    ds::imu_intrinsic cs_mm_calib_handler::get_intrinsic(rs2_stream stream)
    {
        return (*_calib_parser)->get_intrinsic(stream);
    }

    rs2_extrinsics cs_mm_calib_handler::get_extrinsic(rs2_stream stream)
    {
        return (*_calib_parser)->get_extrinsic_to(stream);
    }

    const std::vector<uint8_t> cs_mm_calib_handler::get_fisheye_calib_raw()
    {
        auto fe_calib_table = (*(ds::check_calib<ds::tm1_eeprom>(*_imu_eeprom_raw))).calibration_table.calib_model.fe_calibration;
        uint8_t* fe_calib_ptr = reinterpret_cast<uint8_t*>(&fe_calib_table);
        return std::vector<uint8_t>(fe_calib_ptr, fe_calib_ptr + ds::fisheye_calibration_table_size);
    }

    bool cs_mm_calib_handler::send_calib_data(const std::vector<uint8_t>& calib_data, bool verify)
    {
        uint64_t address, regLength;
        uint32_t restSize = calib_data.size();
        uint8_t* sendBuffer = (UINT8*)(&calib_data[0]);
        uint8_t readBuffer[GVCP_WRITEMEM_MAX_COUNT];
        uint8_t resendCount = 3;
        double maxWaitTime = 5.2;
        uint16_t size = 0;
        GEV_STATUS gevStatus;
        bool status = true;

        if (!_connected_device->IsConnected()) {
            status = false;
            return status;
        }

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

        return status;
    }

    bool cs_mm_calib_handler::receive_calib_data(std::vector<uint8_t>& buffer_out)
    {
        uint64_t address, regLength;
        uint32_t restSize;
        uint8_t readBuffer[GVCP_WRITEMEM_MAX_COUNT];
        uint8_t resendCount = 3;
        double maxWaitTime = 0.2;
        uint16_t size = 0;
        GEV_STATUS gevStatus;
        bool status = true;

        if (!_connected_device->IsConnected()) {
            status = false;
            return status;
        }

        if (!_calib_rx_once_flag) {
            // Verify HWm command RX buffer size
            status = _connected_device->GetNode("ImuCalibRxBuffer")->GetRegisterNodeLength(_calib_rx_reg_len);
            if (status) {
                // Get HWm command RX buffer address
                status = _connected_device->GetNode("ImuCalibRxBuffer")->GetRegisterNodeAddress(_calib_rx_reg_addr);
                if (status) {
                    _calib_rx_once_flag = true;
                }
            }
        }

        if ((_calib_rx_reg_len == 0) || (_calib_rx_reg_addr == 0)) {
            status = false;
            return status;
        }

        address = _calib_rx_reg_addr;
        restSize = _calib_rx_reg_len;
        buffer_out.reserve(_calib_rx_reg_len);

        // Execute HWm RX command
        status = _connected_device->CommandNodeExecute("ImuCalibRxBufferReceive");
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

        return status;
    }

    cs_motion::cs_motion(
        std::shared_ptr<context> ctx,
        const platform::backend_device_group& group)
        : device(ctx, group), cs_device_interface(ctx, group), cs_depth(ctx, group),
        _accel_stream(new stream(RS2_STREAM_ACCEL)),
        _gyro_stream(new stream(RS2_STREAM_GYRO))
    {
        using namespace ds;

        // TODO - nh - update D400e device fw to assert IMU present flag on GVD command
        if (((_device_capabilities & d400_caps::CAP_IMU_SENSOR) == d400_caps::CAP_UNDEFINED) ||
            (((_device_capabilities & d400_caps::CAP_BMI_055) == d400_caps::CAP_UNDEFINED) &&
            ((_device_capabilities & d400_caps::CAP_BMI_085) == d400_caps::CAP_UNDEFINED))) {
            return;
        }

        // TODO - nh - REMOVE
        std::vector<platform::hid_device_info> hid_devices_tmp;
        platform::hid_device_info hdi;

        hdi.id = gyro_sensor_name;
        hdi.vid = "8086";
        hdi.pid = "b3a";
        hdi.unique_id = CS_HID_GYRO_UNIQUE_ID;
        hdi.device_path = "\\\\?\\HID#VID_8086&PID_0B3A&MI_05#7&15c73191&0&0000#{c317c286-c468-4288-9975-d4c4587c442c}\\{CD0C768D-3A1A-42FF-B85F-2D99E500F872}";
        hdi.serial_number = _cs_device->get_serial();
        hid_devices_tmp.push_back(hdi);

        hdi.id = accel_sensor_name;
        hdi.vid = "8086";
        hdi.pid = "b3a";
        hdi.unique_id = CS_HID_ACCL_UNIQUE_ID;
        hdi.device_path = "\\\\?\\HID#VID_8086&PID_0B3A&MI_05#7&15c73191&0&0000#{c317c286-c468-4288-9975-d4c4587c442c}\\{CEE8DFF2-39B7-4517-A9CA-F0BDECE62396}";
        hdi.serial_number = _cs_device->get_serial();;
        hid_devices_tmp.push_back(hdi);
        // TODO - nh - REMOVE

        _mm_calib = std::make_shared<cs_mm_calib_handler>(_cs_device, _hw_monitor,_device_capabilities);

        _accel_intrinsic = std::make_shared<lazy<ds::imu_intrinsic>>([this]() { return _mm_calib->get_intrinsic(RS2_STREAM_ACCEL); });
        _gyro_intrinsic = std::make_shared<lazy<ds::imu_intrinsic>>([this]() { return _mm_calib->get_intrinsic(RS2_STREAM_GYRO); });
        // D435i to use predefined values extrinsics
        _depth_to_imu = std::make_shared<lazy<rs2_extrinsics>>([this]() { return _mm_calib->get_extrinsic(RS2_STREAM_ACCEL); });

        // Make sure all MM streams are positioned with the same extrinsics
        environment::get_instance().get_extrinsics_graph().register_extrinsics(*_depth_stream, *_accel_stream, _depth_to_imu);
        environment::get_instance().get_extrinsics_graph().register_same_extrinsics(*_accel_stream, *_gyro_stream);
        register_stream_to_extrinsic_group(*_gyro_stream, 0);
        register_stream_to_extrinsic_group(*_accel_stream, 0);

        // Try to add HID endpoint
        //auto hid_ep = create_hid_device(ctx, group.hid_devices, _fw_version);
        auto hid_ep = create_hid_device(ctx, hid_devices_tmp, _fw_version);
        if (hid_ep) {
            _motion_module_device_idx = static_cast<uint8_t>(add_sensor(hid_ep));

            // HID metadata attributes
            hid_ep->get_raw_sensor()->register_metadata(RS2_FRAME_METADATA_FRAME_TIMESTAMP, make_hid_header_parser(&platform::hid_header::timestamp));
        }
    }

    std::shared_ptr<synthetic_sensor> cs_motion::create_hid_device(
        std::shared_ptr<context> ctx,
        const std::vector<platform::hid_device_info>& all_hid_infos,
        const firmware_version& camera_fw_version)
    {
        if (all_hid_infos.empty()) {
            LOG_WARNING("No HID info provided, IMU is disabled");
            return nullptr;
        }

        static const char* custom_sensor_fw_ver = "5.6.0.0";

        //std::unique_ptr<frame_timestamp_reader> timestamp_reader_metadata(new iio_hid_timestamp_reader());
        std::unique_ptr<frame_timestamp_reader> timestamp_reader_metadata(new cs_hid_timestamp_reader());
        auto enable_global_time_option = std::shared_ptr<global_time_option>(new global_time_option());
        
        /*auto&& backend = ctx->get_backend();
        std::unique_ptr<frame_timestamp_reader> timestamp_reader_backup(new cs_timestamp_reader(backend.create_time_service()));
        std::unique_ptr<frame_timestamp_reader> timestamp_reader_metadata(new cs_timestamp_reader_from_metadata(std::move(timestamp_reader_backup)));
        auto enable_global_time_option = std::shared_ptr<global_time_option>(new cs_global_time_option_motion(this));*/

        // Dynamically populate the supported HID profiles according to the selected IMU module
        std::vector<odr> accel_fps_rates;
        std::map<unsigned, unsigned> fps_and_frequency_map;
        if (ds::d400_caps::CAP_BMI_085 && _device_capabilities) {
            accel_fps_rates = { odr::IMU_FPS_100,odr::IMU_FPS_200 };
        }
        else { // Applies to BMI_055 and unrecognized sensors
            accel_fps_rates = { odr::IMU_FPS_63,odr::IMU_FPS_250 };
        }

        for (auto&& elem : accel_fps_rates) {
            sensor_name_and_hid_profiles.push_back({ accel_sensor_name, { RS2_FORMAT_MOTION_XYZ32F, RS2_STREAM_ACCEL, 0, 1, 1, static_cast<uint16_t>(elem)} });
            fps_and_frequency_map.emplace(unsigned(elem), hid_fps_translation.at(elem));
        }
        fps_and_sampling_frequency_per_rs2_stream[RS2_STREAM_ACCEL] = fps_and_frequency_map;

        // TODO - nh - cs_hid_sensor
        auto raw_hid_ep = std::make_shared<cs_hid_sensor>(
            ctx->get_backend().create_hid_device(all_hid_infos.front()),
            _cs_device,
            std::unique_ptr<frame_timestamp_reader>(new global_timestamp_reader(std::move(timestamp_reader_metadata), _tf_keeper, enable_global_time_option)),
            fps_and_sampling_frequency_per_rs2_stream,
            sensor_name_and_hid_profiles,
            this);

        auto hid_ep = std::make_shared<cs_motion_sensor>("Motion Module", raw_hid_ep, this, this);
        // !TODO - nh - cs_hid_sensor

        // TODO - nh - cs_sensor (will integrate accl and gyro streams inside cs_sensor)
        /*auto&& backend = ctx->get_backend();
        std::unique_ptr<frame_timestamp_reader> timestamp_reader_backup(new cs_timestamp_reader(backend.create_time_service()));
        std::unique_ptr<frame_timestamp_reader> timestamp_reader_metadata(new cs_timestamp_reader_from_metadata(std::move(timestamp_reader_backup)));
        auto raw_hid_ep = std::make_shared<cs_sensor>(
            "Raw Motion Module", 
            _cs_device,
            std::unique_ptr<frame_timestamp_reader>(new global_timestamp_reader(std::move(timestamp_reader_metadata), 
            _tf_keeper, 
            enable_global_time_option)), 
            this, 
            CS_STREAM_IMU);

        auto hid_ep = std::make_shared<cs_motion_sensor>("Motion Module", raw_hid_ep, this, this, cs_motion_fourcc_to_rs2_format, cs_motion_fourcc_to_rs2_stream);*/
        // !TODO - nh - cs_sensor

        hid_ep->register_option(RS2_OPTION_GLOBAL_TIME_ENABLED, enable_global_time_option);

        // register pre-processing
        bool enable_imu_correction = false;
        std::shared_ptr<enable_motion_correction> mm_correct_opt = nullptr;

        //  Motion intrinsic calibration presents is a prerequisite for motion correction.
        try
        {
            // Writing to log to dereference underlying structure
            // (DEPRECATED -> to complex operation at the end, in some cases was causing bug #200 on bugzilla)
            //LOG_INFO("Accel Sensitivity:" << (**_accel_intrinsic).sensitivity);
            //LOG_INFO("Gyro Sensitivity:" << (**_gyro_intrinsic).sensitivity);
            // 
            // TODO - get log severity level and execute code bellow
            //float3x3 accel_sen = (**_accel_intrinsic).sensitivity;
            //float3x3 gyro_sen = (**_gyro_intrinsic).sensitivity;
            //LOG_INFO("Accel Sensitivity:" << accel_sen);
            //LOG_INFO("Gyro Sensitivity:" << gyro_sen);

            mm_correct_opt = std::make_shared<enable_motion_correction>(hid_ep.get(), option_range{ 0, 1, 1, 1 });
            hid_ep->register_option(RS2_OPTION_ENABLE_MOTION_CORRECTION, mm_correct_opt);
        }
        catch (...) {}

        hid_ep->register_processing_block(
            { {RS2_FORMAT_MOTION_XYZ32F, RS2_STREAM_ACCEL} },
            { {RS2_FORMAT_MOTION_XYZ32F, RS2_STREAM_ACCEL} },
            [&, mm_correct_opt]() { return std::make_shared<cs_acceleration_transform>(_mm_calib, mm_correct_opt);
        });

        hid_ep->register_processing_block(
            { {RS2_FORMAT_MOTION_XYZ32F, RS2_STREAM_GYRO} },
            { {RS2_FORMAT_MOTION_XYZ32F, RS2_STREAM_GYRO} },
            [&, mm_correct_opt]() { return std::make_shared<cs_gyroscope_transform>(_mm_calib, mm_correct_opt);
        });

        uint16_t pid = static_cast<uint16_t>(strtoul(all_hid_infos.front().pid.data(), nullptr, 16));

        // TODO - nh - IMU Temperature sensor
        /*if ((camera_fw_version >= firmware_version(custom_sensor_fw_ver)) && (!val_in_range(pid, { ds::RS400_IMU_PID, ds::RS435I_PID, ds::RS430I_PID, ds::RS465_PID }))) {
            hid_ep->register_option(RS2_OPTION_MOTION_MODULE_TEMPERATURE,
                                    std::make_shared<motion_module_temperature_option>(*raw_hid_ep));
        }*/

        return hid_ep;
    }

    rs2_motion_device_intrinsic cs_motion::get_motion_intrinsics(rs2_stream stream) const
    {
        if (stream == RS2_STREAM_ACCEL) {
            return create_motion_intrinsics(**_accel_intrinsic);
        }

        if (stream == RS2_STREAM_GYRO) {
            return create_motion_intrinsics(**_gyro_intrinsic);
        }

        throw std::runtime_error(to_string() << "Motion Intrinsics unknown for stream " << rs2_stream_to_string(stream) << "!");
    }

    cs_motion_sensor::cs_motion_sensor(
        std::string name,
        std::shared_ptr<sensor_base> sensor,
        device* device,
        cs_motion* owner)
        : synthetic_sensor(name, sensor, device),
        _owner(owner)
    {
    }

    /*cs_motion_sensor::cs_motion_sensor(
        std::string name,
        std::shared_ptr<sensor_base> sensor,
        device* device,
        cs_motion* owner,
        std::map<uint32_t, rs2_format> cs_motion_fourcc_to_rs2_format, 
        std::map<uint32_t, rs2_stream> cs_motion_fourcc_to_rs2_stream)
        : synthetic_sensor(name, sensor, device, cs_motion_fourcc_to_rs2_format, cs_motion_fourcc_to_rs2_stream),
        _owner(owner)
    {
    }*/

    rs2_motion_device_intrinsic cs_motion_sensor::get_motion_intrinsics(rs2_stream stream) const
    {
        return _owner->get_motion_intrinsics(stream);
    }

    stream_profiles cs_motion_sensor::init_stream_profiles()
    {
        auto lock = environment::get_instance().get_extrinsics_graph().lock();
        auto results = synthetic_sensor::init_stream_profiles();

        for (auto p : results) {

            // register stream types
            if (p->get_stream_type() == RS2_STREAM_ACCEL) {
                assign_stream(_owner->_accel_stream, p);
            }
            if (p->get_stream_type() == RS2_STREAM_GYRO) {
                assign_stream(_owner->_gyro_stream, p);
            }

            //set motion intrinsics
            if (p->get_stream_type() == RS2_STREAM_ACCEL || p->get_stream_type() == RS2_STREAM_GYRO) {
                auto motion = dynamic_cast<motion_stream_profile_interface*>(p.get());
                assert(motion);
                auto st = p->get_stream_type();
                motion->set_intrinsics([this, st]() { return get_motion_intrinsics(st); });
            }
        }

        return results;
    }
}
