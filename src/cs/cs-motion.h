// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015 Intel Corporation. All Rights Reserved.

#pragma once

#include "cs-device.h"
#include "sensor.h"
#include "ds5/ds5-motion.h"
#include <map>

namespace librealsense
{
    class cs_mm_calib_handler
    {
    public:
        cs_mm_calib_handler(std::shared_ptr<platform::cs_device> cs_device, std::shared_ptr<hw_monitor> hw_monitor, ds::d400_caps dev_cap);
        ~cs_mm_calib_handler() {}

        ds::imu_intrinsic get_intrinsic(rs2_stream);
        rs2_extrinsics get_extrinsic(rs2_stream);       // The extrinsic defined as Depth->Stream rigid-body transfom.
        const std::vector<uint8_t> get_fisheye_calib_raw();
        float3x3 imu_to_depth_alignment() { return (*_calib_parser)->imu_to_depth_alignment(); }

    private:
        bool send_calib_data(const std::vector<uint8_t>& calib_data, bool verify = false);
        bool receive_calib_data(std::vector<uint8_t>& buffer_out);
        std::vector<uint8_t> get_imu_eeprom_raw();

    private:
        std::shared_ptr<hw_monitor> _hw_monitor;
        ds::d400_caps _dev_cap;
        lazy< std::shared_ptr<mm_calib_parser>> _calib_parser;
        lazy<std::vector<uint8_t>> _imu_eeprom_raw;
        lazy<std::vector<uint8_t>> _fisheye_calibration_table_raw;
        std::shared_ptr<platform::cs_device> _device;
        smcs::IDevice _connected_device;

        //std::once_flag _calib_tx_once_flag;
        bool _calib_tx_once_flag;
        UINT64 _calib_tx_reg_addr;
        UINT64 _calib_tx_reg_len;
        //std::once_flag _calib_rx_once_flag;
        bool _calib_rx_once_flag;
        UINT64 _calib_rx_reg_addr;
        UINT64 _calib_rx_reg_len;
    };



    class cs_motion : public virtual cs_depth
    {
    public:
        cs_motion(std::shared_ptr<context> ctx, const platform::backend_device_group& group);

        std::shared_ptr<synthetic_sensor> create_hid_device(std::shared_ptr<context> ctx, const std::vector<platform::hid_device_info>& all_hid_infos, const firmware_version& camera_fw_version);
        rs2_motion_device_intrinsic get_motion_intrinsics(rs2_stream) const;

    private:

        friend class cs_motion_sensor;

        optional_value<uint8_t> _fisheye_device_idx;

        std::shared_ptr<cs_mm_calib_handler>        _mm_calib;
        std::shared_ptr<lazy<ds::imu_intrinsic>> _accel_intrinsic;
        std::shared_ptr<lazy<ds::imu_intrinsic>> _gyro_intrinsic;
        lazy<std::vector<uint8_t>>              _fisheye_calibration_table_raw;
        std::shared_ptr<lazy<rs2_extrinsics>>   _depth_to_imu;                  // Mechanical installation pose
        uint16_t _pid;    // product PID

        // Bandwidth parameters required for HID sensors
        // The Acceleration configuration will be resolved according to the IMU sensor type at run-time
        std::vector<std::pair<std::string, stream_profile>> sensor_name_and_hid_profiles =
        { { gyro_sensor_name,     {RS2_FORMAT_MOTION_XYZ32F, RS2_STREAM_GYRO, 0, 1, 1, int(odr::IMU_FPS_200)}},
          { gyro_sensor_name,     {RS2_FORMAT_MOTION_XYZ32F, RS2_STREAM_GYRO, 0, 1, 1, int(odr::IMU_FPS_400)}}};

        // Translate frequency to SENSOR_PROPERTY_CURRENT_REPORT_INTERVAL.
        std::map<rs2_stream, std::map<unsigned, unsigned>> fps_and_sampling_frequency_per_rs2_stream =
        { { RS2_STREAM_GYRO,     {{unsigned(odr::IMU_FPS_200),  hid_fps_translation.at(odr::IMU_FPS_200)},
                                 { unsigned(odr::IMU_FPS_400),  hid_fps_translation.at(odr::IMU_FPS_400)}}} };

    protected:
        std::shared_ptr<stream_interface> _accel_stream;
        std::shared_ptr<stream_interface> _gyro_stream;

        optional_value<uint8_t> _motion_module_device_idx;

        //ds::d400_caps _device_capabilities;
    };



    class cs_motion_sensor : public synthetic_sensor, public motion_sensor
    {
    public:
        explicit cs_motion_sensor(std::string name, std::shared_ptr<sensor_base> sensor, device* device, cs_motion* owner);
        //explicit cs_motion_sensor(std::string name, std::shared_ptr<sensor_base> sensor, device* device, cs_motion* owner, std::map<uint32_t, rs2_format> cs_motion_fourcc_to_rs2_format, std::map<uint32_t, rs2_stream> cs_motion_fourcc_to_rs2_stream);

        rs2_motion_device_intrinsic get_motion_intrinsics(rs2_stream stream) const;

        stream_profiles init_stream_profiles() override;

    private:
        const cs_motion* _owner;
    };



}
