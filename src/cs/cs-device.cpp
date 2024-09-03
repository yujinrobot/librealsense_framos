// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 FRAMOS GmbH.

#include "cs-device.h"
#include "cs-timestamp.h"
#include "cs-options.h"

#include "proc/decimation-filter.h"
#include "proc/threshold.h"
#include "proc/disparity-transform.h"
#include "proc/spatial-filter.h"
#include "proc/colorizer.h"
#include "proc/temporal-filter.h"
#include "proc/y8i-to-y8y8.h"
#include "proc/y12i-to-y16y16.h"
#include "proc/color-formats-converter.h"
#include "proc/syncer-processing-block.h"
#include "proc/hole-filling-filter.h"
#include "proc/depth-formats-converter.h"
#include "proc/depth-decompress.h"
#include "proc/hdr-merge.h"
#include "proc/sequence-id-filter.h"
#include "../common/fw/firmware-version.h"
#include "../third-party/json.hpp"

namespace librealsense
{
    class LRS_EXTENSION_API cs_global_time_option_depth : public global_time_option
    {
    public:
        cs_global_time_option_depth(cs_depth* cs_depth) : m_cs_depth(cs_depth) {}

        void set(float value) override
        {
            if (!is_valid(value))
                throw invalid_value_exception(to_string() << "set(...) failed! " << value << " is not a valid value");
            _value = value;

            m_cs_depth->enable_time_diff_keeper(value != 0);
        };

    private:
        cs_depth* m_cs_depth;
    };


    class LRS_EXTENSION_API cs_global_time_option_color : public global_time_option
    {
    public:
        cs_global_time_option_color(cs_color* cs_color) : m_cs_color(cs_color) {}

        void set(float value) override
        {
            if (!is_valid(value))
                throw invalid_value_exception(to_string() << "set(...) failed! " << value << " is not a valid value");
            _value = value;

            m_cs_color->enable_time_diff_keeper(value != 0);
        };

    private:
        cs_color* m_cs_color;
    };

    std::map<uint32_t, rs2_format> cs_depth_fourcc_to_rs2_format = {
            {rs_fourcc('Y','U','Y','2'), RS2_FORMAT_YUYV},
            {rs_fourcc('Y','U','Y','V'), RS2_FORMAT_YUYV},
            {rs_fourcc('U','Y','V','Y'), RS2_FORMAT_UYVY},
            {rs_fourcc('G','R','E','Y'), RS2_FORMAT_Y8},
            {rs_fourcc('Y','8','I',' '), RS2_FORMAT_Y8I},
            {rs_fourcc('W','1','0',' '), RS2_FORMAT_W10},
            {rs_fourcc('Y','1','6',' '), RS2_FORMAT_Y16},
            {rs_fourcc('Y','1','2','I'), RS2_FORMAT_Y12I},
            {rs_fourcc('Z','1','6',' '), RS2_FORMAT_Z16},
            {rs_fourcc('Z','1','6','H'), RS2_FORMAT_Z16H},
            {rs_fourcc('R','G','B','2'), RS2_FORMAT_BGR8}

    };
    std::map<uint32_t, rs2_stream> cs_depth_fourcc_to_rs2_stream = {
            {rs_fourcc('Y','U','Y','2'), RS2_STREAM_INFRARED},
            {rs_fourcc('Y','U','Y','V'), RS2_STREAM_INFRARED},
            {rs_fourcc('U','Y','V','Y'), RS2_STREAM_INFRARED},
            {rs_fourcc('G','R','E','Y'), RS2_STREAM_INFRARED},
            {rs_fourcc('Y','8','I',' '), RS2_STREAM_INFRARED},
            {rs_fourcc('W','1','0',' '), RS2_STREAM_INFRARED},
            {rs_fourcc('Y','1','6',' '), RS2_STREAM_INFRARED},
            {rs_fourcc('Y','1','2','I'), RS2_STREAM_INFRARED},
            {rs_fourcc('R','G','B','2'), RS2_STREAM_INFRARED},
            {rs_fourcc('Z','1','6',' '), RS2_STREAM_DEPTH},
            {rs_fourcc('Z','1','6','H'), RS2_STREAM_DEPTH}
    };

    std::map<uint32_t, rs2_format> cs_color_fourcc_to_rs2_format = {
            {rs_fourcc('Y','U','Y','2'), RS2_FORMAT_YUYV},
            {rs_fourcc('Y','U','Y','V'), RS2_FORMAT_YUYV},
            {rs_fourcc('U','Y','V','Y'), RS2_FORMAT_UYVY},
            {rs_fourcc('M','J','P','G'), RS2_FORMAT_MJPEG},
            {rs_fourcc('B','Y','R','2'), RS2_FORMAT_RAW16}
    };
    std::map<uint32_t, rs2_stream> cs_color_fourcc_to_rs2_stream = {
            {rs_fourcc('Y','U','Y','2'), RS2_STREAM_COLOR},
            {rs_fourcc('Y','U','Y','V'), RS2_STREAM_COLOR},
            {rs_fourcc('U','Y','V','Y'), RS2_STREAM_COLOR},
            {rs_fourcc('B','Y','R','2'), RS2_STREAM_COLOR},
            {rs_fourcc('M','J','P','G'), RS2_STREAM_COLOR},
    };

    cs_auto_exposure_roi_method::cs_auto_exposure_roi_method(const hw_monitor& hwm,
                                                             ds::fw_cmd cmd)
            : _hw_monitor(hwm), _cmd(cmd) {}

    void cs_auto_exposure_roi_method::set(const region_of_interest& roi)
    {
        command cmd(_cmd);
        cmd.param1 = roi.min_y;
        cmd.param2 = roi.max_y;
        cmd.param3 = roi.min_x;
        cmd.param4 = roi.max_x;
        _hw_monitor.send(cmd);
    }

    region_of_interest cs_auto_exposure_roi_method::get() const
    {
        region_of_interest roi;
        command cmd(_cmd + 1);
        auto res = _hw_monitor.send(cmd);

        if (res.size() < 4 * sizeof(uint16_t))
        {
            throw std::runtime_error("Invalid result size!");
        }

        auto words = reinterpret_cast<uint16_t*>(res.data());

        roi.min_y = words[0];
        roi.max_y = words[1];
        roi.min_x = words[2];
        roi.max_x = words[3];

        return roi;
    }

    cs_external_sync_mode::cs_external_sync_mode(hw_monitor& hwm, cs_sensor& depth)
        : _hwm(hwm), _depth(depth)
    {
        _range = [this]()
        {
            return _depth.supports_option(RS2_OPTION_EXTERNAL_EVENT_BURST_COUNT) ? option_range{ cs_inter_cam_mode::CS_INTERCAM_SYNC_DEFAULT,
                cs_inter_cam_mode::CS_INTERCAM_SYNC_MAX - 1,
                1,  cs_inter_cam_mode::CS_INTERCAM_SYNC_DEFAULT } :
                option_range{ cs_inter_cam_mode::CS_INTERCAM_SYNC_DEFAULT,
                cs_inter_cam_mode::CS_INTERCAM_SYNC_MAX - 2,
                1, cs_inter_cam_mode::CS_INTERCAM_SYNC_DEFAULT }
                ;
        };
    }

    void cs_external_sync_mode::set(float value)
    {
        _depth.set_inter_cam_sync_mode(value, false);
        _record_action(*this);
    }

    float cs_external_sync_mode::query() const
    {
        return _depth.get_inter_cam_sync_mode(false);
    }

    option_range cs_external_sync_mode::get_range() const
    {
        return *_range;
    }

    cs_external_sync_mode_gs::cs_external_sync_mode_gs(hw_monitor& hwm, cs_sensor& depth)
        : _hwm(hwm), _depth(depth)
    {
        _range = [this]()
        {
            return option_range{ cs_inter_cam_sync_mode_gs::CS_INTERCAM_SYNC_DEFAULT_GS,
                cs_inter_cam_sync_mode_gs::CS_INTERCAM_SYNC_MAX_GS-1,
                1,
                cs_inter_cam_sync_mode_gs::CS_INTERCAM_SYNC_DEFAULT_GS };
        };
    }

    void cs_external_sync_mode_gs::set(float value)
    {
        if (_depth.is_streaming())
            throw std::runtime_error("Cannot change Inter-camera HW synchronization mode while streaming!");

        _depth.set_inter_cam_sync_mode(value, true);
        _record_action(*this);
    }

    float cs_external_sync_mode_gs::query() const
    {
        return _depth.get_inter_cam_sync_mode(true);
    }

    option_range cs_external_sync_mode_gs::get_range() const
    {
        return *_range;
    }

    cs_external_sync_mode_color::cs_external_sync_mode_color(cs_sensor& color)
        : _color(color)
    {
        _range = [this]()
        {
            return _color.supports_option(RS2_OPTION_EXTERNAL_EVENT_BURST_COUNT) ? option_range{ cs_inter_cam_mode_color::CS_INTERCAM_SYNC_DEFAULT_COLOR,
                cs_inter_cam_mode_color::CS_INTERCAM_SYNC_MAX_COLOR - 1,
                1, cs_inter_cam_mode_color::CS_INTERCAM_SYNC_DEFAULT_COLOR } :
                option_range{ cs_inter_cam_mode_color::CS_INTERCAM_SYNC_DEFAULT_COLOR,
                cs_inter_cam_mode_color::CS_INTERCAM_SYNC_MAX_COLOR - 2,
                1, cs_inter_cam_mode_color::CS_INTERCAM_SYNC_DEFAULT_COLOR };
        };
    }

    void cs_external_sync_mode_color::set(float value)
    {
        _color.set_inter_cam_sync_mode(value, false);
        _record_action(*this);
    }

    float cs_external_sync_mode_color::query() const
    {
        return _color.get_inter_cam_sync_mode(false);
    }

    option_range cs_external_sync_mode_color::get_range() const
    {
        return *_range;
    }

    cs_external_sync_mode_color_gs::cs_external_sync_mode_color_gs(cs_sensor& color)
        : _color(color)
    {
        _range = [this]()
        {
            return option_range{ cs_inter_cam_mode_color_gs::CS_INTERCAM_SYNC_DEFAULT_COLOR_GS,
                                 cs_inter_cam_mode_color_gs::CS_INTERCAM_SYNC_MAX_COLOR_GS - 1,
                                 1, cs_inter_cam_mode_color_gs::CS_INTERCAM_SYNC_DEFAULT_COLOR_GS };
        };
    }

    void cs_external_sync_mode_color_gs::set(float value)
    {
        _color.set_inter_cam_sync_mode(value, true);
        _record_action(*this);
    }

    float cs_external_sync_mode_color_gs::query() const
    {
        return _color.get_inter_cam_sync_mode(true);
    }

    option_range cs_external_sync_mode_color_gs::get_range() const
    {
        return *_range;
    }

    cs_device_interface::cs_device_interface(std::shared_ptr<context> ctx,
                                             const platform::backend_device_group& group)
            : global_time_interface()
    {
        _cs_device_info = group.cs_devices.front();
        _cs_device = ctx->get_backend().create_cs_device(_cs_device_info);

        _hw_monitor = std::make_shared<hw_monitor>(std::make_shared<cs_command_transfer>(_cs_device));
    }

    double cs_device_interface::get_device_time_ms()
    {
        if (_cs_device) {
            return _cs_device->get_device_timestamp_ms();
        }
        else {
            throw std::runtime_error("cs_device not initialized");
        }
    }

    cs_color::cs_color(std::shared_ptr<context> ctx,
                       const platform::backend_device_group& group)
            : device(ctx, group),
              cs_device_interface(ctx, group),
              _color_stream(new stream(RS2_STREAM_COLOR))
    {
        _color_device_idx = add_sensor(create_color_device(ctx, _cs_device));
        color_init(ctx, group);
    }

    cs_depth::cs_depth(std::shared_ptr<context> ctx,
                       const platform::backend_device_group& group)
            : device(ctx, group),
              cs_device_interface(ctx, group),
              _depth_stream(new stream(RS2_STREAM_DEPTH)),
              _left_ir_stream(new stream(RS2_STREAM_INFRARED, 1)),
              _right_ir_stream(new stream(RS2_STREAM_INFRARED, 2)),
              _device_capabilities(ds::d400_caps::CAP_UNDEFINED)
    {
        _depth_device_idx = add_sensor(create_depth_device(ctx, _cs_device));
        depth_init(ctx, group);
    }

    void cs_color::color_init(std::shared_ptr<context> ctx, const platform::backend_device_group& group)
    {
        using namespace ds;

        _color_calib_table_raw = [this]() { return get_raw_calibration_table(rgb_calibration_id); };
        _color_extrinsic = std::make_shared<lazy<rs2_extrinsics>>([this]() { return from_pose(get_color_stream_extrinsic(*_color_calib_table_raw)); });

        register_stream_to_extrinsic_group(*_color_stream, 0);

        auto& color_ep = get_color_sensor();
        auto& raw_color_sensor = get_raw_color_sensor();

        std::vector<uint8_t> gvd_buff(HW_MONITOR_BUFFER_SIZE);
        _hw_monitor->get_gvd(gvd_buff.size(), gvd_buff.data(), GVD);
        // fooling tests recordings - don't remove
        _hw_monitor->get_gvd(gvd_buff.size(), gvd_buff.data(), GVD);

        auto fwv = _hw_monitor->get_firmware_version_string(gvd_buff, camera_fw_version_offset);
        _fw_version = firmware_version(fwv);

        color_ep.register_option(RS2_OPTION_BRIGHTNESS, std::make_shared<cs_pu_option>(raw_color_sensor, RS2_OPTION_BRIGHTNESS, CS_STREAM_COLOR));
        color_ep.register_option(RS2_OPTION_CONTRAST, std::make_shared<cs_pu_option>(raw_color_sensor, RS2_OPTION_CONTRAST, CS_STREAM_COLOR));
        color_ep.register_option(RS2_OPTION_GAIN, std::make_shared<cs_pu_option>(raw_color_sensor, RS2_OPTION_GAIN, CS_STREAM_COLOR));
        color_ep.register_option(RS2_OPTION_HUE, std::make_shared<cs_pu_option>(raw_color_sensor, RS2_OPTION_HUE, CS_STREAM_COLOR));
        color_ep.register_option(RS2_OPTION_SATURATION, std::make_shared<cs_pu_option>(raw_color_sensor, RS2_OPTION_SATURATION, CS_STREAM_COLOR));
        color_ep.register_option(RS2_OPTION_SHARPNESS, std::make_shared<cs_pu_option>(raw_color_sensor, RS2_OPTION_SHARPNESS, CS_STREAM_COLOR));
        color_ep.register_option(RS2_OPTION_GAMMA, std::make_shared<cs_pu_option>(raw_color_sensor, RS2_OPTION_GAMMA, CS_STREAM_COLOR));
        color_ep.register_option(RS2_OPTION_BACKLIGHT_COMPENSATION, std::make_shared<cs_pu_option>(raw_color_sensor, RS2_OPTION_BACKLIGHT_COMPENSATION, CS_STREAM_COLOR));
        color_ep.register_option(RS2_OPTION_AUTO_EXPOSURE_PRIORITY, std::make_shared<cs_pu_option>(raw_color_sensor, RS2_OPTION_AUTO_EXPOSURE_PRIORITY, CS_STREAM_COLOR));

        if (_cs_device->is_option_supported(RS2_OPTION_RGB_LED_TOGGLE, CS_STREAM_COLOR))
            color_ep.register_option(RS2_OPTION_RGB_LED_TOGGLE, std::make_shared<cs_pu_option>(raw_color_sensor, RS2_OPTION_RGB_LED_TOGGLE, CS_STREAM_COLOR));

        auto white_balance_option = std::make_shared<cs_pu_option>(raw_color_sensor, RS2_OPTION_WHITE_BALANCE, CS_STREAM_COLOR);
        auto auto_white_balance_option = std::make_shared<cs_pu_option>(raw_color_sensor, RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE, CS_STREAM_COLOR);
        color_ep.register_option(RS2_OPTION_WHITE_BALANCE, white_balance_option);
        color_ep.register_option(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE, auto_white_balance_option);
        color_ep.register_option(RS2_OPTION_WHITE_BALANCE,
                                 std::make_shared<auto_disabling_control>(
                                         white_balance_option,
                                         auto_white_balance_option));

        auto exposure_option = std::make_shared<cs_pu_option>(raw_color_sensor, RS2_OPTION_EXPOSURE, CS_STREAM_COLOR);
        auto auto_exposure_option = std::make_shared<cs_pu_option>(raw_color_sensor, RS2_OPTION_ENABLE_AUTO_EXPOSURE, CS_STREAM_COLOR);
        color_ep.register_option(RS2_OPTION_EXPOSURE, exposure_option);
        color_ep.register_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, auto_exposure_option);
        color_ep.register_option(RS2_OPTION_EXPOSURE,
                                 std::make_shared<auto_disabling_control>(
                                         exposure_option,
                                         auto_exposure_option));

        color_ep.register_option(RS2_OPTION_POWER_LINE_FREQUENCY,
                                 std::make_shared<cs_pu_option>(raw_color_sensor, RS2_OPTION_POWER_LINE_FREQUENCY, CS_STREAM_COLOR,
                                                                std::map<float, std::string>{ { 0.f, "Disabled"},
                                                                                              { 1.f, "50Hz" },
                                                                                              { 2.f, "60Hz" },
                                                                                              { 3.f, "Auto" },
                                                                                              { 4.f, "OutDoor" },}));
        // Starting with firmware 5.10.9, auto-exposure ROI is available for color sensor
        if (_fw_version >= firmware_version("5.10.9.0"))
        {
            roi_sensor_interface* roi_sensor;
            if (roi_sensor = dynamic_cast<roi_sensor_interface*>(&color_ep))
                roi_sensor->set_roi_method(std::make_shared<cs_auto_exposure_roi_method>(*_hw_monitor, ds::fw_cmd::SETRGBAEROI));
        }

        if (_cs_device->is_burst_mode_supported()) {
            auto external_burst = std::make_shared<cs_pu_option>(raw_color_sensor, RS2_OPTION_EXTERNAL_EVENT_BURST_COUNT, CS_STREAM_COLOR);
            color_ep.register_option(RS2_OPTION_EXTERNAL_EVENT_BURST_COUNT, external_burst);
        }

        if (_fw_version >= firmware_version("5.9.15.1"))
        {
            if (_cs_device->is_color_genlock_mode_supported()) {
                auto ext_sync_mode = std::make_shared<cs_external_sync_mode_color_gs>(raw_color_sensor);
                color_ep.register_option(RS2_OPTION_INTER_CAM_SYNC_MODE, ext_sync_mode);
            }
            else {
                auto ext_sync_mode = std::make_shared<cs_external_sync_mode_color>(raw_color_sensor);
                color_ep.register_option(RS2_OPTION_INTER_CAM_SYNC_MODE, ext_sync_mode);
            }
        }

        auto inter_packet_delay_option = std::make_shared<cs_pu_option>(raw_color_sensor, RS2_OPTION_INTER_PACKET_DELAY, CS_STREAM_COLOR);
        color_ep.register_option(RS2_OPTION_INTER_PACKET_DELAY, inter_packet_delay_option);

        auto packet_size_option = std::make_shared<cs_pu_option>(raw_color_sensor, RS2_OPTION_PACKET_SIZE, CS_STREAM_COLOR);
        color_ep.register_option(RS2_OPTION_PACKET_SIZE, packet_size_option);

        if (_cs_device->is_software_trigger_supported()) {
            color_ep.register_option(RS2_OPTION_SOFTWARE_TRIGGER,
                std::make_shared<cs_software_trigger_option>(raw_color_sensor, RS2_OPTION_SOFTWARE_TRIGGER, CS_STREAM_COLOR,
                    std::map<float, std::string>{ { 1.f, "Trigger" }}));

            color_ep.register_option(RS2_OPTION_EXT_TRIGGER_SOURCE,
                std::make_shared<cs_external_trigger_option>(raw_color_sensor, RS2_OPTION_EXT_TRIGGER_SOURCE, CS_STREAM_COLOR,
                    std::map<float, std::string>{ { 1.f, "Hardware" },
                    { 2.f, "Software" }}));
        }
        enable_time_diff_keeper(true);
    }

    void cs_depth::depth_init(std::shared_ptr<context> ctx, const platform::backend_device_group& group)
    {
        using namespace ds;

        auto&& backend = ctx->get_backend();
        auto& raw_sensor = get_raw_depth_sensor();

        _color_calib_table_raw = [this]()
        {
            return get_raw_calibration_table(rgb_calibration_id);
        };

        _depth_extrinsic = std::make_shared<lazy<rs2_extrinsics>>([this]()
                {
                    rs2_extrinsics ext = identity_matrix();
                    auto table = check_calib<coefficients_table>(*_depth_calib_table_raw);
                    ext.translation[0] = 0.001f * table->baseline; // mm to meters
                    return ext;
                });

        environment::get_instance().get_extrinsics_graph().register_same_extrinsics(*_depth_stream, *_left_ir_stream);
        environment::get_instance().get_extrinsics_graph().register_extrinsics(*_depth_stream, *_right_ir_stream, _depth_extrinsic);

        register_stream_to_extrinsic_group(*_depth_stream, 0);
        register_stream_to_extrinsic_group(*_left_ir_stream, 0);
        register_stream_to_extrinsic_group(*_right_ir_stream, 0);
        
        _depth_calib_table_raw = [this]() { return get_raw_calibration_table(coefficients_table_id); };
        _new_calib_table_raw = [this]() { return get_new_calibration_table(); };

        auto pid = group.cs_devices.front().vid;
        _pid = group.cs_devices.front().vid;

        std::vector<uint8_t> gvd_buff(HW_MONITOR_BUFFER_SIZE);

        auto& depth_sensor = get_depth_sensor();
        auto& raw_depth_sensor = get_raw_depth_sensor();

        using namespace platform;

        std::string optic_serial, asic_serial;
        bool advanced_mode;
        _hw_monitor->get_gvd(gvd_buff.size(), gvd_buff.data(), GVD);
        // fooling tests recordings - don't remove
        _hw_monitor->get_gvd(gvd_buff.size(), gvd_buff.data(), GVD);

        optic_serial = _hw_monitor->get_module_serial_string(gvd_buff, module_serial_offset);
        asic_serial = _hw_monitor->get_module_serial_string(gvd_buff, module_asic_serial_offset);
        auto fwv = _hw_monitor->get_firmware_version_string(gvd_buff, camera_fw_version_offset);
        _fw_version = firmware_version(fwv);

        _recommended_fw_version = firmware_version(D4XX_RECOMMENDED_FIRMWARE_VERSION);
        if (_fw_version >= firmware_version("5.10.4.0"))
            _device_capabilities = parse_device_capabilities(pid);

        if (_cs_device->is_imu_supported()) {
            _device_capabilities |= d400_caps::CAP_IMU_SENSOR;
            auto imu_id = _cs_device->get_imu_id();
            if (imu_id == BMI055_IDENTIFIER) {
                _device_capabilities |= d400_caps::CAP_BMI_055;
            }
            else if (imu_id == BMI085_IDENTIFIER) {
                _device_capabilities |= d400_caps::CAP_BMI_085;
            }
        }



        advanced_mode = is_camera_in_advanced_mode();

        if (_fw_version >= firmware_version("5.12.1.1"))
        {
            depth_sensor.register_processing_block(processing_block_factory::create_id_pbf(RS2_FORMAT_Z16H, RS2_STREAM_DEPTH));
        }

        if (advanced_mode)
        {
            depth_sensor.register_processing_block(
                    { {RS2_FORMAT_Y8I} },
                    { {RS2_FORMAT_Y8, RS2_STREAM_INFRARED, 1} , {RS2_FORMAT_Y8, RS2_STREAM_INFRARED, 2} },
                    []() { return std::make_shared<y8i_to_y8y8>(); }
            ); // L+R

            depth_sensor.register_processing_block(
                    {RS2_FORMAT_Y12I},
                    {{RS2_FORMAT_Y16, RS2_STREAM_INFRARED, 1}, {RS2_FORMAT_Y16, RS2_STREAM_INFRARED, 2}},
                    []() {return std::make_shared<y12i_to_y16y16>(); }
            );
        }

        if (_fw_version >= firmware_version("5.6.3.0"))
        {
            auto gain_option = std::make_shared<cs_pu_option>(raw_depth_sensor, RS2_OPTION_GAIN, CS_STREAM_DEPTH);
            depth_sensor.register_option(RS2_OPTION_GAIN, gain_option);

            auto exposure_option = std::make_shared<cs_depth_exposure_option>(raw_depth_sensor, RS2_OPTION_EXPOSURE, CS_STREAM_DEPTH);
            
            // TODO NH - redundant, remove this, will be re-registerd few lines bellow
            depth_sensor.register_option(RS2_OPTION_EXPOSURE, exposure_option);

            auto auto_exposure_option = std::make_shared<cs_pu_option>(raw_depth_sensor, RS2_OPTION_ENABLE_AUTO_EXPOSURE, CS_STREAM_DEPTH);
            depth_sensor.register_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, auto_exposure_option);

            depth_sensor.register_option(RS2_OPTION_EXPOSURE, std::make_shared<auto_disabling_control>( exposure_option, auto_exposure_option));
        }

        auto emitter_enabled_option = std::make_shared<cs_pu_option>(raw_depth_sensor, RS2_OPTION_EMITTER_ENABLED, CS_STREAM_DEPTH);
        depth_sensor.register_option(RS2_OPTION_EMITTER_ENABLED, emitter_enabled_option);

        auto laser_power_option = std::make_shared<cs_pu_option>(raw_depth_sensor, RS2_OPTION_LASER_POWER, CS_STREAM_DEPTH);
        depth_sensor.register_option(RS2_OPTION_LASER_POWER, laser_power_option);

        depth_sensor.register_option(RS2_OPTION_LASER_POWER,
                                     std::make_shared<auto_disabling_control>(
                                             laser_power_option,
                                             emitter_enabled_option,
                                             std::vector<float>{0.f}, 1.f));

        if (_fw_version >= firmware_version("5.5.8.0"))
        {
            if (_cs_device->is_option_supported(RS2_OPTION_ASIC_TEMPERATURE, CS_STREAM_DEPTH))
                depth_sensor.register_option(RS2_OPTION_ASIC_TEMPERATURE,
                                         std::make_shared<cs_asic_and_projector_temperature_options>(raw_depth_sensor,
                                                                                                    RS2_OPTION_ASIC_TEMPERATURE,
                                                                                                    CS_STREAM_DEPTH));
            
            if (_cs_device->is_option_supported(RS2_OPTION_PROJECTOR_TEMPERATURE, CS_STREAM_DEPTH))
                depth_sensor.register_option(RS2_OPTION_PROJECTOR_TEMPERATURE,
                                         std::make_shared<cs_asic_and_projector_temperature_options>(raw_depth_sensor,
                                                                                                     RS2_OPTION_PROJECTOR_TEMPERATURE,
                                                                                                     CS_STREAM_DEPTH));
        }

        if (_cs_device->is_burst_mode_supported()) {
            auto external_burst = std::make_shared<cs_pu_option>(raw_depth_sensor, RS2_OPTION_EXTERNAL_EVENT_BURST_COUNT, CS_STREAM_DEPTH);
            depth_sensor.register_option(RS2_OPTION_EXTERNAL_EVENT_BURST_COUNT, external_burst);
        }

        if (_fw_version >= firmware_version("5.12.4.0") && (_device_capabilities & d400_caps::CAP_GLOBAL_SHUTTER) == d400_caps::CAP_GLOBAL_SHUTTER)
        {
            auto ext_sync_mode = std::make_shared<cs_external_sync_mode_gs>(*_hw_monitor, raw_depth_sensor);
            depth_sensor.register_option(RS2_OPTION_INTER_CAM_SYNC_MODE, ext_sync_mode);
        }
        else if (_fw_version >= firmware_version("5.9.15.1"))
        {
            auto ext_sync_mode = std::make_shared<cs_external_sync_mode>(*_hw_monitor, raw_depth_sensor);
            depth_sensor.register_option(RS2_OPTION_INTER_CAM_SYNC_MODE, ext_sync_mode);
        }

        roi_sensor_interface* roi_sensor = dynamic_cast<roi_sensor_interface*>(&depth_sensor);
        if (roi_sensor)
            roi_sensor->set_roi_method(std::make_shared<cs_auto_exposure_roi_method>(*_hw_monitor));

        depth_sensor.register_option(RS2_OPTION_STEREO_BASELINE, std::make_shared<const_value_option>("Distance in mm between the stereo imagers",
                                                                                                     lazy<float>([this]() { return get_stereo_baseline_mm(); })));

        if (advanced_mode && _fw_version >= firmware_version("5.6.3.0"))
        {
            auto depth_scale = std::make_shared<cs_depth_scale_option>(*_hw_monitor);
            auto depth_sensor = As<cs_depth_sensor, synthetic_sensor>(&get_depth_sensor());
            assert(depth_sensor);

            depth_scale->add_observer([depth_sensor](float val)
                                      {
                                          depth_sensor->set_depth_scale(val);
                                      });

            depth_sensor->register_option(RS2_OPTION_DEPTH_UNITS, depth_scale);
        }
        else
        {
            float default_depth_units = 0.001f; //meters
            // default depth units is different for D405
            if (pid == RS405_PID)
                default_depth_units = 0.0001f;  //meters
            depth_sensor.register_option(RS2_OPTION_DEPTH_UNITS, std::make_shared<const_value_option>("Number of meters represented by a single depth unit",
                lazy<float>([default_depth_units]()
                    { return default_depth_units; })));
        }

        auto inter_packet_delay_option = std::make_shared<cs_pu_option>(raw_depth_sensor, RS2_OPTION_INTER_PACKET_DELAY, CS_STREAM_DEPTH);
        depth_sensor.register_option(RS2_OPTION_INTER_PACKET_DELAY, inter_packet_delay_option);

        auto packet_size_option = std::make_shared<cs_pu_option>(raw_depth_sensor, RS2_OPTION_PACKET_SIZE, CS_STREAM_DEPTH);
        depth_sensor.register_option(RS2_OPTION_PACKET_SIZE, packet_size_option);

        auto output_trigger_enabled_option = std::make_shared<cs_pu_option>(raw_depth_sensor, RS2_OPTION_OUTPUT_TRIGGER_ENABLED, CS_STREAM_DEPTH);
        depth_sensor.register_option(RS2_OPTION_OUTPUT_TRIGGER_ENABLED, output_trigger_enabled_option);

        if (_cs_device->is_software_trigger_supported()) {
            depth_sensor.register_option(RS2_OPTION_SOFTWARE_TRIGGER,
                std::make_shared<cs_software_trigger_option>(raw_depth_sensor, RS2_OPTION_SOFTWARE_TRIGGER, CS_STREAM_DEPTH,
                    std::map<float, std::string>{ { 1.f, "Trigger" }}));

            auto sw_trigger_all = std::make_shared<cs_software_trigger_all_option>(raw_depth_sensor, RS2_OPTION_SOFTWARE_TRIGGER_ALL_SENSORS, CS_STREAM_DEPTH);
            depth_sensor.register_option(RS2_OPTION_SOFTWARE_TRIGGER_ALL_SENSORS, sw_trigger_all);

            depth_sensor.register_option(RS2_OPTION_EXT_TRIGGER_SOURCE,
                std::make_shared<cs_external_trigger_option>(raw_depth_sensor, RS2_OPTION_EXT_TRIGGER_SOURCE, CS_STREAM_DEPTH,
                    std::map<float, std::string>{ { 1.f, "Hardware" },
                    { 2.f, "Software" }}));

            depth_sensor.register_option(RS2_OPTION_USER_OUTPUT_LEVEL,
                std::make_shared<cs_user_output_level_option>(raw_depth_sensor, RS2_OPTION_USER_OUTPUT_LEVEL, CS_STREAM_DEPTH,
                    std::map<float, std::string>{ { 1.f, "Low" },
                    { 2.f, "High" }}));
        }

        if (_cs_device->is_line_debouncer_time_supported()) {
            auto line_debouncer_time_option = std::make_shared<cs_pu_option>(raw_depth_sensor, RS2_OPTION_LINE_DEBOUNCER_TIME, CS_STREAM_DEPTH);
            depth_sensor.register_option(RS2_OPTION_LINE_DEBOUNCER_TIME, line_debouncer_time_option);
        }



        // METADATA REGISTER (+HDR) //

        // check if metadata is supported and enabled
        if (_cs_device->is_metadata_supported()) {

            depth_sensor.register_option(RS2_OPTION_METADATA_TOGGLE, std::make_shared<cs_pu_option>(raw_depth_sensor, RS2_OPTION_METADATA_TOGGLE, CS_STREAM_DEPTH));

            depth_sensor.register_metadata(RS2_FRAME_METADATA_FRAME_TIMESTAMP, make_cs_uvc_header_parser(&framos_uvc_header::timestamp)); // TODO NH - Check "make_uvc_header_parser"!


            // attributes of md_capture_timing
            auto md_prop_offset = offsetof(metadata_raw, mode) + offsetof(md_depth_mode, depth_y_mode) + offsetof(md_depth_y_normal_mode, intel_capture_timing);
            
            depth_sensor.register_metadata(RS2_FRAME_METADATA_FRAME_COUNTER, make_attribute_parser(&md_capture_timing::frame_counter, md_capture_timing_attributes::frame_counter_attribute, md_prop_offset));
            depth_sensor.register_metadata(RS2_FRAME_METADATA_SENSOR_TIMESTAMP, make_rs400_sensor_ts_parser(make_cs_uvc_header_parser(&framos_uvc_header::timestamp), make_attribute_parser(&md_capture_timing::sensor_timestamp, md_capture_timing_attributes::sensor_timestamp_attribute, md_prop_offset)));


            // attributes of md_capture_stats
            md_prop_offset = offsetof(metadata_raw, mode) + offsetof(md_depth_mode, depth_y_mode) + offsetof(md_depth_y_normal_mode, intel_capture_stats);

            depth_sensor.register_metadata(RS2_FRAME_METADATA_WHITE_BALANCE, make_attribute_parser(&md_capture_stats::white_balance, md_capture_stat_attributes::white_balance_attribute, md_prop_offset));


            // attributes of md_depth_control
            md_prop_offset = offsetof(metadata_raw, mode) + offsetof(md_depth_mode, depth_y_mode) + offsetof(md_depth_y_normal_mode, intel_depth_control);

            depth_sensor.register_metadata(RS2_FRAME_METADATA_GAIN_LEVEL, make_attribute_parser(&md_depth_control::manual_gain, md_depth_control_attributes::gain_attribute, md_prop_offset));
            depth_sensor.register_metadata(RS2_FRAME_METADATA_ACTUAL_EXPOSURE, make_attribute_parser(&md_depth_control::manual_exposure, md_depth_control_attributes::exposure_attribute, md_prop_offset));
            depth_sensor.register_metadata(RS2_FRAME_METADATA_AUTO_EXPOSURE, make_attribute_parser(&md_depth_control::auto_exposure_mode, md_depth_control_attributes::ae_mode_attribute, md_prop_offset));
            depth_sensor.register_metadata(RS2_FRAME_METADATA_FRAME_LASER_POWER, make_attribute_parser(&md_depth_control::laser_power, md_depth_control_attributes::laser_pwr_attribute, md_prop_offset));
            depth_sensor.register_metadata(RS2_FRAME_METADATA_FRAME_LASER_POWER_MODE, make_attribute_parser(&md_depth_control::emitterMode, md_depth_control_attributes::emitter_mode_attribute, md_prop_offset, [](const rs2_metadata_type& param) { return param == 1 ? 1 : 0; })); // starting at version 2.30.1 this control is superceeded by RS2_FRAME_METADATA_FRAME_EMITTER_MODE
            depth_sensor.register_metadata(RS2_FRAME_METADATA_EXPOSURE_PRIORITY, make_attribute_parser(&md_depth_control::exposure_priority, md_depth_control_attributes::exposure_priority_attribute, md_prop_offset));
            depth_sensor.register_metadata(RS2_FRAME_METADATA_EXPOSURE_ROI_LEFT, make_attribute_parser(&md_depth_control::exposure_roi_left, md_depth_control_attributes::roi_attribute, md_prop_offset));
            depth_sensor.register_metadata(RS2_FRAME_METADATA_EXPOSURE_ROI_RIGHT, make_attribute_parser(&md_depth_control::exposure_roi_right, md_depth_control_attributes::roi_attribute, md_prop_offset));
            depth_sensor.register_metadata(RS2_FRAME_METADATA_EXPOSURE_ROI_TOP, make_attribute_parser(&md_depth_control::exposure_roi_top, md_depth_control_attributes::roi_attribute, md_prop_offset));
            depth_sensor.register_metadata(RS2_FRAME_METADATA_EXPOSURE_ROI_BOTTOM, make_attribute_parser(&md_depth_control::exposure_roi_bottom, md_depth_control_attributes::roi_attribute, md_prop_offset));
            depth_sensor.register_metadata(RS2_FRAME_METADATA_FRAME_EMITTER_MODE, make_attribute_parser(&md_depth_control::emitterMode, md_depth_control_attributes::emitter_mode_attribute, md_prop_offset));
            depth_sensor.register_metadata(RS2_FRAME_METADATA_FRAME_LED_POWER, make_attribute_parser(&md_depth_control::ledPower, md_depth_control_attributes::led_power_attribute, md_prop_offset));


            // md_configuration (will be used for internal validation only)
            md_prop_offset = offsetof(metadata_raw, mode) + offsetof(md_depth_mode, depth_y_mode) + offsetof(md_depth_y_normal_mode, intel_configuration);

            depth_sensor.register_metadata((rs2_frame_metadata_value)RS2_FRAME_METADATA_HW_TYPE, make_attribute_parser(&md_configuration::hw_type, md_configuration_attributes::hw_type_attribute, md_prop_offset));
            depth_sensor.register_metadata((rs2_frame_metadata_value)RS2_FRAME_METADATA_SKU_ID, make_attribute_parser(&md_configuration::sku_id, md_configuration_attributes::sku_id_attribute, md_prop_offset));
            depth_sensor.register_metadata((rs2_frame_metadata_value)RS2_FRAME_METADATA_FORMAT, make_attribute_parser(&md_configuration::format, md_configuration_attributes::format_attribute, md_prop_offset));
            depth_sensor.register_metadata((rs2_frame_metadata_value)RS2_FRAME_METADATA_WIDTH, make_attribute_parser(&md_configuration::width, md_configuration_attributes::width_attribute, md_prop_offset));
            depth_sensor.register_metadata((rs2_frame_metadata_value)RS2_FRAME_METADATA_HEIGHT, make_attribute_parser(&md_configuration::height, md_configuration_attributes::height_attribute, md_prop_offset));
            depth_sensor.register_metadata((rs2_frame_metadata_value)RS2_FRAME_METADATA_ACTUAL_FPS, std::make_shared<cs_md_attribute_actual_fps>());


            // minimal firmware version in which hdr feature is supported
            firmware_version hdr_firmware_version("5.12.8.100");
            if (_fw_version >= hdr_firmware_version) {

                // attributes of md_capture_timing
                auto md_prop_offset = offsetof(metadata_raw, mode) + offsetof(md_depth_mode, depth_y_mode) + offsetof(md_depth_y_normal_mode, intel_configuration);

                depth_sensor.register_metadata(RS2_FRAME_METADATA_SEQUENCE_SIZE,
                    make_attribute_parser(&md_configuration::sub_preset_info,
                        md_configuration_attributes::sub_preset_info_attribute, md_prop_offset,
                        [](const rs2_metadata_type& param) {
                            // bit mask and offset used to get data from bitfield
                            return (param & md_configuration::SUB_PRESET_BIT_MASK_SEQUENCE_SIZE)
                                >> md_configuration::SUB_PRESET_BIT_OFFSET_SEQUENCE_SIZE;
                        }));

                depth_sensor.register_metadata(RS2_FRAME_METADATA_SEQUENCE_ID,
                    make_attribute_parser(&md_configuration::sub_preset_info,
                        md_configuration_attributes::sub_preset_info_attribute, md_prop_offset,
                        [](const rs2_metadata_type& param) {
                            // bit mask and offset used to get data from bitfield
                            return (param & md_configuration::SUB_PRESET_BIT_MASK_SEQUENCE_ID)
                                >> md_configuration::SUB_PRESET_BIT_OFFSET_SEQUENCE_ID;
                        }));

                depth_sensor.register_metadata(RS2_FRAME_METADATA_SEQUENCE_NAME,
                    make_attribute_parser(&md_configuration::sub_preset_info,
                        md_configuration_attributes::sub_preset_info_attribute, md_prop_offset,
                        [](const rs2_metadata_type& param) {
                            // bit mask and offset used to get data from bitfield
                            return (param & md_configuration::SUB_PRESET_BIT_MASK_ID)
                                >> md_configuration::SUB_PRESET_BIT_OFFSET_ID;
                        }));



                // HDR REGISTER //

                std::shared_ptr<option> exposure_option = nullptr;
                std::shared_ptr<option> gain_option = nullptr;
                std::shared_ptr<cs_hdr_option> hdr_enabled_option = nullptr;

                // EXPOSURE preparing options
                auto cs_exposure_option = std::make_shared<cs_depth_exposure_option>(raw_depth_sensor, RS2_OPTION_EXPOSURE, CS_STREAM_DEPTH);
                option_range exposure_range = cs_exposure_option->get_range();

                // GAIN preparing options
                auto cs_gain_option = std::make_shared<cs_pu_option>(raw_depth_sensor, RS2_OPTION_GAIN, CS_STREAM_DEPTH);
                option_range gain_range = cs_gain_option->get_range();

                // AUTO EXPOSURE
                auto cs_auto_exposure_option = std::make_shared<cs_pu_option>(raw_depth_sensor, RS2_OPTION_ENABLE_AUTO_EXPOSURE, CS_STREAM_DEPTH);
                //depth_sensor.register_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, cs_auto_exposure_option);

                auto cs_depth = As<cs_depth_sensor, synthetic_sensor>(&get_depth_sensor());
                cs_depth->init_hdr_config(exposure_range, gain_range);
                auto&& hdr_cfg = cs_depth->get_hdr_config();

                // values from 4 to 14 - for internal use
                // value 15 - saved for emiter on off subpreset
                option_range hdr_id_range = { 0.f /*min*/, 3.f /*max*/, 1.f /*step*/, 1.f /*default*/ };
                auto hdr_id_option = std::make_shared<cs_hdr_option>(hdr_cfg, RS2_OPTION_SEQUENCE_NAME, hdr_id_range, std::map<float, std::string>{ {0.f, "0"}, { 1.f, "1" }, { 2.f, "2" }, { 3.f, "3" } });
                depth_sensor.register_option(RS2_OPTION_SEQUENCE_NAME, hdr_id_option);

                option_range hdr_sequence_size_range = { 2.f /*min*/, 2.f /*max*/, 1.f /*step*/, 2.f /*default*/ };
                auto hdr_sequence_size_option = std::make_shared<cs_hdr_option>(hdr_cfg, RS2_OPTION_SEQUENCE_SIZE, hdr_sequence_size_range, std::map<float, std::string>{ { 2.f, "2" } });
                depth_sensor.register_option(RS2_OPTION_SEQUENCE_SIZE, hdr_sequence_size_option);

                option_range hdr_sequ_id_range = { 0.f /*min*/, 2.f /*max*/, 1.f /*step*/, 0.f /*default*/ };
                auto hdr_sequ_id_option = std::make_shared<cs_hdr_option>(hdr_cfg, RS2_OPTION_SEQUENCE_ID, hdr_sequ_id_range, std::map<float, std::string>{ {0.f, "UVC"}, { 1.f, "1" }, { 2.f, "2" } });
                depth_sensor.register_option(RS2_OPTION_SEQUENCE_ID, hdr_sequ_id_option);

                option_range hdr_enable_range = { 0.f /*min*/, 1.f /*max*/, 1.f /*step*/, 0.f /*default*/ };
                hdr_enabled_option = std::make_shared<cs_hdr_option>(hdr_cfg, RS2_OPTION_HDR_ENABLED, hdr_enable_range);
                depth_sensor.register_option(RS2_OPTION_HDR_ENABLED, hdr_enabled_option);

                // EXPOSURE AND GAIN - preparing hdr options
                auto hdr_exposure_option = std::make_shared<cs_hdr_option>(hdr_cfg, RS2_OPTION_EXPOSURE, exposure_range);
                auto hdr_gain_option = std::make_shared<cs_hdr_option>(hdr_cfg, RS2_OPTION_GAIN, gain_range);

                // EXPOSURE AND GAIN - preparing hybrid options
                auto hdr_conditional_exposure_option = std::make_shared<cs_hdr_conditional_option>(hdr_cfg, cs_exposure_option, hdr_exposure_option);
                auto hdr_conditional_gain_option = std::make_shared<cs_hdr_conditional_option>(hdr_cfg, cs_gain_option, hdr_gain_option);

                exposure_option = hdr_conditional_exposure_option;
                gain_option = hdr_conditional_gain_option;

                std::vector<std::pair<std::shared_ptr<option>, std::string>> options_and_reasons = { std::make_pair(hdr_enabled_option, "Auto Exposure cannot be set while HDR is enabled") };
                depth_sensor.register_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, std::make_shared<gated_option>(cs_auto_exposure_option, options_and_reasons));

                // Re-register EXPOSURE option
                depth_sensor.register_option(RS2_OPTION_EXPOSURE, std::make_shared<auto_disabling_control>(exposure_option, cs_auto_exposure_option));

                // Re-register GAIN option
                depth_sensor.register_option(RS2_OPTION_GAIN, std::make_shared<auto_disabling_control>(gain_option, cs_auto_exposure_option));

                int a = 0;
                int b = a;

                // !HDR REGISTER //
            }
        }

        // !METADATA REGISTER (+HDR) //

        // Auto exposure and gain limit
        // Supported only up to FW version 5.13.0.50, Intel disabled and consequently removed the feature in newer library releases
        if (_fw_version >= firmware_version("5.12.10.11") && _fw_version <= firmware_version("5.13.0.50"))
        {
            auto exposure_range = depth_sensor.get_option(RS2_OPTION_EXPOSURE).get_range();
            auto gain_range = depth_sensor.get_option(RS2_OPTION_GAIN).get_range();

            option_range enable_range = { 0.f /*min*/, 1.f /*max*/, 1.f /*step*/, 0.f /*default*/ };

            //GAIN Limit
            auto gain_limit_toggle_control = std::make_shared<cs_limits_option>(RS2_OPTION_AUTO_GAIN_LIMIT_TOGGLE, enable_range, "Toggle Auto-Gain Limit", *_hw_monitor, raw_depth_sensor);
            _gain_limit_value_control = std::make_shared<cs_auto_gain_limit_option>(*_hw_monitor, &depth_sensor, gain_range, gain_limit_toggle_control);
            depth_sensor.register_option(RS2_OPTION_AUTO_GAIN_LIMIT_TOGGLE, gain_limit_toggle_control);

            depth_sensor.register_option(RS2_OPTION_AUTO_GAIN_LIMIT,
                std::make_shared<auto_disabling_control>(
                    _gain_limit_value_control,
                    gain_limit_toggle_control

                ));

            // EXPOSURE Limit
            auto ae_limit_toggle_control = std::make_shared<cs_limits_option>(RS2_OPTION_AUTO_EXPOSURE_LIMIT_TOGGLE, enable_range, "Toggle Auto-Exposure Limit", *_hw_monitor, raw_depth_sensor);
            _ae_limit_value_control = std::make_shared<cs_auto_exposure_limit_option>(*_hw_monitor, &depth_sensor, exposure_range, ae_limit_toggle_control);
            depth_sensor.register_option(RS2_OPTION_AUTO_EXPOSURE_LIMIT_TOGGLE, ae_limit_toggle_control);

            depth_sensor.register_option(RS2_OPTION_AUTO_EXPOSURE_LIMIT,
                std::make_shared<auto_disabling_control>(
                    _ae_limit_value_control,
                    ae_limit_toggle_control

                ));
        }

        enable_time_diff_keeper(true);
    }

    std::shared_ptr<synthetic_sensor> cs_color::create_color_device(std::shared_ptr<context> ctx,
                                                             std::shared_ptr<platform::cs_device> cs_device)
    {
        auto&& backend = ctx->get_backend();
        std::unique_ptr<frame_timestamp_reader> cs_timestamp_reader_backup(new cs_timestamp_reader(backend.create_time_service()));
        std::unique_ptr<frame_timestamp_reader> cs_timestamp_reader_metadata(new cs_timestamp_reader_from_metadata(std::move(cs_timestamp_reader_backup)));

        auto enable_global_time_option = std::shared_ptr<global_time_option>(new cs_global_time_option_color(this));
        auto raw_color_ep = std::make_shared<cs_sensor>("Raw RGB Camera", cs_device,
                std::unique_ptr<frame_timestamp_reader>(new global_timestamp_reader(std::move(cs_timestamp_reader_metadata), _tf_keeper, enable_global_time_option)),
                this, CS_STREAM_COLOR);

        auto color_ep = std::make_shared<cs_color_sensor>(this, raw_color_ep, cs_color_fourcc_to_rs2_format, cs_color_fourcc_to_rs2_stream);
        color_ep->register_option(RS2_OPTION_GLOBAL_TIME_ENABLED, enable_global_time_option);

        color_ep->register_processing_block(processing_block_factory::create_pbf_vector<uyvy_converter>(RS2_FORMAT_UYVY, map_supported_color_formats(RS2_FORMAT_UYVY), RS2_STREAM_COLOR));
        color_ep->register_processing_block(processing_block_factory::create_pbf_vector<yuy2_converter>(RS2_FORMAT_YUYV, map_supported_color_formats(RS2_FORMAT_YUYV), RS2_STREAM_COLOR));
        color_ep->register_processing_block(processing_block_factory::create_id_pbf(RS2_FORMAT_RAW16, RS2_STREAM_COLOR));

        return color_ep;
    }

    std::shared_ptr<synthetic_sensor> cs_depth::create_depth_device(std::shared_ptr<context> ctx,
                                                                    std::shared_ptr<platform::cs_device> cs_device)
    {
        auto&& backend = ctx->get_backend();

        std::unique_ptr<frame_timestamp_reader> timestamp_reader_backup(new cs_timestamp_reader(backend.create_time_service()));
        std::unique_ptr<frame_timestamp_reader> timestamp_reader_metadata(new cs_timestamp_reader_from_metadata(std::move(timestamp_reader_backup)));
        auto enable_global_time_option = std::shared_ptr<global_time_option>(new cs_global_time_option_depth(this));
        auto raw_depth_ep = std::make_shared<cs_sensor>("Raw Depth Sensor", cs_device,
                std::unique_ptr<frame_timestamp_reader>(new global_timestamp_reader(std::move(timestamp_reader_metadata), _tf_keeper, enable_global_time_option)), this, CS_STREAM_DEPTH);

        auto depth_ep = std::make_shared<cs_depth_sensor>(this, raw_depth_ep, cs_depth_fourcc_to_rs2_format, cs_depth_fourcc_to_rs2_stream);
        depth_ep->register_option(RS2_OPTION_GLOBAL_TIME_ENABLED, enable_global_time_option);

        depth_ep->register_processing_block(processing_block_factory::create_id_pbf(RS2_FORMAT_Y8, RS2_STREAM_INFRARED, 1));
        depth_ep->register_processing_block(processing_block_factory::create_id_pbf(RS2_FORMAT_Z16, RS2_STREAM_DEPTH));

        depth_ep->register_processing_block({ {RS2_FORMAT_W10} }, { {RS2_FORMAT_RAW10, RS2_STREAM_INFRARED, 1} }, []() { return std::make_shared<w10_converter>(RS2_FORMAT_RAW10); });
        depth_ep->register_processing_block({ {RS2_FORMAT_W10} }, { {RS2_FORMAT_Y10BPACK, RS2_STREAM_INFRARED, 1} }, []() { return std::make_shared<w10_converter>(RS2_FORMAT_Y10BPACK); });

        depth_ep->register_processing_block(processing_block_factory::create_pbf_vector<uyvy_converter>(RS2_FORMAT_UYVY, map_supported_color_formats(RS2_FORMAT_UYVY), RS2_STREAM_INFRARED));
        depth_ep->register_processing_block(processing_block_factory::create_pbf_vector<yuy2_converter>(RS2_FORMAT_YUYV, map_supported_color_formats(RS2_FORMAT_YUYV), RS2_STREAM_INFRARED));

        return depth_ep;
    }

    bool cs_depth::is_camera_in_advanced_mode() const
    {
        command cmd(ds::UAMG);
        assert(_hw_monitor);
        auto ret = _hw_monitor->send(cmd);
        if (ret.empty())
            throw invalid_value_exception("command result is empty!");

        return (0 != ret.front());
    }

    float cs_depth::get_stereo_baseline_mm() const
    {
        using namespace ds;

        auto table = check_calib<coefficients_table>(*_depth_calib_table_raw);
        return fabs(table->baseline);
    }

    std::vector<uint8_t> cs_color::get_raw_calibration_table(ds::calibration_table_id table_id) const
    {
        command cmd(ds::GETINTCAL, table_id);

        std::vector<uint8_t> temp_color_calib_table_raw = _hw_monitor->send(cmd);
        auto header = reinterpret_cast<const ds::table_header*>(temp_color_calib_table_raw.data());

        temp_color_calib_table_raw.resize(header->table_size + sizeof(ds::table_header));

        return temp_color_calib_table_raw;
    }

    std::vector<uint8_t> cs_depth::get_raw_calibration_table(ds::calibration_table_id table_id) const
    {
        command cmd(ds::GETINTCAL, table_id);

        std::vector<uint8_t> temp_depth_calib_table_raw = _hw_monitor->send(cmd);
        auto header = reinterpret_cast<const ds::table_header*>(temp_depth_calib_table_raw.data());

        temp_depth_calib_table_raw.resize(header->table_size + sizeof(ds::table_header));

        return temp_depth_calib_table_raw;
    }

    std::vector<uint8_t> cs_depth::get_new_calibration_table() const
    {
        //TODO check if resize required
        if (_fw_version >= firmware_version("5.11.9.5"))
        {
            command cmd(ds::RECPARAMSGET);
            return _hw_monitor->send(cmd);
        }
        return {};
    }

    ds::d400_caps cs_depth::parse_device_capabilities(const uint16_t pid) const
    {
        using namespace ds;
        std::array<unsigned char,HW_MONITOR_BUFFER_SIZE> gvd_buf;
        _hw_monitor->get_gvd(gvd_buf.size(), gvd_buf.data(), GVD);

        // Opaque retrieval
        d400_caps val{d400_caps::CAP_UNDEFINED};
        if (gvd_buf[active_projector])  // DepthActiveMode
            val |= d400_caps::CAP_ACTIVE_PROJECTOR;
        if (gvd_buf[rgb_sensor])                           // WithRGB
            val |= d400_caps::CAP_RGB_SENSOR;
        if (gvd_buf[imu_sensor])
        {
            val |= d400_caps::CAP_IMU_SENSOR;
            if (hid_bmi_055_pid.end() != hid_bmi_055_pid.find(pid))
                val |= d400_caps::CAP_BMI_055;
            else
            {
                if (hid_bmi_085_pid.end() != hid_bmi_085_pid.find(pid))
                    val |= d400_caps::CAP_BMI_085;
                else
                    LOG_WARNING("The IMU sensor is undefined for PID " << std::hex << pid << std::dec);
            }
        }
        if (0xFF != (gvd_buf[fisheye_sensor_lb] & gvd_buf[fisheye_sensor_hb]))
            val |= d400_caps::CAP_FISHEYE_SENSOR;
        if (0x1 == gvd_buf[depth_sensor_type])
            val |= d400_caps::CAP_ROLLING_SHUTTER;  // e.g. ASRC
        if (0x2 == gvd_buf[depth_sensor_type])
            val |= d400_caps::CAP_GLOBAL_SHUTTER;   // e.g. AWGC
        // Option INTER_CAM_SYNC_MODE is not enabled in D405
        if (pid != ds::RS405_PID)
            val |= d400_caps::CAP_INTERCAM_HW_SYNC;
        return val;
    }

    processing_blocks cs_depth::get_cs_depth_recommended_proccesing_blocks() const
    {
        auto res = get_depth_recommended_proccesing_blocks();
        res.push_back(std::make_shared<hdr_merge>()); // Requires HDR
        res.push_back(std::make_shared<sequence_id_filter>());
        res.push_back(std::make_shared<threshold>());
        res.push_back(std::make_shared<disparity_transform>(true));
        res.push_back(std::make_shared<spatial_filter>());
        res.push_back(std::make_shared<temporal_filter>());
        res.push_back(std::make_shared<hole_filling_filter>());
        res.push_back(std::make_shared<disparity_transform>(false));
        return res;
    }

    std::vector<uint8_t> cs_depth::send_receive_raw_data(const std::vector<uint8_t>& input)
    {
        return _hw_monitor->send(input);
    }

    void cs_depth::create_snapshot(std::shared_ptr<debug_interface>& snapshot) const
    {
        //TODO: Implement
    }

    void cs_depth::enable_recording(std::function<void(const debug_interface&)> record_action)
    {
        //TODO: Implement
    }

    command cs_depth::get_firmware_logs_command() const
    {
        return command{ ds::GLD, 0x1f4 };
    }

    command cs_depth::get_flash_logs_command() const
    {
        return command{ ds::FRB, 0x17a000, 0x3f8 };
    }

    processing_blocks cs_color_sensor::get_recommended_processing_blocks() const
    {
        return get_color_recommended_proccesing_blocks();
    }

    processing_blocks cs_depth_sensor::get_recommended_processing_blocks() const
    {
        return _owner->get_cs_depth_recommended_proccesing_blocks();
    }

    void cs_depth_sensor::set_frame_metadata_modifier(on_frame_md callback)
    {
        _metadata_modifier = callback;
        auto s = get_raw_sensor().get();
        auto cs = As< librealsense::sensor_base >(s);
        if (cs)
            cs->set_frame_metadata_modifier(callback);
    }

    float cs_depth_sensor::get_preset_max_value() const
    {
        float preset_max_value = RS2_RS400_VISUAL_PRESET_COUNT - 1;
        switch (_owner->_pid)
        {
        case ds::RS400_PID:
        case ds::RS410_PID:
        case ds::RS415_PID:
        case ds::RS465_PID:
        case ds::RS460_PID:
            preset_max_value = static_cast<float>(RS2_RS400_VISUAL_PRESET_REMOVE_IR_PATTERN);
            break;
        default:
            preset_max_value = static_cast<float>(RS2_RS400_VISUAL_PRESET_MEDIUM_DENSITY);
        }
        return preset_max_value;
    }

    void cs_depth_sensor::open(const stream_profiles& requests)
    {
        //modifyed
      /* group_multiple_fw_calls(*this, [&]() {
            _depth_units = get_option(RS2_OPTION_DEPTH_UNITS).query();
            set_frame_metadata_modifier([&](frame_additional_data& data) {data.depth_units = _depth_units.load(); });          
            synthetic_sensor::open(requests);
            });
        */

        /*Reimplemented function group_multiple_fw_calls() since the Intel's function group_multiple_fw_calls() does not support cs_devices*/
        auto& cs = dynamic_cast<cs_sensor&>(*this->get_raw_sensor());
        
        
        cs.invoke_powered(
            [&](platform::cs_device& dev)
            {
                _depth_units = get_option(RS2_OPTION_DEPTH_UNITS).query();
                set_frame_metadata_modifier([&](frame_additional_data& data) {data.depth_units = _depth_units.load(); });
                synthetic_sensor::open(requests);
            });
       
        //_depth_units = get_option(RS2_OPTION_DEPTH_UNITS).query();
       // synthetic_sensor::open(requests);
       
    }

    void cs_depth_sensor::create_snapshot(std::shared_ptr<depth_sensor>& snapshot) const
    {
        snapshot = std::make_shared<depth_sensor_snapshot>(get_depth_scale());
    }

    void cs_depth_sensor::create_snapshot(std::shared_ptr<depth_stereo_sensor>& snapshot) const
    {
        snapshot = std::make_shared<depth_stereo_sensor_snapshot>(get_depth_scale(), get_stereo_baseline_mm());
    }

    void cs_depth_sensor::enable_recording(std::function<void(const depth_sensor&)> recording_function)
    {
        //does not change over time
    }

    void cs_depth_sensor::enable_recording(std::function<void(const depth_stereo_sensor&)> recording_function)
    {
        //does not change over time
    }

    rs2_intrinsics cs_color_sensor::get_intrinsics(const stream_profile& profile) const
    {
        rs2_intrinsics result = get_intrinsic_by_resolution(
                *_owner->_color_calib_table_raw,
                ds::calibration_table_id::rgb_calibration_id,
                profile.width, profile.height);
        //std::cout << "###Color:-> width: "<<  result.width << ", height: " << result.height << ", focal x: " << result.fx << ", focal y: " << result.fy << std::endl;
        //LOG_WARNING("###Color:-> width: "<<  result.width << ", height: " << result.height << ", focal x: " << result.fx << ", focal y: " << result.fy);
        return get_intrinsic_by_resolution(
                *_owner->_color_calib_table_raw,
                ds::calibration_table_id::rgb_calibration_id,
                profile.width, profile.height);
    }

    rs2_intrinsics cs_depth_sensor::get_intrinsics(const stream_profile& profile) const
    {
        rs2_intrinsics result;

        if (ds::try_get_intrinsic_by_resolution_new(*_owner->_new_calib_table_raw,
            profile.width, profile.height, &result))
        {
            //std::cout << "###Depth:-> width: "<<  result.width << ", height: " << result.height << ", focal x: " << result.fx << ", focal y: " << result.fy << std::endl; 
            //LOG_WARNING("###Depth:-> width: "<<  result.width << ", height: " << result.height << ", focal x: " << result.fx << ", focal y: " << result.fy);
            return result;
        }
        else
        {
            result = get_intrinsic_by_resolution(
                    *_owner->_depth_calib_table_raw,
                    ds::calibration_table_id::coefficients_table_id,
                    profile.width, profile.height);
            //std::cout << "###Depth:-> width: "<<  result.width << ", height: " << result.height << ", focal x: " << result.fx << ", focal y: " << result.fy << std::endl; 
            //LOG_WARNING("###Depth:-> width: "<<  result.width << ", height: " << result.height << ", focal x: " << result.fx << ", focal y: " << result.fy);
            return get_intrinsic_by_resolution(
                    *_owner->_depth_calib_table_raw,
                ds::calibration_table_id::coefficients_table_id,
                profile.width, profile.height);
        }
    }

        rs2_intrinsics cs_depth_sensor::get_color_intrinsics(const stream_profile& profile) const
        {
            return get_intrinsic_by_resolution(
                *_owner->_color_calib_table_raw,
                ds::calibration_table_id::rgb_calibration_id,
                profile.width, profile.height);
        }


    stream_profiles cs_depth_sensor::init_stream_profiles()
    {
        auto lock = environment::get_instance().get_extrinsics_graph().lock();

        auto&& results = synthetic_sensor::init_stream_profiles();

        for (auto&& p : results)
        {
            // Register stream types
            if (p->get_stream_type() == RS2_STREAM_DEPTH)
            {
                assign_stream(_owner->_depth_stream, p);
            }
            else if (p->get_stream_type() == RS2_STREAM_INFRARED && p->get_stream_index() < 2)
            {
                assign_stream(_owner->_left_ir_stream, p);
            }
            else if (p->get_stream_type() == RS2_STREAM_INFRARED  && p->get_stream_index() == 2)
            {
                assign_stream(_owner->_right_ir_stream, p);
            }
            auto&& vid_profile = dynamic_cast<video_stream_profile_interface*>(p.get());

            // used when color stream comes from depth sensor (as in D405)
            if (p->get_stream_type() == RS2_STREAM_COLOR)
            {
                const auto&& profile = to_profile(p.get());
                std::weak_ptr<cs_depth_sensor> wp =
                    std::dynamic_pointer_cast<cs_depth_sensor>(this->shared_from_this());
                vid_profile->set_intrinsics([profile, wp]()
                    {
                        auto sp = wp.lock();
                        if (sp)
                            return sp->get_color_intrinsics(profile);
                        else
                            return rs2_intrinsics{};
                    });
            }
            // Register intrinsics
            if (p->get_format() != RS2_FORMAT_Y16) // Y16 format indicate unrectified images, no intrinsics are available for these
            {
                const auto&& profile = to_profile(p.get());
                std::weak_ptr<cs_depth_sensor> wp =
                        std::dynamic_pointer_cast<cs_depth_sensor>(this->shared_from_this());
                vid_profile->set_intrinsics([profile, wp]()
                                            {
                                                auto sp = wp.lock();
                                                if (sp)
                                                    return sp->get_intrinsics(profile);
                                                else
                                                    return rs2_intrinsics{};
                                            });
            }
        }

        return results;
    }

    stream_profiles cs_color_sensor::init_stream_profiles()
    {
        auto lock = environment::get_instance().get_extrinsics_graph().lock();
        auto&& results = synthetic_sensor::init_stream_profiles();

        for (auto&& p : results)
        {
            // Register stream types
            if (p->get_stream_type() == RS2_STREAM_COLOR)
            {
                assign_stream(_owner->_color_stream, p);
            }

            auto&& video = dynamic_cast<video_stream_profile_interface*>(p.get());
            const auto&& profile = to_profile(p.get());

            std::weak_ptr<cs_color_sensor> wp =
                    std::dynamic_pointer_cast<cs_color_sensor>(this->shared_from_this());
            video->set_intrinsics([profile, wp]()
                                  {
                                      auto sp = wp.lock();
                                      if (sp)
                                          return sp->get_intrinsics(profile);
                                      else
                                          return rs2_intrinsics{};
                                  });
        }
        return results;
    }
}
