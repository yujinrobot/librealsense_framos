// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 FRAMOS GmbH.

#ifndef LIBREALSENSE2_CS_FACTORY_H
#define LIBREALSENSE2_CS_FACTORY_H

#include <cs/cs-sensor.h>
#include <cs/cs-device.h>
#include <cs/cs-motion.h>
#include "context.h"
#include <mutex>
#include "../../../third-party/json.hpp"

#include "../firmware_logger_device.h"

namespace librealsense {

using json = nlohmann::json;

#define CS_PACKET_RESEND_GROUP_MAX_SIZE 1
const std::string CS_HID_ACCL_UNIQUE_ID = "FRAMOS_6CD146_ACCL";
const std::string CS_HID_GYRO_UNIQUE_ID = "FRAMOS_6CD146_GYRO";
const std::string CS_FRAMOS_SERIAL_BEGIN = "6CD146";
/**/
const uint32_t CS_DEV_SERIAL_SYMBOL_COUNT = 6 * 2 + 5; // xx:xx:xx:xx:xx:xx
const char CS_DEV_SERIAL_VALID_SYMBOLS[] = "0123456789abcdefABCDEF";
const uint8_t CS_DEV_SERIAL_COLON_SYMBOL = 0x3A;
const uint8_t CS_DEV_SERIAL_VALID_DIGIT_START = 0x30;
const uint8_t CS_DEV_SERIAL_VALID_DIGIT_END = 0x39;
const uint8_t CS_DEV_SERIAL_VALID_CHAR_START = 0x61;
const uint8_t CS_DEV_SERIAL_VALID_CHAR_END = 0x66;
const uint8_t CS_DEV_SERIAL_VALID_CHAR_CAPS_START = 0x41;
const uint8_t CS_DEV_SERIAL_VALID_CHAR_CAPS_END = 0x46;
/**/
const std::string CS_JSON_OBJ_IP = "ip";
const std::string CS_JSON_OBJ_IP_RANGE = "ip_range";
const std::string CS_JSON_OBJ_SERIAL = "serial";
const std::string CS_JSON_OBJ_SERIAL_RANGE = "serial_range";
const std::string CS_JSON_OBJ_SUBNET = "subnet";
const std::string CS_JSON_FLAG_ALLOW = "allow";
const std::string CS_JSON_FLAG_BEGIN = "begin";
const std::string CS_JSON_FLAG_END = "end";

    struct cs_dev_filter
    {
        struct filter_allowed
        {
            filter_allowed() { allowed = false; }

            bool allowed;   // DEPRECATED!
        };

        struct filter_serial : public filter_allowed
        {
            filter_serial()
            {
                serial = 0;
            }

            std::string serial_s;
            uint64_t serial;
        };

        struct filter_serial_range : public filter_allowed
        {
            filter_serial_range()
            {
                serial_begin = 0;
                serial_end = 0;
            }

            std::string serial_begin_s;
            uint64_t serial_begin;
            std::string serial_end_s;
            uint64_t serial_end;
        };

        struct filter_subnet : public filter_allowed
        {
            uint8_t subnet;
        };

        struct filter_ip : public filter_allowed
        {
            uint32_t ip;
        };

        struct filter_ip_range : public filter_allowed
        {
            uint32_t ip_begin;
            uint32_t ip_end;
        };

        cs_dev_filter()
        {
            _is_valid_on_init = false;
            _is_valid = false;
        }

        bool is_ena() const
        {
            if ((_filter_serial_range.size() > 0) ||
                /*(_filter_serial.size() > 0) ||*/
                (_filter_ip_range.size() > 0) /*||
                (_filter_ip.size() > 0) ||*/
                /*(_filter_subnet.size() > 0)*/) {
                return true;
            }
            return false;
        }

        std::vector<filter_serial_range> _filter_serial_range;
        std::vector<filter_serial> _filter_serial;  // DEPRECATED!
        std::vector<filter_ip_range> _filter_ip_range;
        std::vector<filter_ip> _filter_ip;  // DEPRECATED!
        std::vector<filter_subnet> _filter_subnet;  // DEPRECATED!
        bool _is_valid_on_init;
        bool _is_valid;
    };

    typedef std::vector<cs_dev_filter::filter_serial_range>::const_iterator const_it_filter_serial_range;
    typedef std::vector<cs_dev_filter::filter_ip_range>::const_iterator const_it_filter_ip_range;

    struct cs_dev_filter_holder
    {
        cs_dev_filter_holder() 
        {}

        bool is_ena() const
        {
            if (allow.is_ena() || deny.is_ena()) {
                return true;
            }
            return false;
        }

        bool is_valid() const
        {
            if ((allow._is_valid_on_init) && (allow._is_valid)/* && 
                (deny._is_valid_on_init) && (deny._is_valid)*/) {
                return true;
            }
            return false;
        }

        cs_dev_filter allow;
        cs_dev_filter deny; // DEPRECATED! 
    };

    class cs_info : public device_info {
    public:
        std::shared_ptr <device_interface> create(std::shared_ptr <context> ctx,
                                                  bool register_device_notifications) const override;

        cs_info(std::shared_ptr <context> ctx,
                platform::cs_device_info hwm)
                : device_info(ctx),
                  _hwm(std::move(hwm)) {}

        static std::vector <std::shared_ptr<device_info>> pick_cs_devices(
                std::shared_ptr <context> ctx,
                std::vector <platform::cs_device_info> &cs);

        platform::backend_device_group get_device_data() const override {
            return platform::backend_device_group({_hwm});
        }

        static std::vector <platform::cs_device_info> query_cs_devices();

        static cs_camera_model get_camera_model(std::string pid);

        static bool is_timestamp_supported(std::string pid);

    private:
        platform::cs_device_info _hwm;
    };

    class d400e_camera;

    class cs_device_watcher : public smcs::ICallbackEvent
    {
    public:
        void OnDisconnect(smcs::IDevice device) override;
        static cs_device_watcher& get_cs_device_watcher();
        std::vector <platform::cs_device_info> find_cs_devices(double timer);
        void add_d400e_device(d400e_camera* camera);
        void remove_d400e_device(d400e_camera* camera);
        void handle_disconnected_device(smcs::IDevice device);

    private:
        cs_device_watcher();
        platform::cs_device_info get_cs_device_info(smcs::IDevice device);

    private:
        void filter_out_devices(smcs::DevicesList& device_list, const cs_dev_filter_holder& dev_filter_holder, bool is_dev_filter_req);
        bool handle_deny_list(smcs::IDevice device, const cs_dev_filter& dev_filter_deny) const;
        bool handle_allow_list(smcs::IDevice device, const cs_dev_filter& dev_filter_allow) const;
        void device_filter_list_setup(cs_dev_filter_holder& dev_filter_holder);
        bool validate_filter_list_on_init(cs_dev_filter_holder& dev_filter_holder);
        bool validate_filter_list(cs_dev_filter_holder& dev_filter_holder, const smcs::DevicesList& device_list);
        bool verify_serial(const std::string& serial);
        uint64_t serial_from_string(const std::string& serial, bool revert);
        void parse_filter_file(const std::string& full_file_path, cs_dev_filter_holder& dev_filter_holder);
        void parse_ip_filter(json::const_iterator& it_object, bool is_allowed, cs_dev_filter_holder& dev_filter_holder);
        void parse_ip_range_filter(json::const_iterator& it_object, bool is_allowed, cs_dev_filter_holder& dev_filter_holder);
        void parse_serial_filter(json::const_iterator& it_object, bool is_allowed, cs_dev_filter_holder& dev_filter_holder);
        void parse_serial_range_filter(json::const_iterator& it_object, bool is_allowed, cs_dev_filter_holder& dev_filter_holder);
        void parse_subnet_filter(json::const_iterator& it_object, bool is_allowed, cs_dev_filter_holder& dev_filter_holder);

    private:
        std::mutex connected_cs_devices_mtx_;
        std::set<platform::cs_device_info> connected_cs_devices_;
        std::unordered_map<std::string, uint32_t> serial_connection_ids_;
        std::once_flag once_flag;
        cs_dev_filter_holder _dev_filter_holder;
        bool _is_dev_filter_req;
        std::vector<d400e_camera*> _d400e_camera_list;
    };

    class d400e_camera : /*public cs_depth,*/
                         public cs_color,
                         public cs_motion,
                         public cs_advanced_mode_base,
                         public firmware_logger_device
    {
    public:
        d400e_camera(std::shared_ptr<context> ctx,
                     const platform::backend_device_group& group,
                     bool register_device_notifications);
        ~d400e_camera();
        std::shared_ptr<matcher> create_matcher(const frame_holder& frame) const override;

        void update_matcher_configuration(const rs2_syncer_mode syncer_mode) const override { _syncer_mode = syncer_mode; };

        std::vector<tagged_profile> get_profiles_tags() const override;

        void depth_sensor_disable();
        void color_sensor_disable();
        void motion_sensor_disable();
        void timestamp_latch_disable();

        void hardware_reset() override
        {
            auto& color_sensor = get_sensor(_color_device_idx);
            if (color_sensor.is_streaming()) {
                color_sensor.stop();
                color_sensor.close();
            }

            auto& depth_sensor = get_sensor(_depth_device_idx);
            if (depth_sensor.is_streaming()) {
                depth_sensor.stop();
                depth_sensor.close();
            }

            try {
                auto& motion_sensor = get_sensor(_motion_module_device_idx.value());
                if (motion_sensor.is_streaming()) {
                    motion_sensor.stop();
                    motion_sensor.close();
                }
            }
            catch (std::exception e) {
                LOG_ERROR("FRAMOS - hardware_reset - stopping motion sensor: " << e.what());
            }

            _cs_device->reset();
        }

    private:
        std::string get_equivalent_pid(std::string id) const;
        mutable rs2_syncer_mode _syncer_mode = RS2_SYNCER_MODE_DEFAULT;
    };
}

#endif //LIBREALSENSE2_CS_FACTORY_H