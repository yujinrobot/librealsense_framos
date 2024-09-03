// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 FRAMOS GmbH.

#ifndef LIBREALSENSE2_CS_SENSOR_H
#define LIBREALSENSE2_CS_SENSOR_H

#include <smcs_cpp/CameraSDK.h>
#include <smcs_cpp/IImageBitmap.h>
#include "sensor.h"
#include "types.h"
#include <unordered_map>
#include "cs-event-capture.h"
#include <deque>

namespace librealsense {

    const std::string CS_CAMERA_MODEL_D435e = "D435e";
    const std::string CS_CAMERA_MODEL_D415e = "D415e";
    const std::string CS_CAMERA_MODEL_D455e = "D455e";

    const double CS_HWM_COMMAND_WAIT_TIME = 10.0;
    const double CS_HWM_MAX_WAIT_TIME = 20.0;
    const uint32_t CS_METADATA_SIZE = 2; // two lines

    const UINT32 CS_GEV_STREAM_CHANNEL0 = 0;
    const UINT32 CS_GEV_STREAM_CHANNEL1 = 1;
    const UINT32 CS_GEV_STREAM_CHANNEL2 = 2;

    typedef enum cs_stream {
        CS_STREAM_DEPTH = 0,
        CS_STREAM_COLOR,
        CS_STREAM_IR_LEFT,
        CS_STREAM_IR_RIGHT,
        CS_STREAM_IR_LEFT_COLOR,
        //CS_STREAM_IMU,
        CS_STREAM_COUNT
    } cs_stream;

    typedef enum cs_event_stream {
        CS_EVENT_STREAM_ACC = 0,
        CS_EVENT_STREAM_GYRO,
        CS_EVENT_STREAM_COUNT
    } cs_event_stream;

    enum cs_camera_model {
        CS_D435E,
        CS_D415E,
        CS_D455E,
        CS_UNDEFINED
    };

    typedef enum cs_inter_cam_sync_mode_gs {
        CS_INTERCAM_SYNC_DEFAULT_GS = 0,
        CS_INTERCAM_SYNC_MASTER_GS = 1,
        CS_INTERCAM_SYNC_SLAVE_GS = 2,
        CS_INTERCAM_SYNC_FULL_SLAVE_GS = 3,
        CS_INTERCAM_SYNC_EXTERNAL_GS = 259,
        CS_INTERCAM_SYNC_EXTERNAL_BURST_GS = 260,
        CS_INTERCAM_SYNC_MAX_GS = 261
    } cs_inter_cam_mode_gs;

    typedef enum cs_inter_cam_sync_mode {
        CS_INTERCAM_SYNC_DEFAULT,
        CS_INTERCAM_SYNC_MASTER,
        CS_INTERCAM_SYNC_SLAVE,
        CS_INTERCAM_SYNC_EXTERNAL,
        CS_INTERCAM_SYNC_EXTERNAL_BURST,
        CS_INTERCAM_SYNC_MAX
    } cs_inter_cam_mode;

    typedef enum cs_inter_cam_sync_mode_color {
        CS_INTERCAM_SYNC_DEFAULT_COLOR,
        CS_INTERCAM_SYNC_EXTERNAL_COLOR,
        CS_INTERCAM_SYNC_EXTERNAL_BURST_COLOR,
        CS_INTERCAM_SYNC_MAX_COLOR
    } cs_inter_cam_mode_color;

    typedef enum cs_inter_cam_sync_mode_color_gs {
        CS_INTERCAM_SYNC_DEFAULT_COLOR_GS,
        CS_INTERCAM_SYNC_EXTERNAL_COLOR_GS,
        CS_INTERCAM_SYNC_EXTERNAL_BURST_COLOR_GS,
        CS_INTERCAM_SYNC_GENLOCK_COLOR_GS,
        CS_INTERCAM_SYNC_MAX_COLOR_GS
    } cs_inter_cam_mode_color_gs;

    typedef enum cs_user_output_level {
        CS_USER_OUTPUT_LEVEL_LOW = 1,
        CS_USER_OUTPUT_LEVEL_HIGH,
        CS_USER_OUTPUT_LEVEL_MAX
    } cs_user_output_level;

    typedef enum cs_ext_event_burst_cnt {
        CS_EXT_EVENT_BURST_MIN = 1,
        CS_EXT_EVENT_BURST_MAX = 100000000,
        CS_EXT_EVENT_BURST_MAX_ENUM
    } cs_ext_event_burst_cnt;

    typedef enum imu_identifier {
        BMI055_IDENTIFIER = 0,
        BMI085_IDENTIFIER = 1,
        IMU_INVALID_ID
    } imu_identifier;

    struct cs_event_data
    {
        char data[GVCP_MAX_UDP_DATA_SIZE];
    };

    class cs_firmware_version
    {
    public:
        explicit cs_firmware_version(UINT32 major = 0, UINT32 minor = 0, UINT32 patch = 0, UINT32 build = 0)
            : _major(major)
            , _minor(minor)
            , _patch(patch)
            , _build(build)
        {}

        explicit cs_firmware_version(smcs::IDevice &device);

        bool operator > (const cs_firmware_version &other)
        {
            if (_major > other._major)
                return true;
            else if (_major < other._major)
                return false;

            if (_minor > other._minor)
                return true;
            else if (_minor < other._minor)
                return false;

            if (_patch > other._patch)
                return true;
            else if (_patch < other._patch)
                return false;

            if (_build > other._build)
                return true;
            else if (_build < other._build)
                return false;

            return false;
        }

        bool operator < (const cs_firmware_version &other)
        {
            return !(*this >= other);
        }

        bool operator == (const cs_firmware_version &other)
        {
            return
                (_major == other._major) &&
                (_minor == other._minor) &&
                (_patch == other._patch) &&
                (_build == other._build);
        }

        bool operator >= (const cs_firmware_version &other)
        {
            return (*this > other) || (*this == other);
        }

        bool operator <= (const cs_firmware_version &other)
        {
            return (*this < other) || (*this == other);
        }

    private:
        UINT32 _major, _minor, _patch, _build;
    };

    namespace platform {
        class cs_device {
        public:
            explicit cs_device(platform::cs_device_info hwm);
            ~cs_device();

        private:
            void get_addresses_of_frequent_commands(smcs::IDevice& connected_device, RegMap& reg_map);
            bool is_reg_address_cached(const std::string& name, const RegMap& reg_map);

        public:
            power_state set_power_state(power_state state);

            void stream_on(std::function<void(const notification &n)> error_handler, std::vector<cs_stream> streams, std::vector<platform::stream_profile> profiles);

            void probe_and_commit(stream_profile profile, frame_callback callback, cs_stream stream);

            void close(std::vector<cs_stream> streams);

            void image_poll(cs_stream stream, UINT32 channel);

            bool is_metadata_supported();
            bool check_metadata(UINT8* metadata);
            void handle_metadata(metadata_framos_basic& md, UINT8* metadata);
            void clear_metadata(metadata_framos_basic& md);

            power_state get_power_state() const { return _power_state; }

            bool get_pu(rs2_option opt, int32_t &value, cs_stream stream);

            bool set_pu(rs2_option opt, int32_t value, cs_stream stream);

            control_range get_pu_range(rs2_option option, cs_stream stream);

            std::vector <stream_profile> get_profiles(cs_stream stream);

            bool reset(void);

            std::vector<byte> send_hwm(std::vector<byte>& buffer);

            std::string get_device_version();
            std::string get_ip_address();
            std::string get_subnet_mask();

            enum rs2_format get_rgb_format();
            bool is_infrared_supported();
            bool is_infrared_left_color_supported();
            bool is_option_supported(rs2_option opt, cs_stream stream);
            bool is_software_trigger_supported();
            bool is_line_debouncer_time_supported();
            bool is_rgb_raw16_supported();
            bool is_burst_mode_supported();
            bool is_color_genlock_mode_supported();
            bool is_imu_supported();
            int get_imu_id();
            double get_device_timestamp_ms();

            void set_trigger_mode(float mode, cs_stream stream);
            void set_trigger_mode_gs(float mode, cs_stream stream);
            float get_trigger_mode(cs_stream stream);
            float get_trigger_mode_gs(cs_stream stream);
            bool get_intercam_mode(cs_stream stream);
            void update_external_trigger_mode_flag(cs_stream stream, float value);
            std::map < std::string, std::map<std::string, std::string>> get_trigger_properties(cs_stream stream);

            void toggle_calibration_stream_flag(bool toggle) { _is_calibration_stream = toggle; };
            bool is_calibration_stream_toggled(){ return _is_calibration_stream; };
            void toggle_raw16_flag(bool toggle) { _is_raw16 = toggle; };
            std::string get_serial() const;
            smcs::IDevice get_cs_device() const;

        protected:
            void capture_loop(cs_stream stream, UINT32 channel);

            void set_format(stream_profile profile, cs_stream stream);

            std::vector<std::function<void(const notification &n)>> _error_handler;
            std::vector<stream_profile> _profiles;

        private:

            std::string get_cs_param_name(rs2_option option, cs_stream stream);

            bool get_cs_param_min(rs2_option option, int32_t &value, cs_stream stream);

            bool get_cs_param_max(rs2_option option, int32_t &value, cs_stream stream);

            bool get_cs_param_step(rs2_option option, int32_t &value, cs_stream stream);

            bool get_cs_param_default(rs2_option option, int32_t& value, cs_stream stream);

            bool get_cs_param_value(rs2_option option, int32_t &value, cs_stream stream);

            bool set_cs_param(rs2_option option, int32_t value, cs_stream stream);

            int32_t round_cs_param(rs2_option option, int32_t value, cs_stream stream);

            void init_stream(std::function<void(const notification& n)> error_handler, cs_stream stream);

            void deinit_stream(cs_stream stream);

            void start_acquisition(cs_stream stream);

            void stop_acquisition(cs_stream stream);

            bool select_source(cs_stream stream);
            bool set_source_locked(cs_stream stream, bool locked);
            bool set_region(cs_stream stream, bool enable);
            bool disable_source_regions(cs_stream stream);
            bool select_region(cs_stream stream);
            bool select_channel(cs_stream stream);

            INT64 get_stream_source(cs_stream stream);
            INT64 get_stream_region(cs_stream stream);
            bool get_stream_channel(cs_stream stream, UINT32& channel);
            std::vector<cs_stream> get_stream_group(cs_stream stream);

            void stream_params_lock(cs_stream stream);
            void stream_params_unlock(cs_stream stream);

            uint32_t read_from_buffer(std::vector<byte>& buffer, uint32_t index);

            std::vector<byte> send_hwm_to_device(std::vector<byte>& buffer);
            bool send_hwm_cmd(std::vector<byte>& hwm_cmd, bool verify = false);
            bool receive_hwm_cmd(std::vector<byte>& buffer_out);
            void set_rgb_ae_roi(uint32_t top, uint32_t left, uint32_t bottom, uint32_t right);
            bool send_calib_data(std::vector<byte>& hwm_cmd, bool verify = false);

            std::vector<uint32_t> get_frame_rates(); 
            bool get_frame_rates_from_control(std::vector<uint32_t> &frame_rates);

            bool is_profile_format(UINT32 width, UINT32 height, UINT32 format, const stream_profile& profile);

            uint32_t cs_pixel_format_to_native_pixel_format(std::string cs_format); 
            uint32_t native_pixel_format_to_cs_pixel_format(uint32_t native_format);

            std::string ip_address_to_string(uint32_t ip_address);

            INT64 get_optimal_inter_packet_delay(INT64 packet_size_bytes, INT64 link_speed_mbitps);

            std::string get_api_packet_statistics();
            std::string get_dev_packet_statistics();

            static bool inc_device_count_sn(std::string serialNum);
            static bool dec_device_count_sn(std::string serialNum);
            static int get_device_count_sn(std::string serialNum);
            static bool set_device_init_flag_sn(std::string serialNum, bool setInitFlag);
            static bool get_device_init_flag_sn(std::string serialNum);
            static bool set_device_option_sw_trigger_all_flag_sn(std::string serialNum, bool setTriggerAllFlag);
            static bool get_device_option_sw_trigger_all_flag_sn(std::string serialNum);

            bool is_temperature_supported();
            bool is_rgb_led_toggle_supported();

            // members
            std::vector<std::unique_ptr <std::thread>> _threads;
            platform::cs_device_info _device_info;
            power_state _power_state;
            std::vector<std::atomic<bool>> _is_capturing;
            std::mutex _power_lock, _stream_lock, _hwm_lock, _statistics_lock;
            uint8_t _number_of_streams;
            smcs::ICameraAPI _smcs_api;
            smcs::IDevice _connected_device;
            std::unordered_map<cs_stream, UINT32, std::hash<int>> _stream_channels;
            std::vector<frame_callback> _callbacks;
            cs_firmware_version _cs_firmware_version;
            metadata_framos_basic _md;
            uint32_t _md_size;
            enum rs2_format _rgb_pixel_format;
            bool _infrared_supported;
            bool _infrared_left_color_supported;
            bool _temperature_supported_checked;
            bool _temperature_supported;
            bool _software_trigger_supported_checked;
            bool _software_trigger_supported;
            bool _line_debouncer_time_supported_checked;
            bool _line_debouncer_time_supported;
            bool _rgb_raw16_supported_checked;
            bool _rgb_raw16_supported;
            bool _burst_mode_supported_checked;
            bool _burst_mode_supported;
            bool _color_genlock_mode_supported_checked;
            bool _color_genlock_mode_supported;
            bool _is_full_slave_mode;
            bool _imu_supported_checked;
            bool _imu_supported;
            bool _rgb_led_toggle_supported_checked;
            bool _rgb_led_toggle_supported;
            bool _metadata_supported_checked;
            bool _metadata_supported;
            INT64 _selected_source;
            bool _metadata_toggle_value; // MetadataToggle register value keeper, to prevent constant reading from the register
            bool _selected_source_initialized;
            static std::map<std::string, int> _cs_device_num_objects_sn; // serial_number, number of objects per SN (device creation)
            static std::map<std::string, bool> _cs_device_initialized_sn; // serial_number, is device with SN initialized
            static std::map<std::string, bool> _cs_device_option_sw_trigger_all_flag_sn; // serial_number, is device with SN initialized
            size_t _rgb_ae_roi_hwm_size;

            double _timestamp_to_ms_factor;
            double _statistics_last_call_time_device = 0; // used for device statistics logging
            static double _statistics_last_call_time_driver; // used for driver statistics logging, shared between objects

            bool _is_raw16;
            bool _is_calibration_stream;
            //std::once_flag _hwm_tx_once_flag;
            bool _hwm_tx_once_flag;
            UINT64 _hwm_tx_reg_addr;
            UINT64 _hwm_tx_reg_len;
            //std::once_flag _hwm_rx_once_flag;
            bool _hwm_rx_once_flag;
            UINT64 _hwm_rx_reg_addr;
            UINT64 _hwm_rx_reg_len;
            //std::once_flag _calib_tx_once_flag;
            bool _calib_tx_once_flag;
            UINT64 _calib_tx_reg_addr;
            UINT64 _calib_tx_reg_len;

            RegMap _reg_map; //genicam register map, keeps the values of addresses of freqently called registers

        // TODO - nh - gev events (cs-sensor-motion.cpp)
        public:
            void open_event(hid_callback callback);
            void close_event();
            void start_event(const cfg_event_stream_list& config_list, std::function<void(const notification &n)> error_handler);
            void stop_event();
            void new_event(void* dev_event_info);

        protected:
            bool configure_message_channel(smcs::IDevice device);
            bool configure_event(smcs::IDevice device, const cfg_event_stream_list& config_list);
            bool toggle_event_acc(smcs::IDevice device, bool state);
            bool toggle_event_gyro(smcs::IDevice device, bool state);
            void init_event_stream(const cfg_event_stream_list& config_list, std::function<void(const notification& n)> error_handler);
            void deinit_event_stream();
            void capture_event_loop();
            void event_monitor();

            //Framos::CSocketSimple _socket_event;
            std::unique_ptr<std::thread> _thread_event;
            std::atomic<bool> _is_event_capturing;
            std::function<void(const notification &n)> _error_handler_event;
            std::mutex _mtx_callback_event;
            std::condition_variable _cv_callback_event;
            std::deque<cs_event_data> _event_data_list;
            hid_callback _callback_event;
            void* _socket_event;
            std::string _device_serial;
            uint32_t _device_addr;
            uint32_t _socket_event_addr;
            uint32_t _socket_event_port;
            uint32_t _event_acc_id;
            uint32_t _event_gyro_id;
            bool _is_acc_configured;
            bool _is_gyro_configured;
        // !TODO - nh - gev events  (cs-sensor-motion.cpp)
        };
    }

    class cs_sensor : public sensor_base
    {
    public:
        explicit cs_sensor(
            std::string name, 
            std::shared_ptr<platform::cs_device> cs_device,
            std::unique_ptr<frame_timestamp_reader> timestamp_reader, 
            device* dev, 
            cs_stream stream);
        virtual ~cs_sensor() override;

        rs2_extension stream_to_frame_types(rs2_stream stream);

        virtual stream_profiles init_stream_profiles() override;

        void open(const stream_profiles& requests) override;
        void close() override;
        void start(frame_callback_ptr callback) override;
        void stop() override;

        template<class T>
        auto invoke_powered(T action)
        -> decltype(action(*static_cast<platform::cs_device*>(nullptr)))
        {
            power on(std::dynamic_pointer_cast<cs_sensor>(shared_from_this()));
            return action(*_device);
        }

        std::vector<platform::stream_profile> get_configuration() const { return _internal_config; }

        void register_pu(rs2_option id);

        void try_register_pu(rs2_option id);

        void set_inter_cam_sync_mode(float value, bool gs);
        float get_inter_cam_sync_mode(bool gs);
        bool query_inter_cam_sync_mode(bool gs);
        cs_stream get_stream_id(){ return _cs_stream; }

    private:
        void acquire_power();

        void release_power();

        void reset_streaming();

        cs_stream get_stream(rs2_stream type, int index);

        struct power
        {
            explicit power(std::weak_ptr<cs_sensor> owner)
                    : _owner(owner)
            {
                auto strong = _owner.lock();
                if (strong)
                {
                    strong->acquire_power();
                }
            }

            ~power()
            {
                if (auto strong = _owner.lock())
                {
                    try
                    {
                        strong->release_power();
                    }
                    catch (...) {}
                }
            }
        private:
            std::weak_ptr<cs_sensor> _owner;
        };

        std::unique_ptr<frame_timestamp_reader> _timestamp_reader;
        std::atomic<int> _user_count;
        std::mutex _power_lock;
        std::mutex _configure_lock;
        cs_stream _cs_stream;
        std::vector<cs_stream> _cs_selected_streams;
        std::unique_ptr<power> _power;
        std::shared_ptr<platform::cs_device> _device;
        std::atomic<bool> _external_trigger_mode;
    };

    class cs_command_transfer : public platform::command_transfer
    {
    public:
        std::vector<uint8_t> send_receive(const std::vector<uint8_t>& data, int, bool require_response) override;

        cs_command_transfer(std::shared_ptr<platform::cs_device> device)
            : _device(std::move(device))
        {};

    private:
        std::shared_ptr<platform::cs_device> _device;
    };


    
    class cs_hid_sensor : public sensor_base
    {
    public:
        explicit cs_hid_sensor(
            std::shared_ptr<platform::hid_device> hid_device,
            std::shared_ptr<platform::cs_device> cs_device,
            std::unique_ptr<frame_timestamp_reader> hid_iio_timestamp_reader,
            const std::map<rs2_stream, std::map<unsigned, unsigned>>& fps_and_sampling_frequency_per_rs2_stream,
            const std::vector<std::pair<std::string, stream_profile>>& sensor_name_and_hid_profiles,
            device* dev);

        ~cs_hid_sensor() override;

        void open(const stream_profiles& requests) override;
        void close() override;
        void start(frame_callback_ptr callback) override;
        void stop() override;

    protected:
        stream_profiles init_stream_profiles() override;

    private:
        stream_profiles get_sensor_profiles(std::string sensor_name) const;
        const std::string& rs2_stream_to_sensor_name(rs2_stream stream) const;
        uint32_t stream_to_fourcc(rs2_stream stream) const;
        uint32_t fps_to_sampling_frequency(rs2_stream stream, uint32_t fps) const;

    private:
        const std::map<rs2_stream, uint32_t> stream_and_fourcc = 
            {{RS2_STREAM_GYRO,  rs_fourcc('G','Y','R','O')},
             {RS2_STREAM_ACCEL, rs_fourcc('A','C','C','L')}};

        const std::vector<std::pair<std::string, stream_profile>> _sensor_name_and_hid_profiles;
        std::map<rs2_stream, std::map<uint32_t, uint32_t>> _fps_and_sampling_frequency_per_rs2_stream;
        std::map<rs2_stream, std::vector<uint32_t>> _fps_per_rs2_stream;
        std::shared_ptr<platform::hid_device> _hid_device;
        std::mutex _configure_lock;
        std::map<std::string, std::shared_ptr<stream_profile_interface>> _configured_profiles;
        std::vector<bool> _is_configured_stream;
        std::vector<platform::hid_sensor> _hid_sensors;
        std::unique_ptr<frame_timestamp_reader> _hid_iio_timestamp_reader;
        std::shared_ptr<platform::cs_device> _cs_device;
        cfg_event_stream_list _cfg_event_list;
    };
}

#endif //LIBREALSENSE2_CS_SENSOR_H
