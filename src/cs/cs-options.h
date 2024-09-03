// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 FRAMOS GmbH.



#pragma once



#include "cs-private.h"
#include "backend.h"
#include "types.h"
#include "option.h"
#include "../hdr-config.h"



namespace librealsense
{


    class cs_asic_and_projector_temperature_options : public readonly_option
    {
    public:
        float query() const override;

        option_range get_range() const override;

        bool is_enabled() const override;

        const char* get_description() const override;

        explicit cs_asic_and_projector_temperature_options(cs_sensor& ep, rs2_option opt, cs_stream stream);

    private:
        cs_sensor& _ep;
        rs2_option _option;
        cs_stream _stream;
    };



    class cs_depth_scale_option : public option, public observable_option
    {
    public:
        cs_depth_scale_option(hw_monitor& hwm);
        virtual ~cs_depth_scale_option() = default;
        virtual void set(float value) override;
        virtual float query() const override;
        virtual option_range get_range() const override;
        virtual bool is_enabled() const override { return true; }

        const char* get_description() const override
        {
            return "Number of meters represented by a single depth unit";
        }
        void enable_recording(std::function<void(const option &)> record_action)
        {
            _record_action = record_action;
        }

    private:
        cs::depth_table_control get_depth_table(cs::advanced_query_mode mode) const;
        std::function<void(const option &)> _record_action = [](const option&) {};
        lazy<option_range> _range;
        hw_monitor& _hwm;
    };



    class cs_pu_option : public option
    {
    public:
        void set(float value) override;

        float query() const override;

        option_range get_range() const override;

        bool is_enabled() const override
        {
            if (_id == RS2_OPTION_PACKET_SIZE || _id == RS2_OPTION_METADATA_TOGGLE) {
                return !_ep.is_streaming();
            }
            else {
                return true;
            }
        }

        cs_pu_option(cs_sensor& ep, rs2_option id, cs_stream stream)
                : _ep(ep), _id(id), _stream(stream)
        {
        }

        cs_pu_option(cs_sensor& ep, rs2_option id, cs_stream stream, const std::map<float, std::string>& description_per_value)
                : _ep(ep), _id(id), _stream(stream), _description_per_value(description_per_value)
        {
        }

        virtual ~cs_pu_option() = default;

        const char* get_description() const override;

        const char* get_value_description(float val) const override
        {
            if (_description_per_value.find(val) != _description_per_value.end())
                return _description_per_value.at(val).c_str();
            return nullptr;
        }
        void enable_recording(std::function<void(const option &)> record_action) override
        {
            _record = record_action;
        }

    private:
        cs_stream _stream;
        rs2_option _id;
        const std::map<float, std::string> _description_per_value;
        std::function<void(const option &)> _record = [](const option &) {};

    protected:
        cs_sensor& _ep;
    };



    class cs_depth_exposure_option : public cs_pu_option
    {
    public:
        cs_depth_exposure_option(cs_sensor& ep, rs2_option id, cs_stream stream)
                : cs_pu_option(ep, id, stream) {}
        const char* get_description() const override { return "Depth Exposure (usec)"; }
    };



    class cs_external_trigger_option : public cs_pu_option
    {
    public:
        cs_external_trigger_option(cs_sensor& ep, rs2_option id, cs_stream stream)
            : cs_pu_option(ep, id, stream) {}

        cs_external_trigger_option(cs_sensor& ep, rs2_option id, cs_stream stream, const std::map<float, std::string>& description_per_value)
            : cs_pu_option(ep, id, stream, description_per_value), _stream(stream), _id(id)
        {
        }

        option_range get_range() const override { return option_range{ 1, 2, 1, 1 }; }

        const char* get_description() const override { return "External Trigger Source"; }
    private:
        cs_stream _stream;
        rs2_option _id;
    };



    class cs_readonly_option : public cs_pu_option
    {
    public:
        bool is_read_only() const override { return true; }

        void set(float) override
        {
            throw not_implemented_exception("This option is read-only!");
        }

        bool is_enabled() const override
        {
            return _ep.is_streaming();
        }

        void enable_recording(std::function<void(const option &)> record_action) override
        {
            //empty
        }

        explicit cs_readonly_option(cs_sensor& ep, rs2_option id, cs_stream stream)
                : cs_pu_option(ep, id, stream) {}
    };



    class cs_software_trigger_option : public cs_pu_option
    {
    public:
        cs_software_trigger_option(cs_sensor& ep, rs2_option id, cs_stream stream)
            : cs_pu_option(ep, id, stream) {}

        cs_software_trigger_option(cs_sensor& ep, rs2_option id, cs_stream stream, const std::map<float, std::string>& description_per_value)
            : cs_pu_option(ep, id, stream, description_per_value), _stream(stream), _id(id) {}

        option_range get_range() const override { return option_range{ 1,1,1,1 }; }

        const char* get_description() const override { return "Software Trigger"; }
    private:
        cs_stream _stream;
        rs2_option _id;
    };



    class cs_software_trigger_all_option : public cs_pu_option
    {
    public:
        cs_software_trigger_all_option(cs_sensor& ep, rs2_option id, cs_stream stream)
            : cs_pu_option(ep, id, stream), _stream(stream), _id(id) {}

        const char* get_description() const override { return "Forwards software trigger signal to all sensors"; }
    private:
        cs_stream _stream;
        rs2_option _id;
    };



    class cs_user_output_level_option : public cs_pu_option
    {
    public:
        cs_user_output_level_option(cs_sensor& ep, rs2_option id, cs_stream stream)
            : cs_pu_option(ep, id, stream) {}

        cs_user_output_level_option(cs_sensor& ep, rs2_option id, cs_stream stream, const std::map<float, std::string>& description_per_value)
            : cs_pu_option(ep, id, stream, description_per_value), _stream(stream), _id(id) {}

        option_range get_range() const override { return option_range{ 1,2,1,1 }; }

        const char* get_description() const override { return "User Output Level"; }
    private:
        cs_stream _stream;
        rs2_option _id;
    };



    class cs_hdr_option : public option
    {
    public:
        cs_hdr_option(std::shared_ptr<hdr_config> hdr_cfg, rs2_option option, option_range range)
            : _hdr_cfg(hdr_cfg), _option(option), _range(range) {}

        cs_hdr_option(std::shared_ptr<hdr_config> hdr_cfg, rs2_option option, option_range range, const std::map<float, std::string>& description_per_value)
            : _hdr_cfg(hdr_cfg), _option(option), _range(range), _description_per_value(description_per_value) {}

        virtual ~cs_hdr_option() = default;
        virtual void set(float value) override;
        virtual float query() const override;
        virtual option_range get_range() const override;
        virtual bool is_enabled() const override { return true; }
        virtual const char* get_description() const override { return "HDR Option"; }
        virtual void enable_recording(std::function<void(const option&)> record_action) override { _record_action = record_action; }
        virtual const char* get_value_description(float) const override;

    private:
        std::function<void(const option&)> _record_action = [](const option&) {};
        std::shared_ptr<hdr_config> _hdr_cfg;
        rs2_option _option;
        option_range _range;
        const std::map<float, std::string> _description_per_value;
    };



    // used for options that change their behavior when hdr configuration is in process
    class cs_hdr_conditional_option : public option
    {
    public:
        cs_hdr_conditional_option(std::shared_ptr<hdr_config> hdr_cfg,
            std::shared_ptr<option> cs_option,
            std::shared_ptr<option> hdr_option) :
            _hdr_cfg(hdr_cfg),
            _cs_option(cs_option),
            _hdr_option(hdr_option) {}

        virtual ~cs_hdr_conditional_option() = default;
        virtual void set(float value) override;
        virtual float query() const override;
        virtual option_range get_range() const override;
        virtual bool is_enabled() const override;
        virtual const char* get_description() const override;
        virtual void enable_recording(std::function<void(const option&)> record_action) override { _record_action = record_action; }

    private:
        std::function<void(const option&)> _record_action = [](const option&) {};
        std::shared_ptr<hdr_config> _hdr_cfg;
        std::shared_ptr<option> _cs_option;
        std::shared_ptr<option> _hdr_option;
    };



    class cs_limits_option;

    class cs_auto_exposure_limit_option : public option_base
    {
    public:
        cs_auto_exposure_limit_option(hw_monitor& hwm, sensor_base* depth_ep, option_range range, std::shared_ptr<cs_limits_option> exposure_limit_enable);
        virtual ~cs_auto_exposure_limit_option() = default;
        virtual void set(float value) override;
        virtual float query() const override;
        virtual option_range get_range() const override;
        virtual bool is_enabled() const override { return true; }
        virtual bool is_read_only() const { return _sensor && _sensor->is_opened(); }
        virtual const char* get_description() const override
        {
            return "Exposure limit is in microseconds. Default is 0 which means full exposure range. If the requested exposure limit is greater than frame time, it will be set to frame time at runtime. Setting will not take effect until next streaming session.";
        }
        virtual void enable_recording(std::function<void(const option&)> record_action) override { _record_action = record_action; }

    private:
        std::function<void(const option&)> _record_action = [](const option&) {};
        lazy<option_range> _range;
        hw_monitor& _hwm;
        sensor_base* _sensor;
        std::weak_ptr<cs_limits_option> _exposure_limit_toggle;
    };



    class cs_auto_gain_limit_option : public option_base
    {
    public:
        cs_auto_gain_limit_option(hw_monitor& hwm, sensor_base* depth_ep, option_range range, std::shared_ptr<cs_limits_option> gain_limit_enable);
        virtual ~cs_auto_gain_limit_option() = default;
        virtual void set(float value) override;
        virtual float query() const override;
        virtual option_range get_range() const override;
        virtual bool is_enabled() const override { return true; }
        virtual bool is_read_only() const { return _sensor && _sensor->is_opened(); }
        virtual const char* get_description() const override
        {
            return "Gain limits ranges from 16 to 248. Default is 0 which means full gain. If the requested gain limit is less than 16, it will be set to 16. If the requested gain limit is greater than 248, it will be set to 248. Setting will not take effect until next streaming session.";
        }
        virtual void enable_recording(std::function<void(const option&)> record_action) override { _record_action = record_action; }

    private:
        std::function<void(const option&)> _record_action = [](const option&) {};
        lazy<option_range> _range;
        hw_monitor& _hwm;
        sensor_base* _sensor;
        std::weak_ptr<cs_limits_option> _gain_limit_toggle;
    };



    // Auto-Limits Enable/ Disable
    class cs_limits_option : public option
    {
    public:

        cs_limits_option(rs2_option option, option_range range, const char* description, hw_monitor& hwm, cs_sensor& ep) :
            _option(option), _toggle_range(range), _description(description), _hwm(hwm), _ep(ep) {};

        virtual void set(float value) override
        {
            auto set_limit = _cached_limit;
            if (value == 0) // 0: gain auto-limit is disabled, 1 : gain auto-limit is enabled (all range 16-248 is valid)
                set_limit = 0;

            command cmd_get(cs::AUTO_CALIB);
            cmd_get.param1 = 5;
            std::vector<uint8_t> ret = _hwm.send(cmd_get);
            if (ret.empty())
                throw invalid_value_exception("auto_exposure_limit_option::query result is empty!");

            command cmd(cs::AUTO_CALIB);
            cmd.param1 = 4;
            cmd.param2 = *(reinterpret_cast<uint32_t*>(ret.data()));
            cmd.param3 = static_cast<int>(set_limit);
            if (_option == RS2_OPTION_AUTO_EXPOSURE_LIMIT_TOGGLE)
            {
                cmd.param2 = static_cast<int>(set_limit);
                cmd.param3 = *(reinterpret_cast<uint32_t*>(ret.data() + 4));
            }
            _hwm.send(cmd);
        };

        virtual float query() const override
        {
            auto offset = 0;
            if (_option == RS2_OPTION_AUTO_GAIN_LIMIT_TOGGLE)
                offset = 4;
            command cmd(cs::AUTO_CALIB);
            cmd.param1 = 5;
            auto res = _hwm.send(cmd);
            if (res.empty())
                throw invalid_value_exception("auto_exposure_limit_option::query result is empty!");

            auto limit_val = static_cast<float>(*(reinterpret_cast<uint32_t*>(res.data() + offset)));
            if (limit_val == 0)
                return 0;
            return 1;
        };

        virtual option_range get_range() const override { return _toggle_range; };
        virtual bool is_enabled() const override { return !_ep.is_streaming(); }
        virtual const char* get_description() const override { return _description; };
        virtual void enable_recording(std::function<void(const option&)> record_action) override { _record_action = record_action; }
        virtual const char* get_value_description(float val) const override
        {
            if (_description_per_value.find(val) != _description_per_value.end())
                return _description_per_value.at(val).c_str();
            return nullptr;
        };
        void set_cached_limit(float value) { _cached_limit = value; };
        float get_cached_limit() { return _cached_limit; };

    private:
        std::function<void(const option&)> _record_action = [](const option&) {};
        rs2_option _option;
        option_range _toggle_range;
        const std::map<float, std::string> _description_per_value;
        float _cached_limit;         // used to cache contol limit value when toggle is switched to off
        const char* _description;
        hw_monitor& _hwm;
        cs_sensor& _ep;
    };



}