// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 FRAMOS GmbH.

#include "cs-options.h"

namespace librealsense
{
    void cs_pu_option::set(float value)
    {
        _ep.invoke_powered(
                [this, value](platform::cs_device& dev)
                {
                    if (!dev.set_pu(_id, static_cast<int32_t>(value), _stream))
                        throw invalid_value_exception(to_string() << "get_pu(id=" << std::to_string(_id) << ") failed!" << " Last Error: " << strerror(errno));

                    _record(*this);
                });
    }

    float cs_pu_option::query() const
    {
        return static_cast<float>(_ep.invoke_powered(
                [this](platform::cs_device& dev)
                {
                    int32_t value = 0;
                    if (!dev.get_pu(_id, value, _stream))
                        throw invalid_value_exception(to_string() << "get_pu(id=" << std::to_string(_id) << ") failed!" << " Last Error: " << strerror(errno));
                    return static_cast<float>(value);
                }));
    }

    option_range cs_pu_option::get_range() const
    {
        auto cs_range = _ep.invoke_powered(
                [this](platform::cs_device& dev)
                {
                    return dev.get_pu_range(_id, _stream);
                });

        if (cs_range.min.size() < sizeof(int32_t)) return option_range{0,0,1,0};

        auto min = *(reinterpret_cast<int32_t*>(cs_range.min.data()));
        auto max = *(reinterpret_cast<int32_t*>(cs_range.max.data()));
        auto step = *(reinterpret_cast<int32_t*>(cs_range.step.data()));
        auto def = *(reinterpret_cast<int32_t*>(cs_range.def.data()));
        return option_range{static_cast<float>(min),
                            static_cast<float>(max),
                            static_cast<float>(step),
                            static_cast<float>(def)};
    }

    const char* cs_pu_option::get_description() const
    {
        switch(_id)
        {
            case RS2_OPTION_BACKLIGHT_COMPENSATION: return "Enable / disable backlight compensation";
            case RS2_OPTION_RGB_LED_TOGGLE: return "Toggles the RGB camera LED light";
            case RS2_OPTION_METADATA_TOGGLE: return "Toggles the Metadata";
            case RS2_OPTION_BRIGHTNESS: return "Image brightness";
            case RS2_OPTION_CONTRAST: return "Image contrast";
            case RS2_OPTION_EXPOSURE: return "Controls exposure time of color camera. Setting any value will disable auto exposure";
            case RS2_OPTION_GAIN: return "Image gain";
            case RS2_OPTION_GAMMA: return "Image gamma setting";
            case RS2_OPTION_HUE: return "Image hue";
            case RS2_OPTION_SATURATION: return "Image saturation setting";
            case RS2_OPTION_SHARPNESS: return "Image sharpness setting";
            case RS2_OPTION_WHITE_BALANCE: return "Controls white balance of color image. Setting any value will disable auto white balance";
            case RS2_OPTION_ENABLE_AUTO_EXPOSURE: return "Enable / disable auto-exposure";
            case RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE: return "Enable / disable auto-white-balance";
            case RS2_OPTION_POWER_LINE_FREQUENCY: return "Power Line Frequency";
            case RS2_OPTION_AUTO_EXPOSURE_PRIORITY: return "Limit exposure time when auto-exposure is ON to preserve constant fps rate";
            case RS2_OPTION_INTER_PACKET_DELAY: return "Inter-packet delay";
            case RS2_OPTION_PACKET_SIZE: return "Packet size";
            case RS2_OPTION_ASIC_TEMPERATURE: return "Current Asic Temperature (degree celsius)";
            case RS2_OPTION_PROJECTOR_TEMPERATURE: return "Current Projector Temperature (degree celsius)";
            case RS2_OPTION_LINE_DEBOUNCER_TIME: return "Line Debouncer time (microseconds)";
            default: return _ep.get_option_name(_id);
        }
    }

    cs_asic_and_projector_temperature_options::cs_asic_and_projector_temperature_options(cs_sensor& ep, rs2_option opt, cs_stream stream)
            : _option(opt), _ep(ep), _stream(stream)
    {}

    const char* cs_asic_and_projector_temperature_options::get_description() const
    {
        switch (_option)
        {
            case RS2_OPTION_ASIC_TEMPERATURE:
                return "Current Asic Temperature (degree celsius)";
            case RS2_OPTION_PROJECTOR_TEMPERATURE:
                return "Current Projector Temperature (degree celsius)";
            default:
                throw invalid_value_exception(to_string() << _ep.get_option_name(_option) << " is not temperature option!");
        }
    }

    bool cs_asic_and_projector_temperature_options::is_enabled() const
    {
        return _ep.is_streaming();
    }

    option_range cs_asic_and_projector_temperature_options::get_range() const
    {
        return option_range { -40, 125, 0, 0 };
    }

    float cs_asic_and_projector_temperature_options::query() const
    {
        if (!is_enabled())
            throw wrong_api_call_sequence_exception("query option is allow only in streaming!");

        #pragma pack(push, 1)
        struct temperature
        {
            uint8_t is_valid;
            int32_t value;
        };
        #pragma pack(pop)

        auto temperature_data = static_cast<temperature>( _ep.invoke_powered(
                [this](platform::cs_device& dev)
                {
                    temperature temp{};
                    // if temperature read from camera is in good range, data is valid, otherwise set error value and flag
                    if (dev.get_pu(
                        _option, temp.value, _stream) &&
                        temp.value >= get_range().min &&
                        temp.value <= get_range().max) {
                        temp.is_valid = 1;
                    }
                    else {
                        temp.is_valid = 0;
                        temp.value = -2147483648;
                    }
                    return temp;
                }));

        int32_t temperature::* field;
        uint8_t temperature::* is_valid_field;

        switch (_option)
        {
            case RS2_OPTION_PROJECTOR_TEMPERATURE:
            case RS2_OPTION_ASIC_TEMPERATURE:
                field = &temperature::value;
                is_valid_field = &temperature::is_valid;
                break;
            default:
                throw invalid_value_exception(to_string() << _ep.get_option_name(_option) << " is not temperature option!");
        }

        if (0 == temperature_data.*is_valid_field)
            LOG_ERROR(_ep.get_option_name(_option) << " value is not valid!");

        return temperature_data.*field;
    }

    cs::depth_table_control cs_depth_scale_option::get_depth_table(cs::advanced_query_mode mode) const
    {
        command cmd(cs::GET_ADV);
        cmd.param1 = cs::etDepthTableControl;
        cmd.param2 = mode;
        auto res = _hwm.send(cmd);

        if (res.size() < sizeof(cs::depth_table_control))
            throw std::runtime_error("Not enough bytes returned from the firmware!");

        auto table = (const cs::depth_table_control*)res.data();
        return *table;
    }

    cs_depth_scale_option::cs_depth_scale_option(hw_monitor& hwm)
            : _hwm(hwm)
    {
        _range = [this]()
        {
            auto min = get_depth_table(cs::GET_MIN);
            auto max = get_depth_table(cs::GET_MAX);
            return option_range{ (float)(0.000001 * min.depth_units),
                                 (float)(0.000001 * max.depth_units),
                                 0.000001f, 0.001f };
        };
    }

    void cs_depth_scale_option::set(float value)
    {
        command cmd(cs::SET_ADV);
        cmd.param1 = cs::etDepthTableControl;

        auto depth_table = get_depth_table(cs::GET_VAL);
        depth_table.depth_units = static_cast<uint32_t>(1000000 * value);
        auto ptr = (uint8_t*)(&depth_table);
        cmd.data = std::vector<uint8_t>(ptr, ptr + sizeof(cs::depth_table_control));

        _hwm.send(cmd);
        _record_action(*this);
        notify(value);
    }

    float cs_depth_scale_option::query() const
    {
        auto table = get_depth_table(cs::GET_VAL);
        return (float)(0.000001 * (float)table.depth_units);
    }

    option_range cs_depth_scale_option::get_range() const
    {
        return *_range;
    }

    void cs_hdr_option::set(float value)
    {
        _hdr_cfg->set(_option, value, _range);
        _record_action(*this);
    }

    float cs_hdr_option::query() const
    {
        return _hdr_cfg->get(_option);
    }

    option_range cs_hdr_option::get_range() const
    {
        return _range;
    }

    const char* cs_hdr_option::get_value_description(float val) const
    {
        if (_description_per_value.find(val) != _description_per_value.end())
            return _description_per_value.at(val).c_str();
        return nullptr;
    }

    void cs_hdr_conditional_option::set(float value)
    {
        if (_hdr_cfg->is_config_in_process())
            _hdr_option->set(value);
        else
        {
            if (_hdr_cfg->is_enabled())
                LOG_WARNING("The control - " << _cs_option->get_description()
                    << " - is locked while HDR mode is active.\n");
            else
                _cs_option->set(value);
        }
    }

    float cs_hdr_conditional_option::query() const
    {
        if (_hdr_cfg->is_config_in_process())
            return _hdr_option->query();
        else
            return _cs_option->query();
    }

    option_range cs_hdr_conditional_option::get_range() const
    {
        if (_hdr_cfg->is_config_in_process())
            return _hdr_option->get_range();
        else
            return _cs_option->get_range();
    }

    const char* cs_hdr_conditional_option::get_description() const
    {
        if (_hdr_cfg->is_config_in_process())
            return _hdr_option->get_description();
        else
            return _cs_option->get_description();
    }

    bool cs_hdr_conditional_option::is_enabled() const
    {
        if (_hdr_cfg->is_config_in_process())
            return _hdr_option->is_enabled();
        else
            return _cs_option->is_enabled();
    }

    cs_auto_exposure_limit_option::cs_auto_exposure_limit_option(hw_monitor& hwm, sensor_base* ep, option_range range, std::shared_ptr<cs_limits_option> exposure_limit_enable)
        : option_base(range), _hwm(hwm), _sensor(ep), _exposure_limit_toggle(exposure_limit_enable)
    {
        _range = [range]()
        {
            return range;
        };
        if (auto toggle = _exposure_limit_toggle.lock())
            toggle->set_cached_limit(range.max);
    }

    void cs_auto_exposure_limit_option::set(float value)
    {
        if (!is_valid(value))
            throw invalid_value_exception("set(enable_auto_exposure) failed! Invalid Auto-Exposure mode request " + std::to_string(value));

        if (auto toggle = _exposure_limit_toggle.lock())
        {
            toggle->set_cached_limit(value);
            if (toggle->query() == 0.f)
                toggle->set(1);
        }

        command cmd_get(cs::AUTO_CALIB);
        cmd_get.param1 = 5;
        std::vector<uint8_t> ret = _hwm.send(cmd_get);
        if (ret.empty())
            throw invalid_value_exception("auto_exposure_limit_option::query result is empty!");

        command cmd(cs::AUTO_CALIB);
        cmd.param1 = 4;
        cmd.param2 = static_cast<int>(value);
        cmd.param3 = *(reinterpret_cast<uint32_t*>(ret.data() + 4));
        _hwm.send(cmd);
        _record_action(*this);
    }

    float cs_auto_exposure_limit_option::query() const
    {
        command cmd(cs::AUTO_CALIB);
        cmd.param1 = 5;

        auto res = _hwm.send(cmd);
        if (res.empty())
            throw invalid_value_exception("auto_exposure_limit_option::query result is empty!");

        auto ret = static_cast<float>(*(reinterpret_cast<uint32_t*>(res.data())));
        if (ret< get_range().min || ret > get_range().max)
        {
            if (auto toggle = _exposure_limit_toggle.lock())
                return toggle->get_cached_limit();
        }
        return ret;
    }

    option_range cs_auto_exposure_limit_option::get_range() const
    {
        return *_range;
    }

    cs_auto_gain_limit_option::cs_auto_gain_limit_option(hw_monitor& hwm, sensor_base* ep, option_range range, std::shared_ptr <cs_limits_option> gain_limit_enable)
        : option_base(range), _hwm(hwm), _sensor(ep), _gain_limit_toggle(gain_limit_enable)
    {
        _range = [range]()
        {
            return range;
        };
        if (auto toggle = _gain_limit_toggle.lock())
            toggle->set_cached_limit(range.max);
    }

    void cs_auto_gain_limit_option::set(float value)
    {
        if (!is_valid(value))
            throw invalid_value_exception("set(enable_auto_gain) failed! Invalid Auto-Gain mode request " + std::to_string(value));

        if (auto toggle = _gain_limit_toggle.lock())
        {
            toggle->set_cached_limit(value);
            if (toggle->query() == 0.f)
                toggle->set(1);
        }


        command cmd_get(cs::AUTO_CALIB);
        cmd_get.param1 = 5;
        std::vector<uint8_t> ret = _hwm.send(cmd_get);
        if (ret.empty())
            throw invalid_value_exception("auto_exposure_limit_option::query result is empty!");

        command cmd(cs::AUTO_CALIB);
        cmd.param1 = 4;
        cmd.param2 = *(reinterpret_cast<uint32_t*>(ret.data()));
        cmd.param3 = static_cast<int>(value);
        _hwm.send(cmd);
        _record_action(*this);
    }

    float cs_auto_gain_limit_option::query() const
    {
        command cmd(cs::AUTO_CALIB);
        cmd.param1 = 5;

        auto res = _hwm.send(cmd);
        if (res.empty())
            throw invalid_value_exception("auto_exposure_limit_option::query result is empty!");

        auto ret = static_cast<float>(*(reinterpret_cast<uint32_t*>(res.data() + 4)));
        if (ret< get_range().min || ret > get_range().max)
        {
            if (auto toggle = _gain_limit_toggle.lock())
                return toggle->get_cached_limit();
        }
        return ret;
    }

    option_range cs_auto_gain_limit_option::get_range() const
    {
        return *_range;
    }
}