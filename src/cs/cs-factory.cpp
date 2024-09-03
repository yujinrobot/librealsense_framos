// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 FRAMOS GmbH.

#include "cs/cs-factory.h"
#include "d400e.h"
#include "types.h"
#include <string>

namespace librealsense {

    std::shared_ptr <device_interface> cs_info::create(std::shared_ptr <context> ctx,
                                                       bool register_device_notifications) const 
    {
        switch (get_camera_model(_hwm.id)) {
            case CS_D435E:
            case CS_D415E:
            case CS_D455E:
                return std::make_shared<d400e_camera>(ctx, this->get_device_data(),
                                                      register_device_notifications);
            default:
                throw std::runtime_error(to_string() << "Unsupported CS model! 0x"
                                                     << std::hex << std::setw(4) << std::setfill('0') << _hwm.id);
        }
    }

    cs_camera_model cs_info::get_camera_model(std::string pid) 
    {
        if (pid.compare(CS_CAMERA_MODEL_D435e) == 0) return CS_D435E;
        else if (pid.compare(CS_CAMERA_MODEL_D415e) == 0) return CS_D415E;
        else if (pid.compare(CS_CAMERA_MODEL_D455e) == 0) return CS_D455E;
        else return CS_UNDEFINED;
    }

    bool cs_info::is_timestamp_supported(std::string pid) 
    {
        switch (get_camera_model(pid)) {
            case CS_D435E:
            case CS_D415E:
            case CS_D455E:
                return true;
            default:
                return false;
        }
    }

    std::vector <std::shared_ptr<device_info>> cs_info::pick_cs_devices(
            std::shared_ptr <context> ctx,
            std::vector <platform::cs_device_info> &cs) 
    {
        std::vector <std::shared_ptr<device_info>> results;

        for (auto &group : cs) {
            auto info = std::make_shared<cs_info>(ctx, group);
            results.push_back(info);
        }

        return results;
    }

    std::vector<platform::cs_device_info> cs_info::query_cs_devices() 
    {
        auto& cs_watcher = cs_device_watcher::get_cs_device_watcher();
        return cs_watcher.find_cs_devices(0.15);
    }

    cs_device_watcher::cs_device_watcher() 
    {
        _is_dev_filter_req = false;

        std::call_once(once_flag,
        [this]() { 
            device_filter_list_setup(_dev_filter_holder);
        });
    }

    platform::cs_device_info cs_device_watcher::get_cs_device_info(smcs::IDevice device)
    {
        auto info = platform::cs_device_info();
        info.serial = device->GetSerialNumber();
        info.id = device->GetModelName();
        info.info = device->GetManufacturerSpecificInfo();
        if (serial_connection_ids_.find(info.serial) == serial_connection_ids_.end())
            serial_connection_ids_[info.serial] = 0;
        info.connection_id = serial_connection_ids_[info.serial]; 
        return info;
    }

    void cs_device_watcher::filter_out_devices(smcs::DevicesList& device_list, const cs_dev_filter_holder& dev_filter_holder, bool is_dev_filter_req)
    {
        if (dev_filter_holder.is_ena()) {
            if (dev_filter_holder.is_valid()) {

                bool is_deny = false;
                bool is_allow = false;

                // iterate through device_list
                smcs::DevicesList::const_iterator it_dev = device_list.begin();
                while (it_dev != device_list.end()) {
                    
                    // handle 'deny' list (higher priority) -> DEPRECATED!
                    is_deny = handle_deny_list((*it_dev), dev_filter_holder.deny);
                    if (is_deny) {
                        it_dev = device_list.erase(it_dev);
                        continue;
                    }
                    
                    // handle 'allow' list
                    is_allow = handle_allow_list((*it_dev), dev_filter_holder.allow);
                    if (is_allow) {
                        it_dev++;
                        continue;
                    }

                    // device will be removed when filter file is used and neither 'deny' or 'allow' list contains information about it
                    if ((is_deny == false) && (is_allow == false)) {
                        it_dev = device_list.erase(it_dev);
                        continue;
                    }

                    // we should not be here (continue anyway to complete the iteration)
                    it_dev++;
                }
            }
        }

        // remove all devices if user required device filtering in certain cases
        if (is_dev_filter_req) {

            // remove all devices in case if dev_filter_holder 'allow' list is empty
            if ((dev_filter_holder.allow._filter_serial_range.size() == 0) &&
                (dev_filter_holder.allow._filter_ip_range.size() == 0)) {
                device_list.clear();
            }
            // remove all devices in case if dev_filter_holder is invalid
            else if (dev_filter_holder.is_valid() == false) {
                device_list.clear();
            }
        }
    }

    bool cs_device_watcher::handle_deny_list(smcs::IDevice device, const cs_dev_filter& dev_filter_deny) const
    {
        bool status = false;

        // TODO NH -> DEPRECATED!

        return status;
    }

    bool cs_device_watcher::handle_allow_list(smcs::IDevice device, const cs_dev_filter& dev_filter_allow) const
    {
        bool status = false;

        uint64_t dev_serial = device->GetMacAddress();
        uint32_t dev_ip = device->GetIpAddress();

        // check serial range list
        const_it_filter_serial_range it = dev_filter_allow._filter_serial_range.begin();
        while (it != dev_filter_allow._filter_serial_range.end()) {

            if ((dev_serial >= ((*it).serial_begin)) && (dev_serial <= ((*it).serial_end))) {
                status = true;
                break;
            }

            it++;
        }

        // check ip range list if not in serial range list
        if (status == false) {

            const_it_filter_ip_range it = dev_filter_allow._filter_ip_range.begin();
            while (it != dev_filter_allow._filter_ip_range.end()) {

                if ((dev_ip >= (*it).ip_begin) && (dev_ip <= (*it).ip_end)) {
                    status = true;
                    break;
                }

                it++;
            }
        }

        return status;
    }

    void cs_device_watcher::device_filter_list_setup(cs_dev_filter_holder& dev_filter_holder)
    {
        dev_filter_holder.allow._is_valid_on_init = false;
        dev_filter_holder.allow._is_valid = false;
        dev_filter_holder.deny._is_valid_on_init = false;
        dev_filter_holder.deny._is_valid = false;

        LOG_INFO("FRAMOS - Checking device filtering request.");

        _is_dev_filter_req = false;

        // include the filter list that a user may have forwarded by hard-coding it
        const std::vector<rs2_d400e_filter>& filter_list = d400e::filter_list::get_instance().get_filter_list();
        if (filter_list.size() > 0) {

            // TODO NH - make some validation before merging the list into the main list

            LOG_INFO("FRAMOS - Detected device filtering request with hard-coded filter list.");

            _is_dev_filter_req = true;

            for (auto elem : filter_list) {

                // handle serial_range entry's
                if ((elem.serial_range.begin != 0) &&
                    (elem.serial_range.end != 0)) {

                    cs_dev_filter::filter_serial_range fsr;
                    fsr.serial_begin = elem.serial_range.begin;
                    fsr.serial_end = elem.serial_range.end;
                    fsr.allowed = true;

                    dev_filter_holder.allow._filter_serial_range.push_back(fsr);

                    // skip ip_range checking, serial_range have priority over ip_range
                    continue;
                }

                // handle ip_range entry's
                if ((elem.ip_range.begin != 0) &&
                    (elem.ip_range.end != 0)) {

                    cs_dev_filter::filter_ip_range fir;
                    fir.ip_begin = elem.ip_range.begin;
                    fir.ip_end = elem.ip_range.end;
                    fir.allowed = true;

                    dev_filter_holder.allow._filter_ip_range.push_back(fir);
                }
            }
        }

        // verify command line arguments
        std::string full_file_path = d400e::cli_args::get_instance().get_filter_file_path();
        if (!full_file_path.empty()) {

            LOG_INFO("FRAMOS - Detected device filtering request via filtering file.");

            _is_dev_filter_req = true;

            parse_filter_file(full_file_path, dev_filter_holder);
        }

        if (_is_dev_filter_req) {
            if (!validate_filter_list_on_init(dev_filter_holder)) {
                LOG_ERROR("FRAMOS - Device filter list verification failed.");
                dev_filter_holder.allow._filter_serial_range.clear();
                dev_filter_holder.allow._filter_ip_range.clear();
            }
        }
        else {
            LOG_INFO("FRAMOS - No device filtering request detected. All FRAMOS devices will be available.");
        }
    }

    bool cs_device_watcher::validate_filter_list_on_init(cs_dev_filter_holder& dev_filter_holder)
    {
        bool r_status = false;
        bool serial_status = true;
        bool ip_status = true;

        // check serial range list
        const_it_filter_serial_range it = dev_filter_holder.allow._filter_serial_range.begin();
        while (it != dev_filter_holder.allow._filter_serial_range.end()) {

            if (((*it).serial_begin) > ((*it).serial_end)) {
                LOG_ERROR("FRAMOS - Invalid range in device filtering list [\"serial_range\"].");
                serial_status = false;
                break;
            }

            it++;
        }

        // check ip range list if serial range list is ok
        if (serial_status == true) {

            const_it_filter_ip_range it = dev_filter_holder.allow._filter_ip_range.begin();
            while (it != dev_filter_holder.allow._filter_ip_range.end()) {

                if (((*it).ip_begin) > ((*it).ip_end)) {
                    LOG_ERROR("FRAMOS - Invalid range in device filtering list [\"ip_range\"].");
                    ip_status = false;
                    break;
                }

                it++;
            }
        }

        if ((dev_filter_holder.allow._filter_serial_range.size() == 0) &&
            (dev_filter_holder.allow._filter_ip_range.size() == 0)) {
            serial_status = false;
            ip_status = false;
        }

        if ((dev_filter_holder.allow._filter_serial_range.size() == 0) &&
            (dev_filter_holder.allow._filter_ip_range.size() == 0)) {
            LOG_ERROR("FRAMOS - Device filtering list is empty.");
        }

        r_status = serial_status & ip_status;
        dev_filter_holder.allow._is_valid_on_init = r_status;

        return r_status;
    }

    bool cs_device_watcher::validate_filter_list(cs_dev_filter_holder& dev_filter_holder, const smcs::DevicesList& device_list)
    {
        bool status = false;

        // TODO  NH
        status = true;
        dev_filter_holder.allow._is_valid = true;
        dev_filter_holder.deny._is_valid = true;

        return status;
    }

    bool cs_device_watcher::verify_serial(const std::string& serial)
    {
        bool status = false;

        if (serial.size() == CS_DEV_SERIAL_SYMBOL_COUNT) {
            for (size_t pos = 0; pos < serial.size(); pos++) {
                const char ch = serial.at(pos);
                if (((ch >= CS_DEV_SERIAL_VALID_DIGIT_START) && (ch <= CS_DEV_SERIAL_VALID_DIGIT_END)) ||
                    ((ch >= CS_DEV_SERIAL_VALID_CHAR_START) && (ch <= CS_DEV_SERIAL_VALID_CHAR_END)) ||
                    ((ch >= CS_DEV_SERIAL_VALID_CHAR_CAPS_START) && (ch <= CS_DEV_SERIAL_VALID_CHAR_CAPS_END)) ||
                    (ch == CS_DEV_SERIAL_COLON_SYMBOL)) {
                    status = true;
                    continue;
                }
                else {
                    LOG_ERROR("FRAMOS - Invalid serial number format in device filtering file [unsupported character].");
                    status = false;
                    break;
                }
            }
        }
        else {
            LOG_ERROR("FRAMOS - Invalid serial number format in device filtering file [character count].");
        }

        return status;
    }

    uint64_t cs_device_watcher::serial_from_string(const std::string& serial, bool revert)
    {
        const uint8_t octet_count = 6;
        uint64_t output = 0;
        uint32_t octet[octet_count] = { 0 };

        // ex. 6C:D4:46:00:00:00 == 17
        if (serial.size() == 17) {
            
            for (uint8_t i = 0; i < octet_count; i++) {
                try {
                    octet[i] = std::stoul((serial.substr((i * 3), 2)).c_str(), nullptr, 16);
                    int a = 0;
                    int b = a;
                }
                catch (std::invalid_argument& e) {
                    LOG_ERROR("FRAMOS - Device filtering: " << e.what());
                }
                catch (std::out_of_range& e) {
                    LOG_ERROR("FRAMOS - Device filtering: " << e.what());
                }
            }

            int8_t index = 0;
            for (int8_t i = 40; i >= 0; i -= 8) {
                output |= ((uint64_t)octet[index++]) << i;
            }
        }
        else {
            LOG_ERROR("FRAMOS - Invalid serial number format in device filtering file [invalid serial number size].");
        }

        return output;
    }

    void cs_device_watcher::parse_filter_file(const std::string& full_file_path, cs_dev_filter_holder& dev_filter_holder)
    {
        // Code bellow can parse following json objects defined in the input file (certain object can be specified multiple times)

        /*
        // FILTER - Serial
        {
            "serial": "6C:D1:46:03:01:03",
            "allow" : true
        }
        
        // FILTER - Serial range
        {
            "serial_range":
            {
                "begin": "6C:D1:46:03:01:02",
                "end": "6C:D1:46:03:01:0A"
            },
            "allow" : true
        }

        // FILTER - IP
        {
            "ip": [192, 168, 2, 125],
            "allow" : true
        }

        // FILTER - IP range
        {
            "ip_range":
            {
                "begin": [192, 168, 2, 100],
                "end": [192, 168, 2, 120]
            },
            "allow" : true
        }
        */

        std::ifstream file(full_file_path);
        if (file.is_open()) {

            try {

                json js_file = json::parse(file);

                // traverse through json objects
                json::const_iterator it = js_file.begin();
                while (it != js_file.end()) {

                    if ((*it).is_object()) {

                        bool is_valid = false;
                        bool is_allowed = false;
                        std::string object_name = "";
                        json js_object = json::parse((*it).dump());

                        // TODO NH - remove this if "allow" tag returns to the json filter file
                        is_valid = true;
                        is_allowed = true;

                        json::const_iterator it_object = js_object.begin();
                        while (it_object != js_object.end()) {

                            object_name = it_object.key();

                            // FILTER - IP
                            if (object_name == CS_JSON_OBJ_IP) {

                                if (is_valid) {
                                    parse_ip_filter(it_object, is_allowed, dev_filter_holder);
                                }
                                else {
                                    // TODO NH - Log error!
                                }
                            }
                            // FILTER - IP range
                            else if (object_name == CS_JSON_OBJ_IP_RANGE) {

                                if (is_valid) {
                                    parse_ip_range_filter(it_object, is_allowed, dev_filter_holder);
                                }
                                else {
                                    // TODO NH - Log error!
                                }
                            }
                            // FILTER - Serial
                            else if (object_name == CS_JSON_OBJ_SERIAL) {

                                if (is_valid) {
                                    parse_serial_filter(it_object, is_allowed, dev_filter_holder);
                                }
                                else {
                                    // TODO NH - Log error!
                                }
                            }
                            // FILTER - Serial range
                            else if (object_name == CS_JSON_OBJ_SERIAL_RANGE) {

                                if (is_valid) {
                                    parse_serial_range_filter(it_object, is_allowed, dev_filter_holder);
                                }
                                else {
                                    // TODO NH - Log error!
                                }
                            }
                            // FILTER - Subnet
                            else if (object_name == CS_JSON_OBJ_SUBNET) {

                                if (is_valid) {
                                    parse_subnet_filter(it_object, is_allowed, dev_filter_holder);
                                }
                                else {
                                    // TODO NH - Log error!
                                }
                            }
                            // FILTER "allowed" flag
                            else if (object_name == CS_JSON_FLAG_ALLOW) {
                                if ((*it_object).is_boolean()) {
                                    is_allowed = (*it_object);
                                    is_valid = true;
                                }
                            }

                            it_object++;
                        }
                    }
                    it++;
                }
            }
            catch (const std::exception& e) {
                LOG_ERROR("FRAMOS - Device filtering: " << e.what());
            }

            file.close();
        }
        else {
            LOG_ERROR("FRAMOS - Device filtering cannot open file at location: " << full_file_path.c_str());
        }
    }

    void cs_device_watcher::parse_ip_filter(json::const_iterator& it_object, bool is_allowed, cs_dev_filter_holder& dev_filter_holder)
    {
        if ((*it_object).is_array()) {
            cs_dev_filter::filter_ip holder;

            if ((*it_object).size() == 4) {
                uint8_t lshift = 24;
                holder.ip = 0;
                bool is_err = false;
                for (auto i : it_object.value()) {
                    if (((uint32_t)i) <= 255) {
                        holder.ip |= ((uint32_t)i) << lshift;
                        lshift -= 8;
                    }
                    else {
                        LOG_ERROR("FRAMOS - Invalid IP number format in device filtering file [\"ip\" - octet must be 0-255].");
                        is_err = true;
                        break;
                    }
                }
                if (!is_err) {
                    holder.allowed = is_allowed;
                    if (is_allowed) {
                        dev_filter_holder.allow._filter_ip.push_back(holder);
                    }
                    else {
                        dev_filter_holder.deny._filter_ip.push_back(holder);
                    }
                }
            }
            else {
                LOG_ERROR("FRAMOS - Invalid IP number format in device filtering file [\"ip\" - number of octets].");
            }
        }
        else {
            LOG_ERROR("FRAMOS - Invalid IP number format in device filtering file [\"ip\" - must be an json array format].");
        }
    }

    void cs_device_watcher::parse_ip_range_filter(json::const_iterator& it_object, bool is_allowed, cs_dev_filter_holder& dev_filter_holder)
    {
        bool is_begin = false;
        bool is_end = false;
        bool is_done = false;
        cs_dev_filter::filter_ip_range holder;
        holder.allowed = is_allowed;

        json js_obj_ip_range = json::parse((*it_object).dump());
        json::const_iterator it_obj_ip_range = js_obj_ip_range.begin();
        while (it_obj_ip_range != js_obj_ip_range.end()) {

            if (is_done) {
                LOG_WARNING("FRAMOS - Unsupported number of json \"name\" tags in device filtering file. Detected while parsing [\"ip_range\"] json object.");
                break;
            }

            // range BEGIN
            if (it_obj_ip_range.key() == CS_JSON_FLAG_BEGIN) {
                if ((*it_obj_ip_range).is_array()) {

                    if ((*it_obj_ip_range).size() == 4) {
                        uint8_t lshift = 24;
                        holder.ip_begin = 0;
                        bool is_err = false;
                        for (auto i : it_obj_ip_range.value()) {
                            if (((uint32_t)i) <= 255) {
                                holder.ip_begin |= ((uint32_t)i) << lshift;
                                lshift -= 8;
                            }
                            else {
                                LOG_ERROR("FRAMOS - Invalid IP number format in device filtering file [\"ip_range\" - \"begin\" - octet must be 0-255].");
                                is_err = true;
                                break;
                            }
                        }
                        is_begin = !is_err;
                    }
                    else {
                        LOG_ERROR("FRAMOS - Invalid IP number format in device filtering file [\"ip_range\" - \"begin\" - number of octets].");
                    }
                }
                else {
                    LOG_ERROR("FRAMOS - Invalid IP number format in device filtering file [\"ip_range\" - \"begin\" - must be an array].");
                }
            }
            // range END
            else if (it_obj_ip_range.key() == CS_JSON_FLAG_END) {
                if ((*it_obj_ip_range).is_array()) {

                    if ((*it_obj_ip_range).size() == 4) {
                        uint8_t lshift = 24;
                        holder.ip_end = 0;
                        bool is_err = false;
                        for (auto i : it_obj_ip_range.value()) {
                            if (((uint32_t)i) <= 255) {
                                holder.ip_end |= ((uint32_t)i) << lshift;
                                lshift -= 8;
                            }
                            else {
                                LOG_ERROR("FRAMOS - Invalid IP number format in device filtering file [\"ip_range\" - \"end\" - octet must be 0-255].");
                                is_err = true;
                                break;
                            }
                        }
                        is_end = !is_err;
                    }
                    else {
                        LOG_ERROR("FRAMOS - Invalid IP number format in device filtering file [\"ip_range\" - \"end\" - number of octets].");
                    }
                }
                else {
                    LOG_ERROR("FRAMOS - Invalid IP number format in device filtering file [\"ip_range\" - \"end\" - must be an array].");
                }
            }
            else {
                LOG_WARNING("FRAMOS - Unsupported json \"name\" tag in device filtering file. Detected while parsing [\"ip_range\"] json object.");
            }

            // store only valid informations
            if (is_begin && is_end) {
                if (is_allowed) {
                    dev_filter_holder.allow._filter_ip_range.push_back(holder);
                }
                else {
                    dev_filter_holder.deny._filter_ip_range.push_back(holder);
                }
                is_done = true;
            }

            it_obj_ip_range++;
        }
    }

    void cs_device_watcher::parse_serial_filter(json::const_iterator& it_object, bool is_allowed, cs_dev_filter_holder& dev_filter_holder)
    {
        if ((*it_object).is_string()) {
            cs_dev_filter::filter_serial holder;
            holder.serial_s = (*it_object).get<std::string>();
            holder.allowed = is_allowed;

            bool status = verify_serial(holder.serial_s);
            if (status) {
                holder.serial = serial_from_string(holder.serial_s, false);
                if (is_allowed) {
                    dev_filter_holder.allow._filter_serial.push_back(holder);
                }
                else {
                    dev_filter_holder.deny._filter_serial.push_back(holder);
                }
            }
            else {
                LOG_ERROR("FRAMOS - Invalid serial number format in device filtering file.");
            }
        }
        else {
            LOG_ERROR("FRAMOS - Invalid serial number format in device filtering file [\"serial\" must be a string].");
        }
    }

    void cs_device_watcher::parse_serial_range_filter(json::const_iterator& it_object, bool is_allowed, cs_dev_filter_holder& dev_filter_holder)
    {
        bool is_begin = false;
        bool is_end = false;
        bool is_done = false;
        cs_dev_filter::filter_serial_range holder;
        holder.allowed = is_allowed;

        json js_obj_ser_range = json::parse((*it_object).dump());
        json::const_iterator it_obj_ser_range = js_obj_ser_range.begin();
        while (it_obj_ser_range != js_obj_ser_range.end()) {

            if (is_done) {
                LOG_WARNING("FRAMOS - Unsupported number of json \"name\" tags in device filtering file. Detected while parsing [\"serial_range\"] json object.");
                break;
            }

            // range BEGIN
            if (it_obj_ser_range.key() == CS_JSON_FLAG_BEGIN) {
                if ((*it_obj_ser_range).is_string()) {
                    holder.serial_begin_s = (*it_obj_ser_range).get<std::string>();

                    bool status = verify_serial(holder.serial_begin_s);
                    if (status) {
                        holder.serial_begin = serial_from_string(holder.serial_begin_s, false);
                        is_begin = true;
                    }
                    else {
                        LOG_ERROR("FRAMOS - Invalid serial number format in device filtering file [\"serial_range\" - \"begin\"]");
                    }
                }
                else {
                    LOG_ERROR("FRAMOS - Invalid serial number format in device filtering file [\"serial_range\" - \"begin\" - must be a string].");
                }
            }
            // range END
            else if (it_obj_ser_range.key() == CS_JSON_FLAG_END) {
                if ((*it_obj_ser_range).is_string()) {
                    holder.serial_end_s = (*it_obj_ser_range).get<std::string>();

                    bool status = verify_serial(holder.serial_end_s);
                    if (status) {
                        holder.serial_end = serial_from_string(holder.serial_end_s, false);
                        is_end = true;
                    }
                    else {
                        LOG_ERROR("FRAMOS - Invalid serial number format in device filtering file [\"serial_range\" - \"end\"]");
                    }
                }
                else {
                    LOG_ERROR("FRAMOS - Invalid serial number format in device filtering file [\"serial_range\" - \"end\" - must be a string].");
                }
            }
            else {
                LOG_WARNING("FRAMOS - Unsupported json \"name\" tag in device filtering file. Detected while parsing [\"serial_range\"] json object.");
            }

            // store only valid informations
            if (is_begin && is_end) {
                if (is_allowed) {
                    dev_filter_holder.allow._filter_serial_range.push_back(holder);
                }
                else {
                    dev_filter_holder.deny._filter_serial_range.push_back(holder);
                }
                is_done = true;
            }

            it_obj_ser_range++;
        }
    }

    void cs_device_watcher::parse_subnet_filter(json::const_iterator& it_object, bool is_allowed, cs_dev_filter_holder& dev_filter_holder)
    {
        if ((*it_object).is_number_integer()) {
            cs_dev_filter::filter_subnet holder;
            holder.subnet = (*it_object).get<uint8_t>();
            holder.allowed = is_allowed;
            if (is_allowed) {
                dev_filter_holder.allow._filter_subnet.push_back(holder);
            }
            else {
                dev_filter_holder.deny._filter_subnet.push_back(holder);
            }
        }
        else {
            LOG_ERROR("FRAMOS - Invalid subnet number format in device filtering file [\"subnet\" must be a unsigned interger].");
        }
    }

    std::vector<platform::cs_device_info> cs_device_watcher::find_cs_devices(double max_wait_time_s)
    {
        auto smcs_api = smcs::GetCameraAPI();

        smcs_api->FindAllDevices(max_wait_time_s);
        auto devices = smcs_api->GetAllDevices();

        if (validate_filter_list(_dev_filter_holder, devices)) {
            filter_out_devices(devices, _dev_filter_holder, _is_dev_filter_req);
        }

        std::unique_lock<std::mutex> lock(connected_cs_devices_mtx_);
        /*connected_cs_devices_.clear();
        for (const auto& device : devices)
            if (device->IsOnNetwork() && device->IsSameSubnet())
                connected_cs_devices_.insert(get_cs_device_info(device));*/

        for (const auto& device : devices)
        {
            const bool is_in = connected_cs_devices_.find(get_cs_device_info(device)) != connected_cs_devices_.end();
            if (device->IsOnNetwork() && device->IsSameSubnet())
                connected_cs_devices_.insert(get_cs_device_info(device));
            else if (!device->IsOnNetwork() && is_in)
            {
                // Code bellow handles the case where discovery ack. message from certain device, after FindAllDevices(), could be lost while we are 
                // connected on it. So instead immediate device discard, check CameraSuite if device is still connected.

                // NOTE! Since we are holding "connected_cs_devices_mtx_" mutex, function "device->GetRegister" replaced with "device->IsConnected()" which is much quicker.
                //       cs_device_watcher::OnDisconnect also requires "connected_cs_devices_mtx_" mutex, therefore we need to finish asap

                //UINT32 reg = 0x0A00;
                //UINT32 reg32 = 0;
                //GEV_STATUS gevStatus;
                //bool status = device->GetRegister(reg, &reg32, &gevStatus, 3.0);
                bool status = device->IsConnected();

                if (!status) {
                    handle_disconnected_device(device);
                    connected_cs_devices_.erase(get_cs_device_info(device));
                }
            }
        }

        return std::vector<platform::cs_device_info>(connected_cs_devices_.begin(), connected_cs_devices_.end());
    }

    void cs_device_watcher::add_d400e_device(d400e_camera* camera)
    {
        _d400e_camera_list.push_back(camera);
    }

    void cs_device_watcher::remove_d400e_device(d400e_camera* camera)
    {
        for (size_t i = 0; i < _d400e_camera_list.size(); i++) {
            if (camera == _d400e_camera_list[i]) {
                _d400e_camera_list.erase(_d400e_camera_list.begin() + i);
                break;
            }
        }
    }

    void cs_device_watcher::handle_disconnected_device(smcs::IDevice device)
    {
        // stop/close active streams for unavailable device
        // disable reading timestamp
        for (int i = _d400e_camera_list.size() - 1; i >= 0; i--) {
            std::string serial = _d400e_camera_list[i]->get_info(rs2_camera_info::RS2_CAMERA_INFO_SERIAL_NUMBER);

            if (device->GetSerialNumber() == serial) {
                //_d400e_camera_list[i]->depth_sensor_disable();
                //_d400e_camera_list[i]->color_sensor_disable();
                //_d400e_camera_list[i]->motion_sensor_disable();

                _d400e_camera_list[i]->timestamp_latch_disable();
                break;
            }
        }
    }

    void cs_device_watcher::OnDisconnect(smcs::IDevice device)
    {
        std::unique_lock<std::mutex> lock(connected_cs_devices_mtx_);

        // Reset flag in case it was changed ( cs-sensor::set_cs_param -- case RS2_OPTION_PACKET_SIZE )
        // remove in future if needed
        auto node = device->GetStatisticsNode("DetectOptimalPacketSize");
        if (node != nullptr)
            node->SetBooleanNodeValue(true);

        handle_disconnected_device(device);
        auto info = get_cs_device_info(device);
        connected_cs_devices_.erase(info);
        serial_connection_ids_[info.serial]++;
    }

    cs_device_watcher& cs_device_watcher::get_cs_device_watcher() 
    {
        static cs_device_watcher instance;
        return instance;
    }

    d400e_camera::d400e_camera(std::shared_ptr<context> ctx,
                               const platform::backend_device_group& group,
                               bool register_device_notifications)
            : device(ctx, group, register_device_notifications),
              cs_device_interface(ctx, group),
              cs_depth(ctx, group),
              cs_color(ctx, group),
              cs_motion(ctx, group),
              cs_advanced_mode_base(),
              firmware_logger_device(ctx, group, cs_depth::_hw_monitor,
                  get_firmware_logs_command(),
                  get_flash_logs_command())
    {
        environment::get_instance().get_extrinsics_graph().register_extrinsics(*_color_stream, *_depth_stream, _color_extrinsic);

        register_info(RS2_CAMERA_INFO_NAME, "FRAMOS " + _cs_device_info.id);
        register_info(RS2_CAMERA_INFO_SERIAL_NUMBER, _cs_device_info.serial);
        register_info(RS2_CAMERA_INFO_PRODUCT_ID, get_equivalent_pid(_cs_device_info.id));
        register_info(RS2_CAMERA_INFO_FIRMWARE_VERSION, cs_depth::_fw_version);
        register_info(RS2_CAMERA_INFO_DEVICE_VERSION, _cs_device->get_device_version());
        register_info(RS2_CAMERA_INFO_IP_ADDRESS, _cs_device->get_ip_address());
        register_info(RS2_CAMERA_INFO_SUBNET_MASK, _cs_device->get_subnet_mask());
        register_info(RS2_CAMERA_INFO_RECOMMENDED_FIRMWARE_VERSION, _recommended_fw_version);
        //added because ROS wrapper 2.2.9 requires this property
        register_info(RS2_CAMERA_INFO_PHYSICAL_PORT, "N/A");
        register_info(RS2_CAMERA_INFO_ADVANCED_MODE, ((is_camera_in_advanced_mode()) ? "YES" : "NO"));
        register_info(RS2_CAMERA_INFO_PRODUCT_LINE, "D400e");

        cs_advanced_mode_init(cs_depth::_hw_monitor, &get_depth_sensor());

        rs2_error* e = nullptr;
        int version = rs2_get_api_version(&e);
        std::stringstream s1;
        s1 << (version / 10000) << "." << (version % 10000) / 100 << "." << (version % 100);

        std::stringstream s;
        s << "FRAMOS - API version: " << s1.str() << "\n";        
        s << "    FRAMOS SN: " << _cs_device_info.serial << " - Device version: " << _cs_device->get_device_version() << "\n";        
        s << "    FRAMOS SN: " << _cs_device_info.serial << " - Device IP address: " << _cs_device->get_ip_address() << "\n";        
        s << "    FRAMOS SN: " << _cs_device_info.serial << " - Device subnet mask: " << _cs_device->get_subnet_mask() << "\n";
        LOG_INFO(s.str());

        cs_device_watcher::get_cs_device_watcher().add_d400e_device(this);
    }

    // added destructor which explicitly stops the time diff keepr active objects when called, this is implemented to prevent Pure virtual method called error when time diff keeper destructor is called but,
    // d400e_camera object is still alive
    d400e_camera::~d400e_camera() 
    {

        cs_device_watcher::get_cs_device_watcher().remove_d400e_device(this);

        //_tf_keeper->get_active_object_reference()->stop();
        if (_tf_keeper->is_active_object_active()) {
            _tf_keeper->active_object_stop();
        }
    }    
    ///

    std::shared_ptr<matcher> d400e_camera::create_matcher(const frame_holder& frame) const
    {
        using namespace ds;
        std::vector<stream_interface*> streams = { _depth_stream.get(), _left_ir_stream.get() , _right_ir_stream.get(), _color_stream.get() };
        if (((_device_capabilities & d400_caps::CAP_IMU_SENSOR) == d400_caps::CAP_UNDEFINED) ||
            ((_device_capabilities & d400_caps::CAP_BMI_055) == d400_caps::CAP_UNDEFINED)) {
            return matcher_factory::create(RS2_MATCHER_DEFAULT, streams, _syncer_mode);
        }
        else {
            std::vector<stream_interface*> mm_streams = { _accel_stream.get(), _gyro_stream.get() };
            streams.insert(streams.end(), mm_streams.begin(), mm_streams.end());
            return matcher_factory::create(RS2_MATCHER_DEFAULT, streams, _syncer_mode);
        }
    }

    std::vector<tagged_profile> d400e_camera::get_profiles_tags() const
    {
        std::vector<tagged_profile> markers;
        markers.push_back({ RS2_STREAM_DEPTH, -1, 848, 480, RS2_FORMAT_Z16, 30, profile_tag::PROFILE_TAG_SUPERSET | profile_tag::PROFILE_TAG_DEFAULT });
        markers.push_back({ RS2_STREAM_COLOR, -1, 848, 480, RS2_FORMAT_RGB8, 30, profile_tag::PROFILE_TAG_SUPERSET | profile_tag::PROFILE_TAG_DEFAULT });
        markers.push_back({ RS2_STREAM_INFRARED, -1, 848, 480, RS2_FORMAT_Y8, 30, profile_tag::PROFILE_TAG_SUPERSET});
        markers.push_back({ RS2_STREAM_GYRO, -1, 0, 0, RS2_FORMAT_MOTION_XYZ32F, (int)odr::IMU_FPS_200, profile_tag::PROFILE_TAG_SUPERSET | profile_tag::PROFILE_TAG_DEFAULT });
        markers.push_back({ RS2_STREAM_ACCEL, -1, 0, 0, RS2_FORMAT_MOTION_XYZ32F, (int)odr::IMU_FPS_63, profile_tag::PROFILE_TAG_SUPERSET | profile_tag::PROFILE_TAG_DEFAULT });
        markers.push_back({ RS2_STREAM_ACCEL, -1, 0, 0, RS2_FORMAT_MOTION_XYZ32F, (int)odr::IMU_FPS_100, profile_tag::PROFILE_TAG_SUPERSET | profile_tag::PROFILE_TAG_DEFAULT });
        return markers;
    }

    void d400e_camera::depth_sensor_disable()
    {
        auto& depth_sensor = get_sensor(_depth_device_idx);
        if (depth_sensor.is_streaming()) {
            depth_sensor.stop();
            depth_sensor.close();
        }
    }

    void d400e_camera::color_sensor_disable()
    {
        auto& color_sensor = get_sensor(_color_device_idx);
        if (color_sensor.is_streaming()) {
            color_sensor.stop();
            color_sensor.close();
        }
    }

    void d400e_camera::motion_sensor_disable()
    {
        try {
            auto& motion_sensor = get_sensor(_motion_module_device_idx.value());
            if (motion_sensor.is_streaming()) {
                motion_sensor.stop();
                motion_sensor.close();
            }
        }
        catch (std::exception e) {
        }
    }

    void d400e_camera::timestamp_latch_disable() 
    {
        // NH - TODO - Check if already stopped!?
        if (_tf_keeper->is_active_object_active()) {
            _tf_keeper->active_object_stop();
        }
    }

    std::string d400e_camera::get_equivalent_pid(std::string id) const
    {
        if (id.compare(CS_CAMERA_MODEL_D435e) == 0) 
            return "0B07"; //D435 PID
        else if (id.compare(CS_CAMERA_MODEL_D415e) == 0) 
            return "0AD3"; //D415 PID
        else if (id.compare(CS_CAMERA_MODEL_D455e) == 0) 
            return "0B5C"; //D455 PID
        else
            return "N/A";
    }
}