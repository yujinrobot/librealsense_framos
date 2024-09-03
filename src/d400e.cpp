// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 FRAMOS GmbH.

#include "d400e.h"
#include "smcs_cpp/CameraSDK.h"
#include <algorithm>
#include <iterator>
#include <stdexcept>
#include "types.h"

namespace librealsense
{
    namespace d400e
    {
        // class cli_args

        cli_args::cli_args()
        {
            _rs2_cli_arg.argc = 0;
            _rs2_cli_arg.argv = nullptr;
            _filter_file_path = "";
        }

        cli_args& cli_args::get_instance()
        {
            static cli_args instance;
            return instance;
        }

        void cli_args::set(const rs2_cli_arg* cli_arg)
        {
            if (cli_arg != nullptr) {
                _rs2_cli_arg.argc = cli_arg->argc;
                _rs2_cli_arg.argv = cli_arg->argv;

                verify_cli_arg(&_rs2_cli_arg, _filter_file_path);
            }
        }

        rs2_cli_arg* cli_args::get()
        {
            return &_rs2_cli_arg;
        }

        const std::string& cli_args::get_filter_file_path() const
        {
            return _filter_file_path;
        }

        bool cli_args::verify_cli_arg(rs2_cli_arg* carg, std::string& full_file_path_out)
        {
            bool status = false;

            int remain_argc = carg->argc;

            // check for option flags
            while (remain_argc > 0) {

                if (carg->argv == nullptr) {
                    break;
                }

                // convert argument to lower case
                std::string option = std::string(carg->argv[0]);
                //std::transform(option.begin(), option.end(), option.begin(), ::tolower);

                // check if device filter file is specified
                if (option.find(CS_CLI_ARG_FILTER_FILE) != std::string::npos) {

                    size_t pos = option.find("=");
                    if (pos != std::string::npos) {
                        full_file_path_out = option.substr(pos + 1, option.size() - pos - 1);

                        pos = full_file_path_out.find(CS_CLI_ARG_FILTER_FILE_EXT);
                        if (pos != std::string::npos) {

                            if ((pos + CS_CLI_ARG_FILTER_FILE_EXT.size()) <= full_file_path_out.size()) {
                                full_file_path_out = full_file_path_out.substr(0, pos + CS_CLI_ARG_FILTER_FILE_EXT.size());
                                status = true;
                            }
                            else {
                                // TODO - NH not possible case!?
                                LOG_ERROR("FRAMOS - Device filter tag detected - internal error.");
                            }
                        }
                        else {
                            LOG_ERROR("FRAMOS - Device filter tag detected but no json file extension (ex. \".json\").");
                        }
                    }
                    else {
                        LOG_ERROR("FRAMOS - Device filter tag detected but missing \"=\" sign (ex. dev_filter=filter_file_path).");
                    }

                    break;
                }

                carg->argv++;
                remain_argc--;
            }

            return status;
        }

        // !class cli_args

        // class filter_list

        filter_list::filter_list()
        {
        }
        
        filter_list& filter_list::get_instance()
        {
            static filter_list instance;
            return instance;
        }
        
        void filter_list::set(const std::vector<rs2_d400e_filter>& filter_list)
        {
            _d400e_filter_list.clear();
            _d400e_filter_list.insert(_d400e_filter_list.end(), filter_list.begin(), filter_list.end());
        }
        
        const std::vector<rs2_d400e_filter>& filter_list::get_filter_list() const
        {
            return _d400e_filter_list;
        }

        // !class filter_list

        // class heartbeat_time

        heartbeat_time::heartbeat_time()
        {
            constexpr seconds default_heartbeat_time = 3;
            set(default_heartbeat_time);
        }

        heartbeat_time& heartbeat_time::get_instance()
        {
            static heartbeat_time instance;
            return instance;
        }

        void heartbeat_time::set(seconds time)
        {
            smcs::GetCameraAPI()->SetHeartbeatTime(time);
        }

        seconds heartbeat_time::get()
        {
            return smcs::GetCameraAPI()->GetHeartbeatTime();
        }
        
        // !class heartbeat_time

        // class device_diagnostics

        device_diagnostics::device_diagnostics()
        {
        }

        device_diagnostics& device_diagnostics::get_instance()
        {
            static device_diagnostics instance;
            return instance;
        }

        int device_diagnostics::set(const char* dev_serial, int toggle)
        {
            uint64_t dev_serial_u = 0;
            int r_status = dev_diag_status_NOK;

            if (dev_serial == nullptr) {
                return r_status;
            }

            if ((toggle != dev_diag_toggle_ON) && (toggle != dev_diag_toggle_OFF)) {
                return r_status;
            }

            try {
                const std::string& dev_serial_str = {dev_serial};
                dev_serial_u = std::stoll(dev_serial_str, 0, 16);
            }
            catch (...) {
                return r_status;
            }

            smcs::IDevice cs_device = smcs::GetCameraAPI()->GetDeviceByMac(dev_serial_u);
            if (cs_device.IsValid()) {

                if (cs_device->IsConnected()) {

                    bool b_status = false;

                    if (toggle == (int)dev_diag_toggle_ON) {
                        b_status = cs_device->SetStringNodeValue("DebugInformation", "On");
                    }
                    else if (toggle == (int)dev_diag_toggle_OFF) {
                        b_status = cs_device->SetStringNodeValue("DebugInformation", "Off");
                    }

                    if (b_status) {
                        r_status = dev_diag_status_OK;
                    }
                }
            }

            return r_status;
        }

        // !class device_diagnostics

        void set_buffer_count(int buffer_count)
        {
            auto node = smcs::GetCameraAPI()->GetApiParametersNode("ImageBufferFrameCount");
            if (node == nullptr)
                throw std::runtime_error("Unable to acquire frame buffer count node");

            if (!node->SetIntegerNodeValue(buffer_count))
                throw std::runtime_error("Unable to set frame buffer count");
        }

        int get_buffer_count()
        {
            auto node = smcs::GetCameraAPI()->GetApiParametersNode("ImageBufferFrameCount");
            if (node == nullptr)
                throw std::runtime_error("Unable to acquire frame buffer count node");

            INT64 value;
            if (!node->GetIntegerNodeValue(value))
                throw std::runtime_error("Unable to acquire frame buffer count value");

            return static_cast<int>(value);
        }

        void set_port_range(uint16_t min, uint16_t max)
        {
            auto api = smcs::GetCameraAPI();

            auto enable_port_range_node = api->GetApiParametersNode("EnableStaticPort");
            if (enable_port_range_node == nullptr)
                throw std::runtime_error("Unable to acquire port range node");
            if (!enable_port_range_node->SetBooleanNodeValue(true))
                throw std::runtime_error("Unable to enable port range");

            auto min_port_node = api->GetApiParametersNode("MinimumPortNumber");
            if (min_port_node == nullptr)
                throw std::runtime_error("Unable to acquire min port range node");
            if (!min_port_node->SetIntegerNodeValue(min))
                throw std::runtime_error("Unable to set min port value");

            auto max_port_node = api->GetApiParametersNode("MaximumPortNumber");
            if (max_port_node == nullptr)
                throw std::runtime_error("Unable to acquire max port range node");
            if (!max_port_node->SetIntegerNodeValue(max))
                throw std::runtime_error("Unable to set max port value");

            auto new_ports_node = api->GetApiParametersNode("OpenNewPorts");
            if (new_ports_node == nullptr)
                throw std::runtime_error("Unable to acquire new ports node");
            if (!new_ports_node->CommandNodeExecute())
                throw std::runtime_error("Unable to set new ports");
        }

        void get_port_range(uint16_t* min, uint16_t* max)
        {
            auto api = smcs::GetCameraAPI();

            auto enable_port_range_node = api->GetApiParametersNode("EnableStaticPort");
            if (enable_port_range_node == nullptr)
                throw std::runtime_error("Unable to acquire port range node");
            bool port_range_enabled;
            if (!enable_port_range_node->GetBooleanNodeValue(port_range_enabled))
                throw std::runtime_error("Unabe to acquire port range node value");
            if (!port_range_enabled)
                throw std::runtime_error("Port range not set");

            auto min_port_node = api->GetApiParametersNode("MinimumPortNumber");
            if (min_port_node == nullptr)
                throw std::runtime_error("Unable to acquire min port range node");
            INT64 min_value;
            if (!min_port_node->GetIntegerNodeValue(min_value))
                throw std::runtime_error("Unable to get min port value");
            *min = static_cast<unsigned short>(min_value);

            auto max_port_node = api->GetApiParametersNode("MaximumPortNumber");
            if (max_port_node == nullptr)
                throw std::runtime_error("Unable to acquire max port range node");
            INT64 max_value;
            if (!max_port_node->GetIntegerNodeValue(max_value))
                throw std::runtime_error("Unable to get max port value");
            *max = static_cast<unsigned short>(max_value);
        }

        std::vector<std::string> split_string(const std::string string, char delimiter)
        {
            std::vector<std::string> split;
            auto begin = string.begin();
            auto end = string.end();
            while (begin != end) {
                auto delimiter_position = std::find(begin, end, delimiter);
                split.emplace_back(begin, delimiter_position);
                begin = delimiter_position;
                if (begin != end)
                    ++begin;
            }
            return split;
        }

        std::vector<uint16_t> parse_ports(std::string csv_ports)
        {
            auto split = split_string(csv_ports, ',');
            std::vector<uint16_t> ports;
            std::transform(split.begin(), split.end(), std::back_inserter(ports),
                [](std::string s) { return ::atoi(s.c_str()); }
            );
            return ports;
        }

        std::vector<uint16_t> query_ports(std::string ports_node_name)
        {
            auto ports_node = smcs::GetCameraAPI()->GetApiParametersNode(ports_node_name);
            std::string ports;
            if (ports_node && ports_node->GetStringNodeValue(ports))
                return parse_ports(ports);
            else
                throw std::runtime_error(std::string("Unable to acquire node ") + ports_node_name);

        }

        std::vector<uint16_t> query_control_ports()
        {
            return query_ports("ControlPorts");
        }

        std::vector<uint16_t> query_stream_ports()
        {
            return query_ports("StreamPorts");
        }

        std::vector<uint16_t> query_message_ports()
        {
            return query_ports("MessagePorts");
        }

        std::vector<uint16_t> query_ports(rs2_d400e_port_type type)
        {
            switch (type) {
            case RS2_D400E_PORT_TYPE_CONTROL:
                return query_control_ports();
            case RS2_D400E_PORT_TYPE_STREAM:
                return query_stream_ports();
            case RS2_D400E_PORT_TYPE_MESSAGE:
                return query_message_ports();
            case RS2_D400E_PORT_TYPE_ALL:
            {
                auto ports = query_control_ports();
                auto stream_ports = query_stream_ports();
                ports.insert(ports.end(), stream_ports.begin(), stream_ports.end());
                auto message_ports = query_message_ports();
                ports.insert(ports.end(), message_ports.begin(), message_ports.end());
                return ports;
            }
            default:
                throw std::runtime_error("Invalid port type");
            }
        }

        void set_device_filter_list(const std::vector<rs2_d400e_filter>& filter_list)
        {
            filter_list::get_instance().set(filter_list);
        }
    }
}