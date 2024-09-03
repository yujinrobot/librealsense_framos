// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 FRAMOS GmbH.

#pragma once

#include "librealsense2/h/rs_d400e.h"
#include <vector>
#include <memory>
#include <string>

namespace librealsense
{
    namespace d400e
    {
        using seconds = double;

        const std::string CS_CLI_ARG_FILTER_FILE = "dev_filter";
        const std::string CS_CLI_ARG_FILTER_FILE_EXT = ".json";

        class cli_args
        {
        public:
            static cli_args& get_instance();
            void set(const rs2_cli_arg* cli_arg);
            rs2_cli_arg* get();
            const std::string& get_filter_file_path() const;
            cli_args(cli_args const&) = delete;
            void operator=(cli_args const&) = delete;
        private:
            cli_args();
            bool verify_cli_arg(rs2_cli_arg* carg, std::string& full_file_path_out);
            rs2_cli_arg _rs2_cli_arg;
            std::string _filter_file_path;
        };

        class filter_list
        {
        public:
            static filter_list& get_instance();
            void set(const std::vector<rs2_d400e_filter>& filter_list);
            const std::vector<rs2_d400e_filter>& get_filter_list() const;
            filter_list(filter_list const&) = delete;
            void operator=(filter_list const&) = delete;
        private:
            filter_list();
            std::vector<rs2_d400e_filter> _d400e_filter_list;
        };

        class heartbeat_time
        {
        public:
            static heartbeat_time& get_instance();
            void set(seconds time);
            seconds get();
            heartbeat_time(heartbeat_time const&) = delete;
            void operator=(heartbeat_time const&) = delete;
        private:
            heartbeat_time();
        };

        class device_diagnostics
        {
        public:
            static device_diagnostics& get_instance();
            int set(const char* dev_serial, int toggle);
            device_diagnostics(device_diagnostics const&) = delete;
            void operator=(device_diagnostics const&) = delete;
            
            enum dev_diag_status
            {
                dev_diag_status_OK = 0,
                dev_diag_status_NOK = 1
            };

            enum dev_diag_toggle
            {
                dev_diag_toggle_ON = 1,
                dev_diag_toggle_OFF = 0
            };

        private:
            device_diagnostics();
        };

        void set_buffer_count(int buffer_count);
        int get_buffer_count();
        void set_port_range(uint16_t min, uint16_t max);
        void get_port_range(uint16_t* min, uint16_t* max);
        std::vector<uint16_t> query_ports(rs2_d400e_port_type type);
        void set_device_filter_list(const std::vector<rs2_d400e_filter>& filter_list);
    }
}