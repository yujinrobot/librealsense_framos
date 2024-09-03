// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#ifndef LIBREALSENSE_RS2_D400E_HPP
#define LIBREALSENSE_RS2_D400E_HPP

#include "rs_types.hpp"

namespace rs2
{
    namespace d400e
    {
        typedef rs2_cli_arg cli_arg;

        struct port_range
        {
            uint16_t min;
            uint16_t max;
        };

        /**
        * set heartbeat time for d400e devices
        * \param[in] time     heartbeat time in seconds
        */
        inline void set_heartbeat_time(double time)
        {
            rs2_error* e = nullptr;
            rs2_d400e_set_heartbeat_time(time, &e);
            rs2::error::handle(e);
        }

        /**
        * retrieve heartbeat time for d400e devices
        * \return             heartbeat time in seconds
        */
        inline double get_heartbeat_time()
        {
            rs2_error* e = nullptr;
            auto time = rs2_d400e_get_heartbeat_time(&e);
            rs2::error::handle(e);

            return time;
        }

        /**
        * set buffer count for d400e devices
        * \param[in] buffer_count     number of buffers
        */
        inline void set_buffer_count(int buffer_count)
        {
            rs2_error* e = nullptr;
            rs2_d400e_set_buffer_count(buffer_count, &e);
            rs2::error::handle(e);
        }

        /**
        * retrieve buffer count for d400e devices
        * \return             buffer count in seconds
        */
        inline int get_buffer_count()
        {
            rs2_error* e = nullptr;
            auto buffer_count = rs2_d400e_get_buffer_count(&e);
            rs2::error::handle(e);

            return buffer_count;
        }

        /**
        * enable or disable diagnostics feature for d400e devices
        * \param[in] dev_serial     Serial number of a device on which diagnostic feature have to be toggled
        * \param[in] toggle         1 - to enable device diagnostics feature, 0 - to disable device diagnostics feature
        * \return                   0 on success, 1 on failure
        */
        inline int toggle_device_diagnostics(const char* dev_serial, int toggle)
        {
            rs2_error* e = nullptr;
            int status = rs2_d400e_toggle_device_diagnostics(dev_serial, toggle, &e);
            rs2::error::handle(e);

            return status;
        }

        /**
        * set port range for d400e devices
        * \param[in] range         range of allowed ports
        */
        inline void set_port_range(port_range range)
        {
            rs2_error* e = nullptr;
            rs2_d400e_set_port_range(range.min, range.max, &e);
            rs2::error::handle(e);
        }

        /**
        * get port range for d400e devices
        * return                    previously set allowed port range
        */
        inline port_range get_port_range()
        {
            rs2_error* e = nullptr;
            port_range range;
            rs2_d400e_get_port_range(&range.min, &range.max, &e);
            rs2::error::handle(e);

            return range;
        }

        /**
        * get a list of ports used by d400e devices
        * \param[in] type           filter port list by type
        * return                    list of d400e ports filtered by type
        */
        inline std::vector<uint16_t> query_ports(rs2_d400e_port_type type = RS2_D400E_PORT_TYPE_ALL)
        {
            rs2_error* e = nullptr;

            std::shared_ptr<rs2_d400e_port_list> list(
                rs2_d400e_query_ports_by_type(type, &e),
                rs2_d400e_delete_port_list
            );
            rs2::error::handle(e);

            auto size = rs2_d400e_get_port_count(list.get(), &e);
            rs2::error::handle(e);

            std::vector<uint16_t> ports;
            for (uint32_t i = 0; i < size; ++i) {
                ports.push_back(rs2_d400e_get_port(list.get(), i, &e));
                rs2::error::handle(e);
            }

            return ports;
        }

        /**
        * provide command line arguments (could be used for providing the config file used for device filtering)
        * \param[in] cli_arg        command line arguments structure
        */
        inline void set_cli_args(const cli_arg& carg)
        {
            rs2_error* e = nullptr;

            rs2_cli_arg rs2_carg;
            rs2_carg.argc = carg.argc;
            rs2_carg.argv = carg.argv;

            rs2_d400e_set_cli_args(&rs2_carg, &e);

            rs2::error::handle(e);
        }

        /**
        * provide device filter list (can be used as a replacement of providing device filtering file via command line arguments)
        * \param[in] cli_arg        command line arguments structure
        */
        inline void set_device_filter_list(const std::vector<rs2_d400e_filter>& filter_list)
        {
            rs2_error* e = nullptr;

            std::shared_ptr<rs2_d400e_filter_list> list(
                rs2_d400e_new_filter_list(&e),
                rs2_d400e_delete_filter_list
            );
            rs2::error::handle(e);

            for (auto elem : filter_list) {
                rs2_d400e_filter_list_insert(&elem, list.get(), &e);
                rs2::error::handle(e);
            }

            rs2_d400e_set_device_filter_list(list.get(), &e);
            rs2::error::handle(e);
        }
    }
}
#endif // LIBREALSENSE_RS2_D400E_HPP
