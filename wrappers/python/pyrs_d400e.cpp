// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 FRAMOS GmbH.

#include "python.hpp"
#include "../include/librealsense2/hpp/rs_d400e.hpp"

void init_d400e(py::module& m) {
    /* rs2_d400e.hpp */
    BIND_ENUM(m, rs2_d400e_port_type, RS2_D400E_PORT_TYPE_COUNT, "port types for d400e device");
    auto d400e_submodule = m.def_submodule("d400e", "Functionality specific for D400e series devices");
    py::class_<rs2_d400e_filter_serial_range> serial_range(d400e_submodule, "rs2_d400e_filter_serial_range");
    serial_range
        .def_readwrite("begin", &rs2_d400e_filter_serial_range::begin)
        .def_readwrite("end", &rs2_d400e_filter_serial_range::end)
        .def(py::init<>());
    py::class_<rs2_d400e_filter_ip_range> ip_range(d400e_submodule, "rs2_d400e_filter_ip_range");
    ip_range
        .def_readwrite("begin", &rs2_d400e_filter_ip_range::begin)
        .def_readwrite("end", &rs2_d400e_filter_ip_range::end)
        .def(py::init<>());
    py::class_<rs2_d400e_filter> rs2_filter(d400e_submodule, "rs2_d400e_filter");
    rs2_filter
        .def_readwrite("serial_range", &rs2_d400e_filter::serial_range)
        .def_readwrite("ip_range", &rs2_d400e_filter::ip_range)
        .def(py::init<>());
	py::class_<rs2::d400e::port_range> port_range(d400e_submodule, "port_range");
    port_range
        .def_readwrite("min", &rs2::d400e::port_range::min)
        .def_readwrite("max", &rs2::d400e::port_range::max)
        .def(py::init<uint16_t, uint16_t>());
    d400e_submodule.def("set_heartbeat_time", &rs2::d400e::set_heartbeat_time, "set heartbeat time for d400e devices");
    d400e_submodule.def("get_heartbeat_time", &rs2::d400e::get_heartbeat_time, "retrieve heartbeat time for d400e devices");
    d400e_submodule.def("set_buffer_count", &rs2::d400e::set_buffer_count, "set number of buffers for d400e devices");
    d400e_submodule.def("get_buffer_count", &rs2::d400e::get_buffer_count, "get number of buffers for d400e devices");
    d400e_submodule.def("toggle_device_diagnostics", &rs2::d400e::toggle_device_diagnostics, "enable or disable diagnostics feature for d400e devices");
	d400e_submodule.def("set_port_range", &rs2::d400e::set_port_range, "set allowed port range for d400e devices");
    d400e_submodule.def("get_port_range", &rs2::d400e::get_port_range, "get allowed port range for d400e devices");
    d400e_submodule.def("query_ports", &rs2::d400e::query_ports, "get ports used for d400e devices", "type"_a=RS2_D400E_PORT_TYPE_ALL);
    d400e_submodule.def("set_device_filter_list", &rs2::d400e::set_device_filter_list, "provide device filter list");
    d400e_submodule.def("set_cli_args", [](/*std::vector<std::string> args*/std::string& args) {
        /*std::vector<char*> cstrs;
        cstrs.reserve(args.size());
        for (auto &s : args) {
            cstrs.push_back(const_cast<char*>(s.c_str()));
        }
        rs2::d400e::cli_arg carg;
        carg.argc = cstrs.size();
        carg.argv = const_cast<const char**>(cstrs.data());
        return rs2::d400e::set_cli_args(carg);*/

        std::vector<char*> cstrs;
        rs2::d400e::cli_arg carg;

        carg.argc = 0;
        if (args.size() > 0) {
            carg.argc++;
            cstrs.push_back((char*)(&args[0]));
        }

        for (size_t i = 1; i < args.size(); i++) {
            if (args[i] == ' ') {
                if ((i + 1) < args.size()) {
                    carg.argc++;
                    cstrs.push_back((char*)(&args[i + 1]));
                }
            }
        }
        carg.argv = const_cast<const char**>(cstrs.data());

        return rs2::d400e::set_cli_args(carg);
    }, "provide command line arguments");
    /** end rs_d400e.hpp **/
}
