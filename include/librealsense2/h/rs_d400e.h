// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 FRAMOS GmbH.


/** \file rs_d400e.h
* \brief Exposes functionality specific to D400e devices for C compilers
*/

#ifndef LIBREALSENSE_RS2_D400E_H
#define LIBREALSENSE_RS2_D400E_H

#ifdef __cplusplus
extern "C" {
#endif

#include "rs_types.h"

#define RS2_API_D400E

typedef struct rs2_d400e_port_list rs2_d400e_port_list;
typedef struct rs2_cli_arg rs2_cli_arg;
typedef struct rs2_d400e_filter_list rs2_d400e_filter_list;

/**
* Sets heartbeat time in seconds for D400e series devices.
* Heartbeat time is used for device disconnect detection.
* \param       time      New heartbeat time in seconds for all D400e devices. Heartbeat timeout is set to 4x heartbeat time
* \param[out]  error     If non-null, receives any error that occurs during this call, otherwise, errors are ignored
*/
void rs2_d400e_set_heartbeat_time(double time, rs2_error** error);

/**
* Acquires heartbeat time in seconds for D400e series devices.
* Heartbeat time is used for device disconnect detection.
* \param[out]  error     If non-null, receives any error that occurs during this call, otherwise, errors are ignored
* @return                Current heartbeat time in seconds for all D400e devices. Heartbeat timeout is 4x heartbeat time
*/
double rs2_d400e_get_heartbeat_time(rs2_error** error);

/**
* Sets number of buffers for D400e series devices.
* Must be set before instantiating a context or a pipeline.
* \param       buffer_count Number of buffers to set for all D400e devices
* \param[out]  error     If non-null, receives any error that occurs during this call, otherwise, errors are ignored
*/
void rs2_d400e_set_buffer_count(int buffer_count, rs2_error** error);

/**
* Acquires number for buffers for D400e series devices.
* \param[out]  error     If non-null, receives any error that occurs during this call, otherwise, errors are ignored
* @return                Current number for buffers for all D400e devices
*/
int rs2_d400e_get_buffer_count(rs2_error** error);

/**
* Enable or disable diagnostic packets sent by D400e series devices.
* \param       dev_serial   Serial number of a device on which diagnostic feature have to be toggled
* \param       toggle       1 - to enable device diagnostics feature, 0 - to disable device diagnostics feature
* \param[out]  error        If non-null, receives any error that occurs during this call, otherwise, errors are ignored
* @return                   0 on success, 1 on failure
*/
int rs2_d400e_toggle_device_diagnostics(const char* dev_serial, int toggle, rs2_error** error);

/**
* Sets the range of stream and message ports. All newly opened ports will be in this range. 
* Ports that are already open will be closed and opened within range. 
* Has no effect if called after constructing a context or a pipeline.
* If the number of ports required for normal functioning exceeds the number of free ports in the range, 
* ports will be allocated outside of the range.
* \param       min          Start of the allowd port range (inclusive)
* \param       max          End of the port range (inclusive)
* \param[out]  error        If non-null, receives any error that occurs during this call, otherwise, errors are ignored
*/
void rs2_d400e_set_port_range(unsigned short min, unsigned short max, rs2_error** error);

/**
* Acquires the allowed range of stream and message ports.
* There is no allowed range set by default. It is not possible to acquire allowed port range before setting it.
* \param[out]  min          Start of the allowed port range (inclusive)
* \param[out]  max          End of the port range (inclusive)
* \param[our]  error        If non-null, receives any error that occurs during this call, otherwise, errors are ignored
*/
void rs2_d400e_get_port_range(unsigned short* min, unsigned short* max, rs2_error** error);

/**
* Acquires a list of all ports used by D400e cameras.
* Acquired list must be deleted using rs2_d400e_delete_port_list.
* \param[out]  error        If non-null, receives any error that occurs during this call, otherwise, errors are ignored
* @return                   List of ports used by D400e cameras
*/
rs2_d400e_port_list* rs2_d400e_query_ports(rs2_error** error);

/**
* Acquires a list of ports used by D400e cameras filtered by type.
* Acquired list must be deleted using rs2_d400e_delete_port_list.
* \param[in]   type         Port type
* \param[out]  error        If non-null, receives any error that occurs during this call, otherwise, errors are ignored
* @return                   List of ports used by D400e cameras filtered by type
*/
rs2_d400e_port_list* rs2_d400e_query_ports_by_type(rs2_d400e_port_type type, rs2_error** error);

/**
* Acquires the number of ports in the port list.
* \param[in]   ports        List of ports
* \param[out]  error        If non-null, receives any error that occurs during this call, otherwise, errors are ignored
* @return                   Number of ports in the list
*/
unsigned int rs2_d400e_get_port_count(const rs2_d400e_port_list* ports, rs2_error** error);

/**
* Acquires a port from the list of ports used by D400e cameras.
* \param[in]   ports        List of ports
* \param[in]   index        Index of port to acquire
* \param[out]  error        If non-null, receives any error that occurs during this call, otherwise, errors are ignored
* @return                   Port at given index in the list
*/
unsigned short rs2_d400e_get_port(const rs2_d400e_port_list* ports, unsigned int index, rs2_error** error);

/**
* \param[in]   ports        List of ports.
*/
void rs2_d400e_delete_port_list(rs2_d400e_port_list* ports);

/**
* Provide command line arguments (could be used for providing the config file used for device filtering).
* \param[in]   cli_arg      command line arguments structure.
* \param[our]  error        If non-null, receives any error that occurs during this call, otherwise, errors are ignored
*/
void rs2_d400e_set_cli_args(const rs2_cli_arg* cli_arg, rs2_error** error);

/**
* \param[our]  error        If non-null, receives any error that occurs during this call, otherwise, errors are ignored.
* @return                   new device filter list.
*/
rs2_d400e_filter_list* rs2_d400e_new_filter_list(rs2_error** error);

/**
* \param[in]   filter_list  device filter list to delete.
*/
void rs2_d400e_delete_filter_list(rs2_d400e_filter_list* filter_list);

/**
* \param[in]   filter       filter record which need to be inserted into filter_list.
* \param[in]   filter_list  device filter list in which filter argument will be inserted.
* \param[our]  error        If non-null, receives any error that occurs during this call, otherwise, errors are ignored.
*/
void rs2_d400e_filter_list_insert(const rs2_d400e_filter* filter, rs2_d400e_filter_list* filter_list, rs2_error** error);

/**
* Provide device filter list. It can be used as a replacement of providing device filtering file via command line arguments.
* \param[in]   list         device filter list.
* \param[our]  error        If non-null, receives any error that occurs during this call, otherwise, errors are ignored.
*/
void rs2_d400e_set_device_filter_list(const rs2_d400e_filter_list* list, rs2_error** error);

#ifdef __cplusplus
}
#endif
#endif
