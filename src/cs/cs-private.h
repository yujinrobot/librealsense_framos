// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 FRAMOS GmbH.
#pragma once

#include <map>
#include <iomanip>
#include <string>
#include <cstddef>
#include <cstdint>

namespace librealsense
{
    namespace cs
    {
        enum fw_cmd : uint8_t
        {
            MRD             = 0x01,     // Read Register
            FRB             = 0x09,     // Read from flash
            FWB             = 0x0a,     // Write to flash <Parameter1 Name="StartIndex"> <Parameter2 Name="Size">
            FES             = 0x0b,     // Erase flash sector <Parameter1 Name="Sector Index"> <Parameter2 Name="Number of Sectors">
            FEF             = 0x0c,     // Erase flash full <Parameter1 Name="0xACE">
            FSRU            = 0x0d,     // Flash status register unlock
            FPLOCK          = 0x0e,     // Permanent lock on lower Quarter region of the flash
            GLD             = 0x0f,     // FW logs
            GVD             = 0x10,     // camera details
            GETINTCAL       = 0x15,     // Read calibration table
            SETINTCAL       = 0x16,     // Set Internal sub calibration table
            LOADINTCAL      = 0x1D,     // Get Internal sub calibration table
            DFU             = 0x1E,     // Enter to FW update mode
            HWRST           = 0x20,     // hardware reset
            OBW             = 0x29,     // OVT bypass write
            SET_ADV         = 0x2B,     // set advanced mode control
            GET_ADV         = 0x2C,     // get advanced mode control
            EN_ADV          = 0x2D,     // enable advanced mode
            UAMG            = 0X30,     // get advanced mode status
            PFD             = 0x3b,     // Disable power features <Parameter1 Name="0 - Disable, 1 - Enable" />
            SETAEROI        = 0x44,     // set auto-exposure region of interest
            GETAEROI        = 0x45,     // get auto-exposure region of interest
            MMER            = 0x4F,     // MM EEPROM read ( from DS5 cache )
            CALIBRECALC     = 0x51,     // Calibration recalc and update on the fly
            GET_EXTRINSICS  = 0x53,     // get extrinsics
            CAL_RESTORE_DFLT= 0x61,     // Reset Depth/RGB calibration to factory settings
            SETINTCALNEW    = 0x62,     // Set Internal sub calibration table
            SET_CAM_SYNC    = 0x69,     // set Inter-cam HW sync mode [0-default, 1-master, 2-slave]
            GET_CAM_SYNC    = 0x6A,     // fet Inter-cam HW sync mode
            SETRGBAEROI     = 0x75,     // set RGB auto-exposure region of interest
            GETRGBAEROI     = 0x76,     // get RGB auto-exposure region of interest
            SET_PWM_ON_OFF  = 0x77,     // set emitter on and off mode
            GET_PWM_ON_OFF  = 0x78,     // get emitter on and off mode
            SETSUBPRESET    = 0x7B,     // Download sub-preset
            GETSUBPRESET    = 0x7C,     // Upload the current sub-preset
            GETSUBPRESETNAME= 0x7D,     // Retrieve sub-preset's name
            RECPARAMSGET    = 0x7E,     // Retrieve depth calibration table in new format (fw >= 5.11.12.100)
            AUTO_CALIB      = 0x80      // auto calibration commands
        };

        const int etDepthTableControl = 9; // Identifier of the depth table control

        enum advanced_query_mode
        {
            GET_VAL = 0,
            GET_MIN = 1,
            GET_MAX = 2,
        };

        struct depth_table_control
        {
            uint32_t depth_units;
            int32_t depth_clamp_min;
            int32_t depth_clamp_max;
            int32_t disparity_multiplier;
            int32_t disparity_shift;
        };

        enum inter_cam_sync_mode {
            INTERCAM_SYNC_DEFAULT,
            INTERCAM_SYNC_MASTER,
            INTERCAM_SYNC_SLAVE,
            INTERCAM_SYNC_EXTERNAL,
            INTERCAM_SYNC_MAX
        };

        enum inter_cam_sync_mode_color {
            INTERCAM_SYNC_DEFAULT_COLOR,
            INTERCAM_SYNC_EXTERNAL_COLOR,
            INTERCAM_SYNC_MAX_COLOR
        };

        const uint16_t  D4_HWm_BUFFER_SIZE = 1024;

        struct d4_hwm_cmd
        {
            uint16_t Size;
            uint16_t MagicWord;
            uint32_t OpCode;
            uint32_t Param1;
            uint32_t Param2;
            uint32_t Param3;
            uint32_t Param4;
            uint8_t Data[D4_HWm_BUFFER_SIZE];
        };

        const uint32_t d4_hwm_cmd_size_offset = offsetof(d4_hwm_cmd, Size);
        const uint32_t d4_hwm_cmd_magicword_offset = offsetof(d4_hwm_cmd, MagicWord);
        const uint32_t d4_hwm_cmd_opcode_offset = offsetof(d4_hwm_cmd, OpCode);
        const uint32_t d4_hwm_cmd_param1_offset = offsetof(d4_hwm_cmd, Param1);
        const uint32_t d4_hwm_cmd_param2_offset = offsetof(d4_hwm_cmd, Param2);
        const uint32_t d4_hwm_cmd_param3_offset = offsetof(d4_hwm_cmd, Param3);
        const uint32_t d4_hwm_cmd_param4_offset = offsetof(d4_hwm_cmd, Param4);
        const uint32_t d4_hwm_cmd_data_offset = offsetof(d4_hwm_cmd, Data);

        const uint32_t d4_hwm_cmd_opcode_imu_calib = 0x50;

    } // namespace cs
} // namespace librealsense