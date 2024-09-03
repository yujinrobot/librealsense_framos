// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.

#pragma once

#include "cs/cs-motion.h"
//#include "ds5/ds5-motion.h"
#include "proc/synthetic-stream.h"

namespace librealsense
{
    class enable_motion_correction;
    class cs_mm_calib_handler;
    class functional_processing_block;

    class cs_motion_transform : public functional_processing_block
    {
    public:
        cs_motion_transform(rs2_format target_format, rs2_stream target_stream,
            std::shared_ptr<cs_mm_calib_handler> mm_calib = nullptr,
            std::shared_ptr<enable_motion_correction> mm_correct_opt = nullptr);

    protected:
        cs_motion_transform(const char* name, rs2_format target_format, rs2_stream target_stream,
            std::shared_ptr<cs_mm_calib_handler> mm_calib,
            std::shared_ptr<enable_motion_correction> mm_correct_opt);
        rs2::frame process_frame(const rs2::frame_source& source, const rs2::frame& f) override;

    private:
        void correct_motion(rs2::frame* f);

        std::shared_ptr<enable_motion_correction> _mm_correct_opt = nullptr;
        float3x3            _accel_sensitivity;
        float3              _accel_bias;
        float3x3            _gyro_sensitivity;
        float3              _gyro_bias;
        float3x3            _imu2depth_cs_alignment_matrix;     // Transform and align raw IMU axis [x,y,z] to be consistent with the Depth frame CS
    };

    class cs_acceleration_transform : public cs_motion_transform
    {
    public:
        cs_acceleration_transform(std::shared_ptr<cs_mm_calib_handler> mm_calib = nullptr, std::shared_ptr<enable_motion_correction> mm_correct_opt = nullptr);

    protected:
        cs_acceleration_transform(const char* name, std::shared_ptr<cs_mm_calib_handler> mm_calib, std::shared_ptr<enable_motion_correction> mm_correct_opt);
        void process_function(byte * const dest[], const byte * source, int width, int height, int actual_size, int input_size) override;

    };

    class cs_gyroscope_transform : public cs_motion_transform
    {
    public:
        cs_gyroscope_transform(std::shared_ptr<cs_mm_calib_handler> mm_calib = nullptr, std::shared_ptr<enable_motion_correction> mm_correct_opt = nullptr);

    protected:
        cs_gyroscope_transform(const char* name, std::shared_ptr<cs_mm_calib_handler> mm_calib, std::shared_ptr<enable_motion_correction> mm_correct_opt);
        void process_function(byte * const dest[], const byte * source, int width, int height, int actual_size, int input_size) override;
    };
}
