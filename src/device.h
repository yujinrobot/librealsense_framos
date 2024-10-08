// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015 Intel Corporation. All Rights Reserved.

#pragma once

#include <chrono>
#include <memory>
#include <vector>

#include "backend.h"
#include "archive.h"
#include "hw-monitor.h"
#include "option.h"
#include "sensor.h"
#include "sync.h"
#include "core/streaming.h"

#include "context.h"

namespace librealsense
{

    
stream_interface * find_profile( rs2_stream stream, int index, std::vector< stream_interface * > const & profiles );

class matcher_factory
{
public:
    #ifdef FRAMOS
    static std::shared_ptr< matcher > create( rs2_matchers matcher,
                                                std::vector< stream_interface * > const & profiles,  rs2_syncer_mode syncer_mode = RS2_SYNCER_MODE_DEFAULT );
    #endif
    #ifndef FRAMOS
    static std::shared_ptr< matcher > create( rs2_matchers matcher,
                                                std::vector< stream_interface * > const & profiles );
    #endif

private:
    #ifdef FRAMOS
    static std::shared_ptr< matcher > create_DLR_C_matcher( std::vector< stream_interface * > const & profiles, rs2_syncer_mode syncer_mode = RS2_SYNCER_MODE_DEFAULT );
    static std::shared_ptr< matcher > create_DLR_matcher( std::vector< stream_interface * > const & profiles, rs2_syncer_mode syncer_mode = RS2_SYNCER_MODE_DEFAULT );
    static std::shared_ptr< matcher > create_DI_C_matcher( std::vector< stream_interface * > const & profiles, rs2_syncer_mode syncer_mode = RS2_SYNCER_MODE_DEFAULT );
    static std::shared_ptr< matcher > create_DI_matcher( std::vector< stream_interface * > const & profiles, rs2_syncer_mode syncer_mode = RS2_SYNCER_MODE_DEFAULT );
    static std::shared_ptr< matcher > create_DIC_matcher( std::vector< stream_interface* > const& profiles, rs2_syncer_mode syncer_mode = RS2_SYNCER_MODE_DEFAULT );
    static std::shared_ptr< matcher > create_DIC_C_matcher( std::vector< stream_interface * > const & profiles, rs2_syncer_mode syncer_mode = RS2_SYNCER_MODE_DEFAULT);
    #endif
    #ifndef FRAMOS
    static std::shared_ptr< matcher > create_DLR_C_matcher( std::vector< stream_interface * > const & profiles );
    static std::shared_ptr< matcher > create_DLR_matcher( std::vector< stream_interface * > const & profiles );
    static std::shared_ptr< matcher > create_DI_C_matcher( std::vector< stream_interface * > const & profiles );
    static std::shared_ptr< matcher > create_DI_matcher( std::vector< stream_interface * > const & profiles );
    static std::shared_ptr< matcher > create_DIC_matcher( std::vector< stream_interface* > const& profiles );
    static std::shared_ptr< matcher > create_DIC_C_matcher( std::vector< stream_interface * > const & profiles );
    #endif
    static std::shared_ptr< matcher > create_identity_matcher( stream_interface * profiles );
    static std::shared_ptr< matcher > create_frame_number_matcher( std::vector< stream_interface * > const & profiles );

    #ifdef FRAMOS
    static std::shared_ptr< matcher > create_timestamp_matcher( std::vector< stream_interface * > const & profiles, rs2_syncer_mode syncer_mode = RS2_SYNCER_MODE_DEFAULT );
    static std::shared_ptr< matcher > create_timestamp_composite_matcher( std::vector< std::shared_ptr< matcher > > const & matchers, rs2_syncer_mode syncer_mode = RS2_SYNCER_MODE_DEFAULT);
    #endif
    #ifndef FRAMOS
    static std::shared_ptr< matcher > create_timestamp_matcher( std::vector< stream_interface * > const & profiles );
    static std::shared_ptr< matcher > create_timestamp_composite_matcher( std::vector< std::shared_ptr< matcher > > const & matchers );
    #endif
    static std::shared_ptr< matcher > create_frame_number_composite_matcher( std::vector< std::shared_ptr< matcher > > const & matchers );

    #ifdef FRAMOS
    private: 
        rs2_syncer_mode _syncer_mode;
    #endif
    };
class device : public virtual device_interface, public info_container
{
public:
    virtual ~device();
    size_t get_sensors_count() const override;

    sensor_interface& get_sensor(size_t subdevice) override;
    const sensor_interface& get_sensor(size_t subdevice) const override;

    void hardware_reset() override;

    virtual std::shared_ptr<matcher> create_matcher(const frame_holder& frame) const override;

    size_t find_sensor_idx(const sensor_interface& s) const;

    std::shared_ptr<context> get_context() const override {
        return _context;
    }

    platform::backend_device_group get_device_data() const override
    {
        return _group;
    }

    std::pair<uint32_t, rs2_extrinsics> get_extrinsics(const stream_interface& stream) const override;

    bool is_valid() const override
    {
        std::lock_guard<std::mutex> lock(_device_changed_mtx);
        return _is_valid;
    }

    void tag_profiles(stream_profiles profiles) const override;

    virtual bool compress_while_record() const override { return true; }

    virtual bool contradicts(const stream_profile_interface* a, const std::vector<stream_profile>& others) const override;

    virtual void stop_activity() const;

    bool device_changed_notifications_on() const { return _device_changed_notifications; }

protected:
    int add_sensor(const std::shared_ptr<sensor_interface>& sensor_base);
    int assign_sensor(const std::shared_ptr<sensor_interface>& sensor_base, uint8_t idx);
    void register_stream_to_extrinsic_group(const stream_interface& stream, uint32_t groupd_index);
    std::vector<rs2_format> map_supported_color_formats(rs2_format source_format);

    explicit device(std::shared_ptr<context> ctx,
                    const platform::backend_device_group group,
                    bool device_changed_notifications = true);

    std::map<int, std::pair<uint32_t, std::shared_ptr<const stream_interface>>> _extrinsics;

private:
    std::vector<std::shared_ptr<sensor_interface>> _sensors;
    std::shared_ptr<context> _context;
    const platform::backend_device_group _group;
    bool _is_valid, _device_changed_notifications;
    mutable std::mutex _device_changed_mtx;
    uint64_t _callback_id;
    lazy<std::vector<tagged_profile>> _profiles_tags;

    std::shared_ptr<bool> _is_alive; // Ensures object can be accessed
};

// Helper function that should be used when multiple FW calls needs to be made.
// This function change the USB power to D0 (Operational) using the invoke_power function
// activate the received function and power down the state to D3 (Idle)
template<class T>
auto group_multiple_fw_calls(synthetic_sensor& s, T action)
-> decltype(action())
{
    auto& us = dynamic_cast<uvc_sensor&>(*s.get_raw_sensor());

    return us.invoke_powered([&](platform::uvc_device& dev) { return action(); });
}
}
