// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015 Intel Corporation. All Rights Reserved.

#include "environment.h"
#include "core/video.h"
#include "core/motion.h"
#include "device.h"

using namespace librealsense;
#ifdef FRAMOS
std::shared_ptr< matcher > matcher_factory::create( rs2_matchers matcher,
                                                  std::vector< stream_interface * > const & profiles, rs2_syncer_mode syncer_mode)
{
    switch (matcher)
    {
    case RS2_MATCHER_DI:
        return create_DI_matcher(profiles, syncer_mode);
    case RS2_MATCHER_DI_C:
        return create_DI_C_matcher(profiles, syncer_mode);
    case RS2_MATCHER_DLR_C:
        return create_DLR_C_matcher(profiles, syncer_mode);
    case RS2_MATCHER_DLR:
        return create_DLR_matcher(profiles, syncer_mode);
    case RS2_MATCHER_DIC:
        return create_DIC_matcher( profiles, syncer_mode );
    case RS2_MATCHER_DIC_C:
        return create_DIC_C_matcher( profiles, syncer_mode );
    case RS2_MATCHER_DEFAULT:
    default:
        return create_timestamp_matcher(profiles, syncer_mode);
    }
}
#endif
#ifndef FRAMOS
std::shared_ptr< matcher > matcher_factory::create( rs2_matchers matcher,
                                                    std::vector< stream_interface * > const & profiles )
{
    switch (matcher)
    {
    case RS2_MATCHER_DI:
        return create_DI_matcher(profiles);
    case RS2_MATCHER_DI_C:
        return create_DI_C_matcher(profiles);
    case RS2_MATCHER_DLR_C:
        return create_DLR_C_matcher(profiles);
    case RS2_MATCHER_DLR:
        return create_DLR_matcher(profiles);
    case RS2_MATCHER_DIC:
        return create_DIC_matcher( profiles );
    case RS2_MATCHER_DIC_C:
        return create_DIC_C_matcher( profiles );
    case RS2_MATCHER_DEFAULT:
    default:
        return create_timestamp_matcher(profiles);
    }
}
#endif

stream_interface *
librealsense::find_profile( rs2_stream stream, int index, std::vector< stream_interface * > const & profiles )
{
    auto prof = std::find_if(profiles.begin(), profiles.end(), [&](stream_interface* profile)
    {
        return profile->get_stream_type() == stream && ( index == -1 || profile->get_stream_index() == index );
    });

    if (prof != profiles.end())
        return *prof;
    else
        return nullptr;
}

#ifdef FRAMOS
std::shared_ptr< matcher > matcher_factory::create_DLR_C_matcher( std::vector< stream_interface * > const & profiles, rs2_syncer_mode syncer_mode)
{
    auto color  = find_profile(RS2_STREAM_COLOR, 0, profiles);
    if (!color)
    {
        LOG_DEBUG("Created default matcher");
        return create_timestamp_matcher(profiles, syncer_mode);
    }

    return create_timestamp_composite_matcher({ create_DLR_matcher(profiles),
        create_identity_matcher(color) }, syncer_mode);
}
#endif
#ifndef FRAMOS
std::shared_ptr< matcher > matcher_factory::create_DLR_C_matcher( std::vector< stream_interface * > const & profiles )
{
    auto color  = find_profile(RS2_STREAM_COLOR, 0, profiles);
    if (!color)
    {
        LOG_DEBUG("Created default matcher");
        return create_timestamp_matcher(profiles);
    }

    return create_timestamp_composite_matcher({ create_DLR_matcher(profiles),
        create_identity_matcher(color) });
}
#endif

#ifdef FRAMOS
std::shared_ptr <matcher > matcher_factory::create_DI_C_matcher( std::vector< stream_interface * > const & profiles , rs2_syncer_mode syncer_mode)
{
    auto color = find_profile(RS2_STREAM_COLOR, 0, profiles);
    if (!color)
    {
        LOG_DEBUG("Created default matcher");
        return create_timestamp_matcher(profiles, syncer_mode);
    }

    return create_timestamp_composite_matcher({ create_DI_matcher(profiles),
        create_identity_matcher(color) }, syncer_mode);
}
#endif
#ifndef FRAMOS
std::shared_ptr< matcher > matcher_factory::create_DI_C_matcher( std::vector< stream_interface * > const & profiles )
{
    auto color = find_profile(RS2_STREAM_COLOR, 0, profiles);
    if (!color)
    {
        LOG_DEBUG("Created default matcher");
        return create_timestamp_matcher(profiles);
    }

    return create_timestamp_composite_matcher({ create_DI_matcher(profiles),
        create_identity_matcher(color) });
}
#endif

#ifdef FRAMOS
std::shared_ptr< matcher > matcher_factory::create_DLR_matcher( std::vector< stream_interface * > const & profiles, rs2_syncer_mode syncer_mode)
{
    auto depth = find_profile(RS2_STREAM_DEPTH, 0, profiles);
    auto left = find_profile(RS2_STREAM_INFRARED, 1, profiles);
    auto right = find_profile(RS2_STREAM_INFRARED, 2, profiles);

    if (!depth || !left || !right)
    {
        LOG_DEBUG("Created default matcher");
        return create_timestamp_matcher(profiles, syncer_mode);
    }
    return create_frame_number_matcher({ depth , left , right });
}
#endif

#ifndef FRAMOS
std::shared_ptr< matcher > matcher_factory::create_DLR_matcher( std::vector< stream_interface * > const & profiles )
{
    auto depth = find_profile(RS2_STREAM_DEPTH, 0, profiles);
    auto left = find_profile(RS2_STREAM_INFRARED, 1, profiles);
    auto right = find_profile(RS2_STREAM_INFRARED, 2, profiles);

    if (!depth || !left || !right)
    {
        LOG_DEBUG("Created default matcher");
        return create_timestamp_matcher(profiles);
    }
    return create_frame_number_matcher({ depth , left , right });
}
#endif

#ifdef FRAMOS
std::shared_ptr< matcher > matcher_factory::create_DI_matcher( std::vector< stream_interface * > const & profiles, rs2_syncer_mode syncer_mode )
{
    auto depth = find_profile(RS2_STREAM_DEPTH, 0, profiles);
    auto ir = find_profile(RS2_STREAM_INFRARED, 1, profiles);

    if (!depth || !ir)
    {
        LOG_DEBUG("Created default matcher");
        return create_timestamp_matcher(profiles, syncer_mode);
    }
    return create_frame_number_matcher({ depth , ir });
}
#endif

#ifndef FRAMOS
std::shared_ptr< matcher > matcher_factory::create_DI_matcher( std::vector< stream_interface * > const & profiles )
{
    auto depth = find_profile(RS2_STREAM_DEPTH, 0, profiles);
    auto ir = find_profile(RS2_STREAM_INFRARED, 1, profiles);

    if (!depth || !ir)
    {
        LOG_DEBUG("Created default matcher");
        return create_timestamp_matcher(profiles);
    }
    return create_frame_number_matcher({ depth , ir });
}
#endif

#ifdef FRAMOS
std::shared_ptr< matcher > matcher_factory::create_DIC_matcher( std::vector< stream_interface * > const & profiles, rs2_syncer_mode syncer_mode )
{
    std::vector< std::shared_ptr< matcher > > matchers;
    if( auto depth = find_profile( RS2_STREAM_DEPTH, -1, profiles ) )
        matchers.push_back( create_identity_matcher( depth ));
    if( auto ir = find_profile( RS2_STREAM_INFRARED, -1, profiles ) )
        matchers.push_back( create_identity_matcher( ir ) );
    if( auto confidence = find_profile( RS2_STREAM_CONFIDENCE, -1, profiles ) )
        matchers.push_back( create_identity_matcher( confidence ) );

    if( matchers.empty() )
    {
        LOG_ERROR( "no depth, ir, or confidence streams found for matcher" );
        for( auto && p : profiles )
            LOG_DEBUG( p->get_stream_type() << '/' << p->get_unique_id() );
        throw std::runtime_error( "no depth, ir, or confidence streams found for matcher" );
    }

    return create_timestamp_composite_matcher( matchers, syncer_mode );
}
#endif

#ifndef FRAMOS
std::shared_ptr< matcher > matcher_factory::create_DIC_matcher( std::vector< stream_interface * > const & profiles )
{
    std::vector< std::shared_ptr< matcher > > matchers;
    if( auto depth = find_profile( RS2_STREAM_DEPTH, -1, profiles ) )
        matchers.push_back( create_identity_matcher( depth ));
    if( auto ir = find_profile( RS2_STREAM_INFRARED, -1, profiles ) )
        matchers.push_back( create_identity_matcher( ir ) );
    if( auto confidence = find_profile( RS2_STREAM_CONFIDENCE, -1, profiles ) )
        matchers.push_back( create_identity_matcher( confidence ) );

    if( matchers.empty() )
    {
        LOG_ERROR( "no depth, ir, or confidence streams found for matcher" );
        for( auto && p : profiles )
            LOG_DEBUG( p->get_stream_type() << '/' << p->get_unique_id() );
        throw std::runtime_error( "no depth, ir, or confidence streams found for matcher" );
    }

    return create_timestamp_composite_matcher( matchers );
}
#endif

#ifdef FRAMOS
std::shared_ptr< matcher > matcher_factory::create_DIC_C_matcher( std::vector< stream_interface * > const & profiles, rs2_syncer_mode syncer_mode )
{
    auto color = find_profile( RS2_STREAM_COLOR, 0, profiles );
    if( ! color )
        throw std::runtime_error( "no color stream found for matcher" );

    return create_timestamp_composite_matcher( { create_DIC_matcher( profiles ), create_identity_matcher( color ) }, syncer_mode );
}
#endif

#ifndef FRAMOS
std::shared_ptr< matcher > matcher_factory::create_DIC_C_matcher( std::vector< stream_interface * > const & profiles )
{
    auto color = find_profile( RS2_STREAM_COLOR, 0, profiles );
    if( ! color )
        throw std::runtime_error( "no color stream found for matcher" );

    return create_timestamp_composite_matcher( { create_DIC_matcher( profiles ), create_identity_matcher( color ) } );
}
#endif

std::shared_ptr< matcher >
matcher_factory::create_frame_number_matcher( std::vector< stream_interface * > const & profiles )
{
    std::vector<std::shared_ptr<matcher>> matchers;
    for (auto& p : profiles)
        matchers.push_back(std::make_shared<identity_matcher>(p->get_unique_id(), p->get_stream_type()));

    return create_frame_number_composite_matcher(matchers);
}

#ifdef FRAMOS
std::shared_ptr< matcher >
matcher_factory::create_timestamp_matcher( std::vector< stream_interface * > const & profiles, rs2_syncer_mode syncer_mode )
{
    std::vector<std::shared_ptr<matcher>> matchers;
    for (auto& p : profiles)
        matchers.push_back(std::make_shared<identity_matcher>(p->get_unique_id(), p->get_stream_type()));

    return create_timestamp_composite_matcher(matchers, syncer_mode);
}
#endif
#ifndef FRAMOS
std::shared_ptr< matcher >
matcher_factory::create_timestamp_matcher( std::vector< stream_interface * > const & profiles )
{
    std::vector<std::shared_ptr<matcher>> matchers;
    for (auto& p : profiles)
        matchers.push_back(std::make_shared<identity_matcher>(p->get_unique_id(), p->get_stream_type()));

    return create_timestamp_composite_matcher(matchers);
}
#endif

std::shared_ptr<matcher> matcher_factory::create_identity_matcher(stream_interface *profile)
{
    return std::make_shared<identity_matcher>(profile->get_unique_id(), profile->get_stream_type());
}

std::shared_ptr< matcher >
matcher_factory::create_frame_number_composite_matcher( std::vector< std::shared_ptr< matcher > > const & matchers )
{
    return std::make_shared<frame_number_composite_matcher>(matchers);
}

#ifdef FRAMOS
std::shared_ptr< matcher >
matcher_factory::create_timestamp_composite_matcher( std::vector< std::shared_ptr< matcher > > const & matchers, rs2_syncer_mode syncer_mode )
{
    return std::make_shared<timestamp_composite_matcher>(matchers, syncer_mode);
}
#endif

#ifndef FRAMOS
std::shared_ptr< matcher >
matcher_factory::create_timestamp_composite_matcher( std::vector< std::shared_ptr< matcher > > const & matchers )
{
    return std::make_shared<timestamp_composite_matcher>(matchers);
}
#endif

device::device(std::shared_ptr<context> ctx,
               const platform::backend_device_group group,
               bool device_changed_notifications)
    : _context(ctx), _group(group), _is_valid(true),
      _device_changed_notifications(device_changed_notifications),
      _is_alive(std::make_shared<bool>(true))
{
    _profiles_tags = lazy<std::vector<tagged_profile>>([this]() { return get_profiles_tags(); });

    if (_device_changed_notifications)
    {
        std::weak_ptr<bool> weak = _is_alive;
        auto cb = new devices_changed_callback_internal([this, weak](rs2_device_list* removed, rs2_device_list* added)
        {
            // The callback can be called from one thread while the object is being destroyed by another.
            // Check if members can still be accessed.
            auto alive = weak.lock();
            if (!alive || !*alive)
                return;

            // Update is_valid variable when device is invalid
            std::lock_guard<std::mutex> lock(_device_changed_mtx);
            for (auto& dev_info : removed->list)
            {
                if (dev_info.info->get_device_data() == _group)
                {
                    _is_valid = false;
                    return;
                }
            }
        });

        _callback_id = _context->register_internal_device_callback({ cb, [](rs2_devices_changed_callback* p) { p->release(); } });
    }
}

device::~device()
{
    *_is_alive = false;

    if (_device_changed_notifications)
    {
        _context->unregister_internal_device_callback(_callback_id);
    }
    _sensors.clear();
}

int device::add_sensor(const std::shared_ptr<sensor_interface>& sensor_base)
{
    _sensors.push_back(sensor_base);
    return (int)_sensors.size() - 1;
}

int device::assign_sensor(const std::shared_ptr<sensor_interface>& sensor_base, uint8_t idx)
{
    try
    {
        _sensors[idx] = sensor_base;
        return (int)_sensors.size() - 1;
    }
    catch (std::out_of_range)
    {
        throw invalid_value_exception(to_string() << "Cannot assign sensor - invalid subdevice value" << idx);
    }
}

size_t device::get_sensors_count() const
{
    return static_cast<unsigned int>(_sensors.size());
}

sensor_interface& device::get_sensor(size_t subdevice)
{
    try
    {
        return *(_sensors.at(subdevice));
    }
    catch (std::out_of_range)
    {
        throw invalid_value_exception("invalid subdevice value");
    }
}

size_t device::find_sensor_idx(const sensor_interface& s) const
{
    int idx = 0;
    for (auto&& sensor : _sensors)
    {
        if (&s == sensor.get()) return idx;
        idx++;
    }
    throw std::runtime_error("Sensor not found!");
}

const sensor_interface& device::get_sensor(size_t subdevice) const
{
    try
    {
        return *(_sensors.at(subdevice));
    }
    catch (std::out_of_range)
    {
        throw invalid_value_exception("invalid subdevice value");
    }
}

void device::hardware_reset()
{
    throw not_implemented_exception(to_string() << __FUNCTION__ << " is not implemented for this device!");
}

std::shared_ptr<matcher> device::create_matcher(const frame_holder& frame) const
{

    return std::make_shared<identity_matcher>( frame.frame->get_stream()->get_unique_id(), frame.frame->get_stream()->get_stream_type());
}

std::pair<uint32_t, rs2_extrinsics> device::get_extrinsics(const stream_interface& stream) const
{
    auto stream_index = stream.get_unique_id();
    auto pair = _extrinsics.at(stream_index);
    auto pin_stream = pair.second;
    rs2_extrinsics ext{};
    if (environment::get_instance().get_extrinsics_graph().try_fetch_extrinsics(*pin_stream, stream, &ext) == false)
    {
        throw std::runtime_error(to_string() << "Failed to fetch extrinsics between pin stream (" << pin_stream->get_unique_id() << ") to given stream (" << stream.get_unique_id() << ")");
    }
    return std::make_pair(pair.first, ext);
}

void device::register_stream_to_extrinsic_group(const stream_interface& stream, uint32_t group_index)
{
    auto iter = std::find_if(_extrinsics.begin(),
                           _extrinsics.end(),
                           [group_index](const std::pair<int, std::pair<uint32_t, std::shared_ptr<const stream_interface>>>& p) { return p.second.first == group_index; });
    if (iter == _extrinsics.end())
    {
        //First stream to register for this group
        _extrinsics[stream.get_unique_id()] = std::make_pair(group_index, stream.shared_from_this());
    }
    else
    {
        //iter->second holds the group_id and the key stream
        _extrinsics[stream.get_unique_id()] = iter->second;
    }
}

std::vector<rs2_format> device::map_supported_color_formats(rs2_format source_format)
{
    // Mapping from source color format to all of the compatible target color formats.

    std::vector<rs2_format> target_formats = { RS2_FORMAT_RGB8, RS2_FORMAT_RGBA8, RS2_FORMAT_BGR8, RS2_FORMAT_BGRA8 };
    switch (source_format)
    {
    case RS2_FORMAT_YUYV:
        target_formats.push_back(RS2_FORMAT_YUYV);
        target_formats.push_back(RS2_FORMAT_Y16);
        break;
    case RS2_FORMAT_UYVY:
        target_formats.push_back(RS2_FORMAT_UYVY);
        break;
    default:
        LOG_ERROR("Format is not supported for mapping");
    }
    return target_formats;
}

void device::tag_profiles(stream_profiles profiles) const
{
    for (auto profile : profiles)
    {
        for (auto tag : *_profiles_tags)
        {
            if (auto vp = dynamic_cast<video_stream_profile_interface*>(profile.get()))
            {
                if ((tag.stream == RS2_STREAM_ANY || vp->get_stream_type() == tag.stream) &&
                    (tag.format == RS2_FORMAT_ANY || vp->get_format() == tag.format) &&
                    (tag.width == -1 || vp->get_width() == tag.width) &&
                    (tag.height == -1 || vp->get_height() == tag.height) &&
                    (tag.fps == -1 || vp->get_framerate() == tag.fps) &&
                    (tag.stream_index == -1 || vp->get_stream_index() == tag.stream_index))
                    profile->tag_profile(tag.tag);
            }
            else
            if (auto mp = dynamic_cast<motion_stream_profile_interface*>(profile.get()))
            {
                if ((tag.stream == RS2_STREAM_ANY || mp->get_stream_type() == tag.stream) &&
                    (tag.format == RS2_FORMAT_ANY || mp->get_format() == tag.format) &&
                    (tag.fps == -1 || mp->get_framerate() == tag.fps) &&
                    (tag.stream_index == -1 || mp->get_stream_index() == tag.stream_index))
                    profile->tag_profile(tag.tag);
            }
        }
    }
}

bool device::contradicts(const stream_profile_interface* a, const std::vector<stream_profile>& others) const
{
    if (auto vid_a = dynamic_cast<const video_stream_profile_interface*>(a))
    {
        for (auto request : others)
        {
            if (a->get_framerate() != 0 && request.fps != 0 && (a->get_framerate() != request.fps))
                return true;
            if (vid_a->get_width() != 0 && request.width != 0 && (vid_a->get_width() != request.width))
                return true;
            if (vid_a->get_height() != 0 && request.height != 0 && (vid_a->get_height() != request.height))
                return true;
        }
    }
    return false;
}

void device::stop_activity() const
{
    for (auto& sensor : _sensors)
    {
        auto snr_name = (sensor->supports_info(RS2_CAMERA_INFO_NAME)) ? sensor->get_info(RS2_CAMERA_INFO_NAME) : "";

        // Disable asynchronous services
        for (auto& opt : sensor->get_supported_options())
        {
            if (val_in_range(opt, { RS2_OPTION_GLOBAL_TIME_ENABLED, RS2_OPTION_ERROR_POLLING_ENABLED }))
            {
                try
                {
                    // enumerated options use zero or positive values
                    if (sensor->get_option(opt).query() > 0.f)
                        sensor->get_option(opt).set(false);
                }
                catch (...)
                {
                    LOG_ERROR("Failed to toggle off " << opt << " [" << snr_name << "]");
                }
            }
        }

        // Stop UVC/HID streaming
        try
        {
            if (sensor->is_streaming())
            {
                sensor->stop();
                sensor->close();
            }
        }
        catch (const wrong_api_call_sequence_exception& exc)
        {
            LOG_WARNING("Out of order stop/close invocation for " << snr_name << ": " << exc.what());
        }
        catch (...)
        {
            auto snr_name = (sensor->supports_info(RS2_CAMERA_INFO_NAME)) ? sensor->get_info(RS2_CAMERA_INFO_NAME) : "";
            LOG_ERROR("Failed to deactivate " << snr_name);
        }
    }
}
