// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 FRAMOS GmbH.

#include "cs/cs-timestamp.h"
#include "metadata.h"

namespace librealsense {
    cs_timestamp_reader_from_metadata::cs_timestamp_reader_from_metadata(std::unique_ptr<frame_timestamp_reader> backup_timestamp_reader)
            :_backup_timestamp_reader(std::move(backup_timestamp_reader)), _has_metadata(pins), one_time_note(false)
    {
        reset();
    }

    bool cs_timestamp_reader_from_metadata::has_metadata(const std::shared_ptr<frame_interface>& frame)
    {
        std::lock_guard<std::recursive_mutex> lock(_mtx);

        auto f = std::dynamic_pointer_cast<librealsense::frame>(frame);
        if (!f)
        {
            LOG_ERROR("Frame is not valid. Failed to downcast to librealsense::frame.");
            return false;
        }
        auto md = f->additional_data.metadata_blob;
        auto mds = f->additional_data.metadata_size;

        if (mds)
            return true;

        return false;
    }

    rs2_time_t cs_timestamp_reader_from_metadata::get_frame_timestamp(const std::shared_ptr<frame_interface>& frame)
    {
        std::lock_guard<std::recursive_mutex> lock(_mtx);

        auto f = std::dynamic_pointer_cast<librealsense::frame>(frame);
        if (!f)
        {
            LOG_ERROR("Frame is not valid. Failed to downcast to librealsense::frame.");
            return 0;
        }
        size_t pin_index = 0;

        if (frame->get_stream()->get_format() == RS2_FORMAT_Z16)
            pin_index = 1;

		_has_metadata[pin_index] = has_metadata(frame);

        auto md = (librealsense::metadata_framos_basic*)(f->additional_data.metadata_blob.data());
        if(_has_metadata[pin_index] && md)
        {
            // DEPRECATED FOR NOW - use timestamp data from smcs::IImageInfo preserve consistency with global timestamp reader which is using timestamp from camera (GevTimestampValue)
            // 
            // TODO NH - Providing metadata validity information through the "frame" would eliminate redundancy check if metadata is valid
            //if ((md->capture_timing.header.md_type_id == md_type::META_DATA_INTEL_CAPTURE_TIMING_ID) &&
            //    (md->capture_stats.header.md_type_id == md_type::META_DATA_CAPTURE_STATS_ID) &&
            //    (md->depth_control.header.md_type_id == md_type::META_DATA_INTEL_DEPTH_CONTROL_ID) &&
            //    (md->configuration.header.md_type_id == md_type::META_DATA_INTEL_CONFIGURATION_ID)) {
            //
            //    return (double)(md->header.timestamp) * TIMESTAMP_USEC_TO_MSEC;
            //}
            //else {

                // sensor_base::generate_frame_from_data - will update "backend_timestamp" with timestamp from smcs::IImageInfo converted to msec in cs_device::image_poll
                return (double)(f->additional_data.backend_timestamp);

            //}
        }
        else
        {
            if (!one_time_note)
            {
                LOG_WARNING("CS metadata payloads not available.");
                one_time_note = true;
            }
            return _backup_timestamp_reader->get_frame_timestamp(frame);
        }
    }

    unsigned long long cs_timestamp_reader_from_metadata::get_frame_counter(const std::shared_ptr<frame_interface>& frame) const
    {
        std::lock_guard<std::recursive_mutex> lock(_mtx);

        auto f = std::dynamic_pointer_cast<librealsense::frame>(frame);
        if (!f)
        {
            LOG_ERROR("Frame is not valid. Failed to downcast to librealsense::frame.");
            return 0;
        }
        size_t pin_index = 0;

        if (frame->get_stream()->get_format() == RS2_FORMAT_Z16)
            pin_index = 1;

        if(_has_metadata[pin_index] && f->additional_data.metadata_size > framos_uvc_header_size)
        {
            auto md = (librealsense::metadata_framos_basic*)(f->additional_data.metadata_blob.data());
            if (md->capture_valid())
            {
                return md->capture_timing.frame_counter;
            }
        }

        return _backup_timestamp_reader->get_frame_counter(frame);
    }

    void cs_timestamp_reader_from_metadata::reset()
    {
        std::lock_guard<std::recursive_mutex> lock(_mtx);
        one_time_note = false;
        for (auto i = 0; i < pins; ++i)
        {
            _has_metadata[i] = false;
        }
    }

    rs2_timestamp_domain cs_timestamp_reader_from_metadata::get_frame_timestamp_domain(const std::shared_ptr<frame_interface>& frame) const
    {
        std::lock_guard<std::recursive_mutex> lock(_mtx);
        auto pin_index = 0;
        if (frame->get_stream()->get_format() == RS2_FORMAT_Z16)
            pin_index = 1;

        return _has_metadata[pin_index] ? RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK :
               _backup_timestamp_reader->get_frame_timestamp_domain(frame);
    }

    cs_timestamp_reader::cs_timestamp_reader(std::shared_ptr <platform::time_service> ts)
            : counter(pins), _ts(ts) 
    {
        reset();
    }

    void cs_timestamp_reader::reset() {
        std::lock_guard <std::recursive_mutex> lock(_mtx);
        for (auto i = 0; i < pins; ++i)
        {
            counter[i] = 0;
        }
    }

    rs2_time_t cs_timestamp_reader::get_frame_timestamp(const std::shared_ptr<frame_interface>& frame)
    {
        std::lock_guard <std::recursive_mutex> lock(_mtx);
        return _ts->get_time();
    }

    unsigned long long cs_timestamp_reader::get_frame_counter(const std::shared_ptr<frame_interface>& frame) const
    {
        std::lock_guard <std::recursive_mutex> lock(_mtx);
        auto pin_index = 0;
        if (frame->get_stream()->get_format() == RS2_FORMAT_Z16) // Z16
            pin_index = 1;

        return ++counter[pin_index];
    }

    rs2_timestamp_domain cs_timestamp_reader::get_frame_timestamp_domain(const std::shared_ptr<frame_interface>& frame) const
    {
        return RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME;
    }
    
    cs_hid_timestamp_reader::cs_hid_timestamp_reader()
    {
        counter.resize(sensors);
        reset();
    }

    void cs_hid_timestamp_reader::reset()
    {
        std::lock_guard<std::recursive_mutex> lock(_mtx);
        started = false;
        for (auto i = 0; i < sensors; ++i) {
            counter[i] = 0;
        }
    }

    rs2_time_t cs_hid_timestamp_reader::get_frame_timestamp(const std::shared_ptr<frame_interface>& frame)
    {
        std::lock_guard<std::recursive_mutex> lock(_mtx);

        auto f = std::dynamic_pointer_cast<librealsense::frame>(frame);
        if (has_metadata(frame)) {

            // TODO - nh - remove comment bellow, valid only for Intel cameras
            //  The timestamps conversions path comprise of:
            // FW TS (32bit) ->    USB Phy Layer (no changes)  -> Host Driver TS (Extend to 64bit) ->  LRS read as 64 bit
            // The flow introduces discrepancy with UVC stream which timestamps are not extended to 64 bit by host driver both for Win and v4l backends.
            // In order to allow for hw timestamp-based synchronization of Depth and IMU streams the latter will be trimmed to 32 bit.
            // To revert to the extended 64 bit TS uncomment the next line instead
            //auto timestamp = *((uint64_t*)((const uint8_t*)fo.metadata));

            // The ternary operator is replaced by explicit assignment due to an issue with GCC for RaspberryPi that causes segfauls in optimized build.
            auto timestamp = *(reinterpret_cast<uint32_t*>(f->additional_data.metadata_blob.data()));
            if (f->additional_data.metadata_size >= platform::hid_header_size) {
                timestamp = static_cast<uint32_t>(reinterpret_cast<const platform::hid_header*>(f->additional_data.metadata_blob.data())->timestamp);
            }

            // Should be converted to ms in cs_device::event_monitor() (cs-sensor-motion.cpp)
            //return static_cast<rs2_time_t>(timestamp * TIMESTAMP_USEC_TO_MSEC);
            return static_cast<rs2_time_t>(timestamp);
        }

        if (!started) {
            LOG_WARNING("CS HID timestamp not found, switching to Host timestamps.");
            started = true;
        }

        return std::chrono::duration<rs2_time_t, std::milli>(std::chrono::system_clock::now().time_since_epoch()).count();
    }

    bool cs_hid_timestamp_reader::has_metadata(const std::shared_ptr<frame_interface>& frame) const
    {
        auto f = std::dynamic_pointer_cast<librealsense::frame>(frame);

        if (f->additional_data.metadata_size > 0) {
            return true;
        }

        return false;
    }

    unsigned long long cs_hid_timestamp_reader::get_frame_counter(const std::shared_ptr<frame_interface>& frame) const
    {
        std::lock_guard<std::recursive_mutex> lock(_mtx);

        int index = 0;
        if (frame->get_stream()->get_stream_type() == RS2_STREAM_GYRO) {
            index = 1;
        }

        return ++counter[index];
    }

    rs2_timestamp_domain cs_hid_timestamp_reader::get_frame_timestamp_domain(const std::shared_ptr<frame_interface>& frame) const
    {
        if (has_metadata(frame)) {
            return RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK;
        }

        return RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME;
    }
}
