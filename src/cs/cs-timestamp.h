// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 FRAMOS GmbH.

#ifndef LIBREALSENSE2_CS_TIMESTAMP_H
#define LIBREALSENSE2_CS_TIMESTAMP_H

#include "sensor.h"

namespace librealsense
{
    class cs_timestamp_reader_from_metadata : public frame_timestamp_reader
    {
        std::unique_ptr<frame_timestamp_reader> _backup_timestamp_reader;
        static const int pins = 2;
        std::vector<std::atomic<bool>> _has_metadata;
        bool one_time_note;
        mutable std::recursive_mutex _mtx;

    public:
        cs_timestamp_reader_from_metadata(std::unique_ptr<frame_timestamp_reader> backup_timestamp_reader);

        bool has_metadata(const std::shared_ptr<frame_interface>& frame);

        rs2_time_t get_frame_timestamp(const std::shared_ptr<frame_interface>& frame) override;

        unsigned long long get_frame_counter(const std::shared_ptr<frame_interface>& frame) const override;

        void reset() override;

        rs2_timestamp_domain get_frame_timestamp_domain(const std::shared_ptr<frame_interface>& frame) const override;
    };

    class cs_timestamp_reader : public frame_timestamp_reader
    {
    private:
        static const int pins = 2;
        mutable std::vector<int64_t> counter;
        std::shared_ptr<platform::time_service> _ts;
        mutable std::recursive_mutex _mtx;

    public:
        cs_timestamp_reader(std::shared_ptr<platform::time_service> ts);

        void reset() override;

        rs2_time_t get_frame_timestamp(const std::shared_ptr<frame_interface>& frame) override;

        unsigned long long get_frame_counter(const std::shared_ptr<frame_interface>& frame) const override;

        rs2_timestamp_domain get_frame_timestamp_domain(const std::shared_ptr<frame_interface>& frame) const override;
    };

    class cs_hid_timestamp_reader : public frame_timestamp_reader
    {
    private:
        static const int sensors = 2;
        bool started;
        mutable std::vector<int64_t> counter;
        mutable std::recursive_mutex _mtx;

    public:
        cs_hid_timestamp_reader();

        void reset() override;

        rs2_time_t get_frame_timestamp(const std::shared_ptr<frame_interface>& frame) override;

        bool has_metadata(const std::shared_ptr<frame_interface>& frame) const;

        unsigned long long get_frame_counter(const std::shared_ptr<frame_interface>& frame) const override;

        rs2_timestamp_domain get_frame_timestamp_domain(const std::shared_ptr<frame_interface>& frame) const override;
    };
}

#endif //LIBREALSENSE2_CS_TIMESTAMP_H
