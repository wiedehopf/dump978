// -*- c++ -*-

// Copyright (c) 2019, FlightAware LLC.
// All rights reserved.
// Licensed under the 2-clause BSD license; see the LICENSE file

#ifndef DUMP978_SAMPLE_SOURCE_H
#define DUMP978_SAMPLE_SOURCE_H

#include <chrono>
#include <fstream>
#include <functional>
#include <memory>

#include <boost/asio/io_service.hpp>
#include <boost/asio/posix/stream_descriptor.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/filesystem.hpp>

#include "common.h"
#include "convert.h"

namespace dump978 {
    class SampleSource : public std::enable_shared_from_this<SampleSource> {
      public:
        typedef std::shared_ptr<SampleSource> Pointer;
        typedef std::function<void(std::uint64_t, const uat::Bytes &, const boost::system::error_code &ec)> Consumer;

        virtual ~SampleSource() {}

        virtual void Start() = 0;
        virtual void Stop() = 0;

        void SetConsumer(Consumer consumer) { consumer_ = consumer; }

      protected:
        SampleSource() {}

        void DispatchBuffer(std::uint64_t timestamp, const uat::Bytes &buffer) {
            if (consumer_) {
                consumer_(timestamp, buffer, boost::system::error_code());
            }
        }

        void DispatchError(const boost::system::error_code &ec) {
            if (consumer_) {
                consumer_(0, uat::Bytes(), ec);
            }
        }

      private:
        Consumer consumer_;
    };

    class FileSampleSource : public SampleSource {
      public:
        static SampleSource::Pointer Create(boost::asio::io_service &service, const boost::filesystem::path &path, SampleFormat format, bool throttle, std::size_t samples_per_second = 2083333, std::size_t samples_per_block = 524288) { return Pointer(new FileSampleSource(service, path, format, throttle, samples_per_second, samples_per_block)); }

        void Start() override;
        void Stop() override;

      private:
        FileSampleSource(boost::asio::io_service &service, const boost::filesystem::path &path, SampleFormat format, bool throttle, std::size_t samples_per_second, std::size_t samples_per_block) : service_(service), path_(path), alignment_(BytesPerSample(format)), throttle_(throttle), bytes_per_second_(samples_per_second * alignment_), timer_(service) { block_.reserve(samples_per_block * alignment_); }

        void ReadBlock(const boost::system::error_code &ec);

        boost::asio::io_service &service_;
        boost::filesystem::path path_;
        unsigned alignment_;
        bool throttle_;
        std::size_t bytes_per_second_;

        std::ifstream stream_;
        boost::asio::steady_timer timer_;
        std::chrono::steady_clock::time_point next_block_;
        uat::Bytes block_;
        std::uint64_t timestamp_;
    };

    class StdinSampleSource : public SampleSource {
      public:
        static SampleSource::Pointer Create(boost::asio::io_service &service, SampleFormat format, std::size_t samples_per_second = 2083333, std::size_t samples_per_block = 524288) { return Pointer(new StdinSampleSource(service, format, samples_per_second, samples_per_block)); }

        void Start() override;
        void Stop() override;

      private:
        StdinSampleSource(boost::asio::io_service &service, SampleFormat format, std::size_t samples_per_second, std::size_t samples_per_block) : service_(service), alignment_(BytesPerSample(format)), samples_per_second_(samples_per_second), stream_(service), used_(0) { block_.resize(samples_per_block * alignment_); }

        void ScheduleRead();

        boost::asio::io_service &service_;
        unsigned alignment_;
        std::size_t samples_per_second_;
        boost::asio::posix::stream_descriptor stream_;
        uat::Bytes block_;
        std::size_t used_;
    };
}; // namespace dump978

#endif
