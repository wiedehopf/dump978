// -*- c++ -*-

// Copyright (c) 2019, FlightAware LLC.
// All rights reserved.
// Licensed under the 2-clause BSD license; see the LICENSE file

#ifndef DUMP978_SOAPY_SOURCE_H
#define DUMP978_SOAPY_SOURCE_H

#include <atomic>
#include <memory>
#include <thread>

#include <SoapySDR/Device.hpp>

#include "sample_source.h"

namespace dump978 {
    class SoapySampleSource : public SampleSource {
      public:
        static SampleSource::Pointer Create(const std::string &device_name, const boost::program_options::variables_map &options) { return Pointer(new SoapySampleSource(service, device_name, options)); }

        virtual ~SoapySampleSource();

        void Init() override;
        void Start() override;
        void Stop() override;
        SampleFormat Format() override { return format_; }

      private:
        SoapySampleSource(const std::string &device_name, const boost::program_options::variables_map &options);

        void Run();

        SampleFormat format_ = SampleFormat::UNKNOWN;
        std::string device_name_;
        boost::program_options::variables_map options_;

        std::shared_ptr<SoapySDR::Device> device_;
        std::shared_ptr<SoapySDR::Stream> stream_;
        std::unique_ptr<std::thread> rx_thread_;
        bool halt_ = false;

        static std::atomic_bool log_handler_registered_;
    };
}; // namespace dump978

#endif
