// -*- c++ -*-

// Copyright (c) 2019, FlightAware LLC.
// All rights reserved.
// Licensed under the 2-clause BSD license; see the LICENSE file

#ifndef DUMP978_SOAPY_SOURCE_H
#define DUMP978_SOAPY_SOURCE_H

#include <memory>
#include <thread>
#include <atomic>

#include <SoapySDR/Device.hpp>

#include "sample_source.h"

namespace dump978 {
    class SoapySampleSource : public SampleSource {
    public:
        static SampleSource::Pointer Create(SampleFormat format,
                                            const std::string &device_name) {
            return Pointer(new SoapySampleSource(format, device_name));
        }

        virtual ~SoapySampleSource();

        void Start() override;
        void Stop() override;

    private:
        SoapySampleSource(SampleFormat format, const std::string &device_name);

        void Run();
        
        SampleFormat format_;
        std::string device_name_;
        
        std::shared_ptr<SoapySDR::Device> device_;
        std::shared_ptr<SoapySDR::Stream> stream_;
        std::unique_ptr<std::thread> rx_thread_;
        bool halt_;

        static std::atomic_bool log_handler_registered_;
    };
};

#endif
