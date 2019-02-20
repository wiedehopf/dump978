// Copyright (c) 2019, FlightAware LLC.
// All rights reserved.
// Licensed under the 2-clause BSD license; see the LICENSE file

#include "soapy_source.h"

#include <iostream>

#include <SoapySDR/Errors.hpp>
#include <SoapySDR/Logger.hpp>

namespace dump978 {
    std::atomic_bool SoapySampleSource::log_handler_registered_(false);

    static void SoapyLogger(const SoapySDRLogLevel logLevel, const char *message)
    {
        static std::map<SoapySDRLogLevel,std::string> levels = {
            { SOAPY_SDR_FATAL,     "FATAL" },
            { SOAPY_SDR_CRITICAL,  "CRITICAL" },
            { SOAPY_SDR_ERROR,     "ERROR" },
            { SOAPY_SDR_WARNING,   "WARNING" },
            { SOAPY_SDR_NOTICE,    "NOTICE" },
            { SOAPY_SDR_INFO,      "INFO" },
            { SOAPY_SDR_DEBUG,     "DEBUG" },
            { SOAPY_SDR_TRACE,     "TRACE" },
            { SOAPY_SDR_SSI,       "SSI" }
        };

        std::string level;
        auto i = levels.find(logLevel);
        if (i == levels.end())
            level = "UNKNOWN";
        else
            level = i->second;

        std::cerr << "SoapySDR: " << level << ": " << message << std::endl;
    }

    SoapySampleSource::SoapySampleSource(SampleFormat format, const std::string &device_name)
        : format_(format), device_name_(device_name), halt_(false)
    {
        if (!log_handler_registered_.exchange(true)) {
            SoapySDR::registerLogHandler(SoapyLogger);
        }
    }

    SoapySampleSource::~SoapySampleSource()
    {
        Stop();
    }

    void SoapySampleSource::Start()
    {
        device_ = { SoapySDR::Device::make(device_name_), &SoapySDR::Device::unmake };
        if (!device_) {
            throw std::runtime_error("no suitable device found");
        }

        // hacky mchackerson
        device_->setSampleRate(SOAPY_SDR_RX, 0, 2083333.0);
        device_->setFrequency(SOAPY_SDR_RX, 0, 978000000);
        device_->setGain(SOAPY_SDR_RX, 0, 50.0);
        device_->setBandwidth(SOAPY_SDR_RX, 0, 3.0e6);

        std::string soapy_format;
        switch (format_) {
        case SampleFormat::CU8:   soapy_format = "CU8"; break;
        case SampleFormat::CS8:   soapy_format = "CS8"; break;
        case SampleFormat::CS16H: soapy_format = "CS16"; break;
        case SampleFormat::CF32H: soapy_format = "CF32"; break;
        default:
            throw std::runtime_error("unsupported sample format");
        }

        std::vector<size_t> channels = { 0 };

        stream_ = { device_->setupStream(SOAPY_SDR_RX, soapy_format, channels),
                    std::bind(&SoapySDR::Device::closeStream, device_, std::placeholders::_1) };
        if (!stream_) {
            throw std::runtime_error("failed to construct stream");
        }

        device_->activateStream(stream_.get());

        halt_ = false;
        rx_thread_.reset(new std::thread(&SoapySampleSource::Run, this));
    }

    void SoapySampleSource::Stop()
    {
        if (stream_) {
            // rtlsdr needs the rx thread to drain data before this returns..
            device_->deactivateStream(stream_.get());
        }
        if (rx_thread_) {
            halt_ = true;
            rx_thread_->join();
            rx_thread_.reset();
        }
        if (stream_) {
            stream_.reset();
        }
        if (device_) {
            device_.reset();
        }
    }

    void SoapySampleSource::Run()
    {
        const auto bytes_per_element = BytesPerSample(format_);
        const auto elements = std::max<size_t>(65536, device_->getStreamMTU(stream_.get()));

        uat::Bytes block;
        block.reserve(elements * bytes_per_element);

        while (!halt_) {
            void *buffs[1] = { block.data() };
            int flags = 0;
            long long time_ns;

            block.resize(elements * bytes_per_element);
            auto elements_read = device_->readStream(stream_.get(),
                                                     buffs,
                                                     elements,
                                                     flags,
                                                     time_ns,
                                                     /* timeout, microseconds */ 1000000);
            if (halt_) {
                break;
            }

            if (elements_read < 0) {
                std::cerr << "SoapySDR reports error: " << SoapySDR::errToStr(elements_read) << std::endl;
                auto ec = boost::system::error_code(0, boost::system::generic_category());
                DispatchError(ec);
                break;
            }

            block.resize(elements_read * bytes_per_element);

            // work out a starting timestamp
            static auto unix_epoch = std::chrono::system_clock::from_time_t(0);
            auto end_of_block = std::chrono::system_clock::now();
            auto start_of_block = end_of_block - (std::chrono::milliseconds(1000) * elements / 2083333);
            std::uint64_t timestamp =
                std::chrono::duration_cast<std::chrono::milliseconds>(start_of_block - unix_epoch).count();

            DispatchBuffer(timestamp, block);
        }
    }
};
