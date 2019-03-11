// Copyright (c) 2019, FlightAware LLC.
// All rights reserved.
// Licensed under the 2-clause BSD license; see the LICENSE file

#include "soapy_source.h"

#include <iostream>

#include <SoapySDR/Errors.hpp>
#include <SoapySDR/Logger.hpp>

namespace dump978 {
    std::atomic_bool SoapySampleSource::log_handler_registered_(false);

    static void SoapyLogger(const SoapySDRLogLevel logLevel, const char *message) {
        static std::map<SoapySDRLogLevel, std::string> levels = {{SOAPY_SDR_FATAL, "FATAL"}, {SOAPY_SDR_CRITICAL, "CRITICAL"}, {SOAPY_SDR_ERROR, "ERROR"}, {SOAPY_SDR_WARNING, "WARNING"}, {SOAPY_SDR_NOTICE, "NOTICE"}, {SOAPY_SDR_INFO, "INFO"}, {SOAPY_SDR_DEBUG, "DEBUG"}, {SOAPY_SDR_TRACE, "TRACE"}, {SOAPY_SDR_SSI, "SSI"}};

        std::string level;
        auto i = levels.find(logLevel);
        if (i == levels.end())
            level = "UNKNOWN";
        else
            level = i->second;

        std::cerr << "SoapySDR: " << level << ": " << message << std::endl;
    }

    SoapySampleSource::SoapySampleSource(const std::string &device_name, const boost::program_options::variables_map &options) : timer_(service), device_name_(device_name), options_(options) {
        if (!log_handler_registered_.exchange(true)) {
            SoapySDR::registerLogHandler(SoapyLogger);
        }
    }

    SoapySampleSource::~SoapySampleSource() { Stop(); }

    void SoapySampleSource::Init() {
        device_ = {SoapySDR::Device::make(device_name_), &SoapySDR::Device::unmake};
        if (!device_) {
            throw std::runtime_error("no suitable device found");
        }

        // hacky mchackerson
        device_->setSampleRate(SOAPY_SDR_RX, 0, 2083333.0);
        device_->setFrequency(SOAPY_SDR_RX, 0, 978000000);
        device_->setBandwidth(SOAPY_SDR_RX, 0, 3.0e6);

        if (options_.count("sdr-auto-gain")) {
            if (!device_->hasGainMode(SOAPY_SDR_RX, 0)) {
                throw std::runtime_error("device does not support automatic gain mode");
            }
            device_->setGainMode(SOAPY_SDR_RX, 0, true);
        } else {
            auto gain = options_.count("sdr-gain") ? options_["sdr-gain"].as<double>() : 100.0;
            device_->setGainMode(SOAPY_SDR_RX, 0, false);
            device_->setGain(SOAPY_SDR_RX, 0, gain);
        }

        if (options_.count("sdr-ppm")) {
            if (device_->hasFrequencyCorrection(SOAPY_SDR_RX, 0)) {
                device_->setFrequencyCorrection(SOAPY_SDR_RX, 0, options_["sdr-ppm"].as<double>());
            } else {
                throw std::runtime_error("device does not support frequency correction");
            }
        }

        if (options_.count("sdr-antenna")) {
            device_->setAntenna(SOAPY_SDR_RX, 0, options_["sdr-antenna"].as<std::string>());
        }

        if (options_.count("sdr-device-settings")) {
            for (auto kv : SoapySDR::KwargsFromString(options_["sdr-stream-settings"].as<std::string>())) {
                device_->writeSetting(kv.first, kv.second);
            }
        }

        if (options_.count("format")) {
            format_ = options_["format"].as<SampleFormat>();
        }

        std::string soapy_format;
        if (format_ == SampleFormat::UNKNOWN) {
            double fullScale;
            soapy_format = device_->getNativeStreamFormat(SOAPY_SDR_RX, 0, fullScale);
            // clang-format off
            static std::map<std::string,SampleFormat> format_map = {
                { "CU8", SampleFormat::CU8 },
                { "CS8", SampleFormat::CS8 },
                { "CS16", SampleFormat::CS16H },
                { "CF32", SampleFormat::CF32H }
            };
            // clang-format on
            auto i = format_map.find(soapy_format);
            if (i != format_map.end()) {
                format_ = i->second;
            } else {
                throw std::runtime_error("Unsupported native SDR format: " + soapy_format + "; try specifying --format");
            }
        } else {
            switch (format_) {
            case SampleFormat::CU8:
                soapy_format = "CU8";
                break;
            case SampleFormat::CS8:
                soapy_format = "CS8";
                break;
            case SampleFormat::CS16H:
                soapy_format = "CS16";
                break;
            case SampleFormat::CF32H:
                soapy_format = "CF32";
                break;
            default:
                throw std::runtime_error("unsupported sample format");
            }
        }

        std::vector<size_t> channels = {0};

        SoapySDR::Kwargs stream_settings;
        if (device_->getDriverKey() == "RTLSDR") {
            // some soapysdr builds have a very low default here
            stream_settings["buffsize"] = "262144";
        }

        if (options_.count("sdr-stream-settings")) {
            for (auto kv : SoapySDR::KwargsFromString(options_["sdr-stream-settings"].as<std::string>())) {
                stream_settings[kv.first] = kv.second;
            }
        }

        stream_ = {device_->setupStream(SOAPY_SDR_RX, soapy_format, channels, stream_settings), std::bind(&SoapySDR::Device::closeStream, device_, std::placeholders::_1)};
        if (!stream_) {
            throw std::runtime_error("failed to construct stream");
        }
    }

    void SoapySampleSource::Start() {
        if (!device_ || !stream_) {
            Init();
        }

        device_->activateStream(stream_.get());

        halt_ = false;
        rx_thread_.reset(new std::thread(&SoapySampleSource::Run, this));
    }

    void SoapySampleSource::Stop() {
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

    void SoapySampleSource::Run() {
        const auto bytes_per_element = BytesPerSample(format_);
        const auto elements = std::max<size_t>(65536, device_->getStreamMTU(stream_.get()));

        uat::Bytes block;
        block.reserve(elements * bytes_per_element);

        while (!halt_) {
            void *buffs[1] = {block.data()};
            int flags = 0;
            long long time_ns;

            block.resize(elements * bytes_per_element);
            auto elements_read = device_->readStream(stream_.get(), buffs, elements, flags, time_ns,
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
            std::uint64_t timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(start_of_block - unix_epoch).count();

            DispatchBuffer(timestamp, block);
        }
    }
}; // namespace dump978
