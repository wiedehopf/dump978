// Copyright (c) 2019, FlightAware LLC.
// All rights reserved.
// Licensed under the 2-clause BSD license; see the LICENSE file

#include <boost/asio/ip/address_v4.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/signal_set.hpp>
#include <boost/exception/diagnostic_information.hpp>
#include <boost/program_options.hpp>
#include <boost/regex.hpp>

#include <iostream>
#include <memory>

#include "convert.h"
#include "demodulator.h"
#include "message_dispatch.h"
#include "sample_source.h"
#include "soapy_source.h"
#include "socket_output.h"

using namespace flightaware::uat;

namespace po = boost::program_options;
using boost::asio::ip::tcp;

struct listen_option {
    std::string host;
    std::string port;
};

SampleFormat format;

// Specializations of validate for --xxx-port
void validate(boost::any &v, const std::vector<std::string> &values, listen_option *target_type, int) {
    po::validators::check_first_occurrence(v);
    const std::string &s = po::validators::get_single_string(values);

    static const boost::regex r("(?:([^:]+):)?(\\d+)");
    boost::smatch match;
    if (boost::regex_match(s, match, r)) {
        listen_option o;
        o.host = match[1];
        o.port = match[2];
        v = boost::any(o);
    } else {
        throw po::validation_error(po::validation_error::invalid_option_value);
    }
}

// Specializations of validate for --format
namespace flightaware::uat {
    void validate(boost::any &v, const std::vector<std::string> &values, SampleFormat *target_type, int) {
        po::validators::check_first_occurrence(v);
        const std::string &s = po::validators::get_single_string(values);

        // clang-format off
        static std::map<std::string, SampleFormat> formats = {
            {"CU8", SampleFormat::CU8},
            {"CS8", SampleFormat::CS8},
            {"CS16H", SampleFormat::CS16H},
            {"CF32H", SampleFormat::CF32H}
        };
        // clang-format on

        auto entry = formats.find(s);
        if (entry == formats.end())
            throw po::validation_error(po::validation_error::invalid_option_value);

        v = boost::any(entry->second);
    }
} // namespace flightaware::uat

#define EXIT_NO_RESTART (64)

static int realmain(int argc, char **argv) {
    boost::asio::io_service io_service;

    // clang-format off
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("version", "show version")
        ("raw-stdout", "write raw messages to stdout")
        ("json-stdout", "write decoded json to stdout")
        ("format", po::value<SampleFormat>(), "set sample format")
        ("stdin", "read sample data from stdin")
        ("file", po::value<std::string>(), "read sample data from a file")
        ("file-throttle", "throttle file input to realtime")
        ("sdr", po::value<std::string>(), "read sample data from named SDR device")
        ("sdr-auto-gain", "enable SDR AGC")
        ("sdr-gain", po::value<double>(), "set SDR gain in dB")
        ("sdr-ppm", po::value<double>(), "set SDR frequency correction in PPM")
        ("sdr-antenna", po::value<std::string>(), "set SDR antenna name")
        ("sdr-stream-settings", po::value<std::string>(), "set SDR stream key-value settings")
        ("sdr-device-settings", po::value<std::string>(), "set SDR device key-value settings")
        ("raw-port", po::value<std::vector<listen_option>>(), "listen for connections on [host:]port and provide raw messages")
        ("json-port", po::value<std::vector<listen_option>>(), "listen for connections on [host:]port and provide decoded json");
    // clang-format on

    po::variables_map opts;

    try {
        po::store(po::parse_command_line(argc, argv, desc), opts);
        po::notify(opts);
    } catch (boost::program_options::error &err) {
        std::cerr << err.what() << std::endl;
        std::cerr << desc << std::endl;
        return EXIT_NO_RESTART;
    }

    if (opts.count("help")) {
        std::cerr << "dump978-fa " << VERSION << std::endl;
        std::cerr << desc << std::endl;
        return EXIT_NO_RESTART;
    }

    if (opts.count("version")) {
        std::cerr << "dump978-fa " << VERSION << std::endl;
        return EXIT_NO_RESTART;
    }

    MessageDispatch dispatch;
    SampleSource::Pointer source;

    tcp::resolver resolver(io_service);

    if (opts.count("stdin") + opts.count("file") + opts.count("sdr") != 1) {
        std::cerr << "Exactly one of --stdin, --file, or --sdr must be used" << std::endl;
        return EXIT_NO_RESTART;
    }

    if (opts.count("stdin")) {
        source = StdinSampleSource::Create(io_service, opts);
    } else if (opts.count("file")) {
        boost::filesystem::path path(opts["file"].as<std::string>());
        source = FileSampleSource::Create(io_service, path, opts);
    } else if (opts.count("sdr")) {
        auto device = opts["sdr"].as<std::string>();
        source = SoapySampleSource::Create(io_service, device, opts);
    } else {
        assert("impossible case" && false);
    }

    auto create_output_port = [&](std::string option, SocketListener::ConnectionFactory factory) -> bool {
        if (!opts.count(option)) {
            return true;
        }

        bool ok = true;
        for (auto l : opts[option].as<std::vector<listen_option>>()) {
            tcp::resolver::query query(l.host, l.port, tcp::resolver::query::passive);
            boost::system::error_code ec;

            bool success = false;
            tcp::resolver::iterator end;
            for (auto i = resolver.resolve(query, ec); i != end; ++i) {
                const auto &endpoint = i->endpoint();

                try {
                    auto listener = SocketListener::Create(io_service, endpoint, dispatch, factory);
                    listener->Start();
                    std::cerr << option << ": listening for connections on " << endpoint << std::endl;
                    success = true;
                } catch (boost::system::system_error &err) {
                    std::cerr << option << ": could not listen on " << endpoint << ": " << err.what() << std::endl;
                    ec = err.code();
                }
            }

            if (!success) {
                std::cerr << option << ": no available listening addresses" << std::endl;
                ok = false;
            }
        }

        return ok;
    };

    auto raw_ok = create_output_port("raw-port", &RawOutput::Create);
    auto json_ok = create_output_port("json-port", &JsonOutput::Create);
    if (!raw_ok || !json_ok) {
        return 1;
    }

    if (opts.count("raw-stdout")) {
        dispatch.AddClient([](SharedMessageVector messages) {
            for (const auto &message : *messages) {
                std::cout << message << std::endl;
            }
        });
    }

    if (opts.count("json-stdout")) {
        dispatch.AddClient([](SharedMessageVector messages) {
            for (const auto &message : *messages) {
                if (message.Type() == MessageType::DOWNLINK_SHORT || message.Type() == MessageType::DOWNLINK_LONG) {
                    std::cout << AdsbMessage(message).ToJson() << std::endl;
                }
            }
        });
    }

    source->Init();
    auto format = source->Format();

    auto receiver = std::make_shared<SingleThreadReceiver>(format);
    receiver->SetConsumer(std::bind(&MessageDispatch::Dispatch, &dispatch, std::placeholders::_1));

    bool saw_error = false;

    source->SetConsumer([&io_service, &saw_error, receiver](std::uint64_t timestamp, const Bytes &buffer, const boost::system::error_code &ec) {
        if (ec) {
            if (ec == boost::asio::error::eof) {
                std::cerr << "Sample source reports EOF" << std::endl;
            } else {
                std::cerr << "Sample source reports error: " << ec.message() << std::endl;
                saw_error = true;
            }
            io_service.stop();
        } else {
            receiver->HandleSamples(timestamp, buffer.begin(), buffer.end());
        }
    });

    boost::asio::signal_set signals(io_service, SIGINT, SIGTERM);
    signals.async_wait([&io_service, &saw_error](const boost::system::error_code &ec, int signum) {
        std::cerr << "Caught signal " << signum << ", exiting" << std::endl;
        saw_error = true;
        io_service.stop();
    });

    source->Start();

    io_service.run();

    source->Stop();

    if (saw_error) {
        std::cerr << "Abnormal exit" << std::endl;
        return 1;
    } else {
        std::cerr << "Ran out of things to do, exiting" << std::endl;
        return 0;
    }
}

int main(int argc, char **argv) {
    try {
        return realmain(argc, argv);
    } catch (...) {
        std::cerr << "Uncaught exception: " << boost::current_exception_diagnostic_information() << std::endl;
        return 2;
    }
}
