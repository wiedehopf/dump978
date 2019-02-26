// Copyright (c) 2019, FlightAware LLC.
// All rights reserved.
// Licensed under the 2-clause BSD license; see the LICENSE file

#include <boost/exception/diagnostic_information.hpp>
#include <boost/program_options.hpp>
#include <boost/regex.hpp>

#include <iostream>
#include <memory>

#include "faup978_reporter.h"
#include "message_source.h"
#include "socket_input.h"
#include "uat_message.h"

using namespace uat;
using namespace faup978;

namespace po = boost::program_options;
using boost::asio::ip::tcp;

struct connect_option {
    std::string host;
    std::string port;
};

// Specializations of validate for --connect
void validate(boost::any &v, const std::vector<std::string> &values, connect_option *target_type, int) {
    po::validators::check_first_occurrence(v);
    const std::string &s = po::validators::get_single_string(values);

    static const boost::regex r("(?:([^:]+):)?(\\d+)");
    boost::smatch match;
    if (boost::regex_match(s, match, r)) {
        v = boost::any(connect_option{match[1], match[2]});
    } else {
        throw po::validation_error(po::validation_error::invalid_option_value);
    }
}

#define EXIT_NO_RESTART (64)

static int realmain(int argc, char **argv) {
    boost::asio::io_service io_service;

    po::options_description desc("Allowed options");
    desc.add_options()("help", "produce help message")("connect", po::value<connect_option>(), "connect to host:port for raw UAT data");

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
        std::cerr << desc << std::endl;
        return EXIT_NO_RESTART;
    }

    if (!opts.count("connect")) {
        std::cerr << "--connect option is required" << std::endl;
        return EXIT_NO_RESTART;
    }

    auto connect = opts["connect"].as<connect_option>();
    auto input = RawInput::Create(io_service, connect.host, connect.port);
    auto reporter = Reporter::Create(io_service);

    input->SetConsumer(std::bind(&Reporter::HandleMessages, reporter, std::placeholders::_1));
    input->SetErrorHandler([&io_service](const boost::system::error_code &ec) {
        std::cerr << "Connection failed: " << ec.message() << std::endl;
        io_service.stop();
    });

    reporter->Start();
    input->Start();

    io_service.run();

    input->Stop();
    reporter->Stop();
    return 0;
}

int main(int argc, char **argv) {
#if 0
    try {
        return realmain(argc, argv);
    } catch (...) {
        std::cerr << "Uncaught exception: " << boost::current_exception_diagnostic_information() << std::endl;
        return 2;
    }
#else
    return realmain(argc, argv);
#endif
}
