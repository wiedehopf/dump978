// -*- c++ -*-

// Copyright (c) 2019, FlightAware LLC.
// All rights reserved.
// Licensed under the 2-clause BSD license; see the LICENSE file

#ifndef SOCKET_INPUT_H
#define SOCKET_INPUT_H

#include <memory>
#include <string>

#include <boost/asio/io_service.hpp>
#include <boost/asio/ip/tcp.hpp>

#include "message_source.h"

namespace uat {
    class RawInput : public MessageSource, public std::enable_shared_from_this<RawInput> {
      public:
        typedef std::shared_ptr<RawInput> Pointer;
        typedef std::function<void(const boost::system::error_code &)> ErrorHandler;

        static Pointer Create(boost::asio::io_service &service, const std::string &host, const std::string &port_or_service) { return Pointer(new RawInput(service, host, port_or_service)); }

        void Start();
        void Stop();

        void SetErrorHandler(ErrorHandler handler) { error_handler_ = handler; }

      private:
        RawInput(boost::asio::io_service &service, const std::string &host, const std::string &port_or_service);

        void TryNextEndpoint(const boost::system::error_code &last_error);
        void ScheduleRead();
        void ParseBuffer();
        boost::optional<uat::RawMessage> ParseLine(const std::string &line);
        void HandleError(const boost::system::error_code &ec);

        boost::asio::io_service &service_;
        std::string host_;
        std::string port_or_service_;

        boost::asio::ip::tcp::resolver resolver_;
        boost::asio::ip::tcp::socket socket_;
        boost::asio::ip::tcp::resolver::iterator next_endpoint_;

        ErrorHandler error_handler_;

        std::vector<char> readbuf_;
        std::size_t used_;
    };
}; // namespace uat

#endif
