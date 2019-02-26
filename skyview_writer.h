// -*- c++ -*-

// Copyright (c) 2019, FlightAware LLC.
// All rights reserved.
// Licensed under the 2-clause BSD license; see the LICENSE file

#ifndef SKYVIEW_WRITER_H
#define SKYVIEW_WRITER_H

#include <chrono>
#include <memory>

#include <boost/asio/io_service.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/asio/strand.hpp>
#include <boost/filesystem.hpp>

#include "track.h"
#include "uat_message.h"

namespace uat {
    namespace skyview {
        class SkyviewWriter : public std::enable_shared_from_this<SkyviewWriter> {
          public:
            typedef std::shared_ptr<SkyviewWriter> Pointer;

            static Pointer Create(boost::asio::io_service &service, Tracker::Pointer tracker, const boost::filesystem::path &dir, std::chrono::milliseconds interval) { return Pointer(new SkyviewWriter(service, tracker, dir, interval)); }

            void Start();
            void Stop();

          private:
            SkyviewWriter(boost::asio::io_service &service, Tracker::Pointer tracker, const boost::filesystem::path &dir, std::chrono::milliseconds interval) : service_(service), strand_(service), timer_(service), tracker_(tracker), dir_(dir), interval_(interval) {}

            void PeriodicWrite();

            boost::asio::io_service &service_;
            boost::asio::io_service::strand strand_;
            boost::asio::steady_timer timer_;
            uat::Tracker::Pointer tracker_;
            boost::filesystem::path dir_;
            std::chrono::milliseconds interval_;
        };
    } // namespace skyview
} // namespace uat

#endif
