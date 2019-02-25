// -*- c++ -*-

// Copyright (c) 2019, FlightAware LLC.
// All rights reserved.
// Licensed under the 2-clause BSD license; see the LICENSE file

#ifndef FAUP1090_REPORTER_H
#define FAUP1090_REPORTER_H

#include <chrono>
#include <memory>

#include <boost/asio/io_service.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/asio/strand.hpp>

#include "track.h"
#include "uat_message.h"

namespace faup978 {
    struct ReportState {
        std::uint64_t slow_report_time = 0;
        std::uint64_t report_time = 0;
        uat::AircraftState report_state;
    };

    class Reporter : public std::enable_shared_from_this<Reporter> {
      public:
        typedef std::shared_ptr<Reporter> Pointer;

        static Pointer Create(boost::asio::io_service &service, std::chrono::milliseconds interval = std::chrono::milliseconds(500), std::chrono::milliseconds timeout = std::chrono::seconds(300)) { return Pointer(new Reporter(service, interval, timeout)); }

        void Start();
        void Stop();

        void HandleMessages(uat::SharedMessageVector messages) { tracker_->HandleMessages(messages); }

      private:
        Reporter(boost::asio::io_service &service, std::chrono::milliseconds interval, std::chrono::milliseconds timeout) : service_(service), strand_(service), report_timer_(service), purge_timer_(service), interval_(interval), timeout_(timeout) { tracker_ = uat::Tracker::Create(service, timeout); }

        void PeriodicReport();
        void PurgeOld();
        void ReportOneAircraft(const uat::Tracker::AddressKey &key, const uat::AircraftState &aircraft, std::uint64_t now);

        boost::asio::io_service &service_;
        boost::asio::io_service::strand strand_;
        boost::asio::steady_timer report_timer_;
        boost::asio::steady_timer purge_timer_;
        std::chrono::milliseconds interval_;
        std::chrono::milliseconds timeout_;
        uat::Tracker::Pointer tracker_;
        std::map<uat::Tracker::AddressKey, ReportState> reported_;
    };
} // namespace faup978

#endif
