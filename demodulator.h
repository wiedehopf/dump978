// -*- c++ -*-

// Copyright (c) 2019, FlightAware LLC.
// All rights reserved.
// Licensed under the 2-clause BSD license; see the LICENSE file

#ifndef DUMP978_DEMODULATOR_H
#define DUMP978_DEMODULATOR_H

#include <functional>
#include <vector>

#include "common.h"
#include "convert.h"
#include "fec.h"
#include "message_source.h"
#include "uat_message.h"

namespace dump978 {
    class Demodulator {
      public:
        // Return value of Demodulate
        struct Message {
            uat::Bytes payload;
            unsigned corrected_errors;
            uat::PhaseBuffer::const_iterator begin;
            uat::PhaseBuffer::const_iterator end;
        };

        virtual ~Demodulator() {}
        virtual std::vector<Message> Demodulate(uat::PhaseBuffer::const_iterator begin, uat::PhaseBuffer::const_iterator end) = 0;

        virtual unsigned NumTrailingSamples() = 0;

      protected:
        uat::FEC fec_;
    };

    class TwoMegDemodulator : public Demodulator {
      public:
        std::vector<Message> Demodulate(uat::PhaseBuffer::const_iterator begin, uat::PhaseBuffer::const_iterator end) override;
        unsigned NumTrailingSamples() override;

      private:
        boost::optional<Message> DemodBest(uat::PhaseBuffer::const_iterator begin, bool downlink);
        boost::optional<Message> DemodOneDownlink(uat::PhaseBuffer::const_iterator begin);
        boost::optional<Message> DemodOneUplink(uat::PhaseBuffer::const_iterator begin);
    };

    class Receiver : public uat::MessageSource {
      public:
        virtual void HandleSamples(std::uint64_t timestamp, uat::Bytes::const_iterator begin, uat::Bytes::const_iterator end) = 0;
    };

    class SingleThreadReceiver : public Receiver {
      public:
        SingleThreadReceiver(SampleFormat format);

        void HandleSamples(std::uint64_t timestamp, uat::Bytes::const_iterator begin, uat::Bytes::const_iterator end) override;

      private:
        SampleConverter::Pointer converter_;
        std::unique_ptr<Demodulator> demodulator_;

        uat::Bytes samples_;
        std::size_t saved_samples_ = 0;

        uat::PhaseBuffer phase_;
    };

}; // namespace dump978

#endif
