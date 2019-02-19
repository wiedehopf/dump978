// -*- c++ -*-

// Copyright (c) 2019, FlightAware LLC.
// All rights reserved.
// Licensed under the 2-clause BSD license; see the LICENSE file

#ifndef DUMP978_DEMODULATOR_H
#define DUMP978_DEMODULATOR_H

#include <vector>
#include <functional>

#include "fec.h"
#include "common.h"
#include "uat_message.h"
#include "convert.h"
#include "message_source.h"

namespace dump978 {
    class Demodulator {
    public:
        virtual ~Demodulator() {}
        virtual uat::SharedMessageVector Demodulate(std::uint64_t timestamp, const uat::PhaseBuffer &buffer) = 0;

        virtual unsigned NumTrailingSamples() = 0;

    protected:
        uat::FEC fec_;
    };

    class TwoMegDemodulator : public Demodulator {
    public:
        uat::SharedMessageVector Demodulate(std::uint64_t timestamp, const uat::PhaseBuffer &buffer) override;
        unsigned NumTrailingSamples() override;

    private:
        uat::RawMessage DemodBest(const uat::PhaseBuffer &buffer, unsigned start, bool downlink, std::uint64_t timestamp);
        uat::RawMessage DemodOneDownlink(const uat::PhaseBuffer &buffer, unsigned start, std::uint64_t timestamp);
        uat::RawMessage DemodOneUplink(const uat::PhaseBuffer &buffer, unsigned start, std::uint64_t timestamp);
    };

    class Receiver : public uat::MessageSource {
    public:
        virtual void HandleSamples(std::uint64_t timestamp, const uat::Bytes &buffer) = 0;
    };
    
    class SingleThreadReceiver : public Receiver {
    public:
        SingleThreadReceiver(SampleFormat format);

        void HandleSamples(std::uint64_t timestamp, const uat::Bytes &buffer) override;

    private:
        SampleConverter::Pointer converter_;
        uat::PhaseBuffer phase_;
        std::unique_ptr<Demodulator> demodulator_;
    };

};
       
#endif

