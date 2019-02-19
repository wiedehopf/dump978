// -*- c++ -*-

// Copyright (c) 2019, FlightAware LLC.
// All rights reserved.
// Licensed under the 2-clause BSD license; see the LICENSE file

#ifndef DUMP978_CONVERT_H
#define DUMP978_CONVERT_H

#include <memory>

#include "common.h"

namespace dump978 {
    // Describes a sample data layout:
    //   CU8  - interleaved I/Q data, 8 bit unsigned integers
    //   CS8  - interleaved I/Q data, 8 bit signed integers
    //   CS16H - interleaved I/Q data, 16 bit signed integers, host byte order
    //   CF32H - interleaved I/Q data, 32 bit signed floats, host byte order
    enum class SampleFormat { CU8, CS8, CS16H, CF32H, UNKNOWN };

    // Return the number of bytes for 1 sample in the given format
    inline static unsigned BytesPerSample(SampleFormat f) {
        switch (f) {
        case SampleFormat::CU8: return 2;
        case SampleFormat::CS8: return 2;
        case SampleFormat::CS16H: return 4;
        case SampleFormat::CF32H: return 8;
        default: return 0;
        }
    }

    // Base class for all sample converters.
    // Use SampleConverter::Create to build converters.
    class SampleConverter {
    public:
        typedef std::shared_ptr<SampleConverter> Pointer;

        virtual ~SampleConverter() {};

        // Read samples from `in` and append one phase value per sample to `out`.
        // The input buffer should contain an integral number of samples (trailing
        // partial samples are ignored, not buffered)
        virtual void Convert(const uat::Bytes &in, uat::PhaseBuffer &out) = 0;

        // Return a new SampleConverter that converts from the given format
        static Pointer Create(SampleFormat format);
    };

    class CU8Converter : public SampleConverter {
    public:
        CU8Converter();

        void Convert(const uat::Bytes &in, uat::PhaseBuffer &out) override;

    private:
        union cu8_alias {
            std::uint8_t iq[2];
            std::uint16_t iq16;
        };

        std::array<std::uint16_t,65536> lookup_;
    };

    class CS8Converter : public SampleConverter {
    public:
        CS8Converter();

        void Convert(const uat::Bytes &in, uat::PhaseBuffer &out) override;

    private:
        union cs8_alias {
            std::int8_t iq[2];
            std::uint16_t iq16;
        };

        std::array<std::uint16_t,65536> lookup_;
    };

    class CS16HConverter : public SampleConverter {
    public:
        void Convert(const uat::Bytes &in, uat::PhaseBuffer &out) override;
    };

    class CF32HConverter : public SampleConverter {
    public:
        void Convert(const uat::Bytes &in, uat::PhaseBuffer &out) override;
    };
};

#endif
