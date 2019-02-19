// Copyright (c) 2019, FlightAware LLC.
// All rights reserved.
// Licensed under the 2-clause BSD license; see the LICENSE file

#include "convert.h"

#include <cmath>
#include <assert.h>

// TODO: This probably needs PhaseBuffer to use a POD-vector-like thing (not
// std::vector) to run at an acceptable speed. Profile it and check.

namespace dump978 {
    static inline std::uint16_t scaled_atan2(double y, double x) {
        double ang = std::atan2(y, x) + M_PI; // atan2 returns [-pi..pi], normalize to [0..2*pi]
        double scaled_ang = std::round(32768 * ang / M_PI);
        return scaled_ang < 0 ? 0 : scaled_ang > 65535 ? 65535 : (std::uint16_t)scaled_ang;
    }
    
    SampleConverter::Pointer SampleConverter::Create(SampleFormat format) {
        switch (format) {
        case SampleFormat::CU8:
            return Pointer(new CU8Converter());
        case SampleFormat::CS8:
            return Pointer(new CS8Converter());
        case SampleFormat::CS16H:
            return Pointer(new CS16HConverter());
        case SampleFormat::CF32H:
            return Pointer(new CF32HConverter());
        default:
            throw std::runtime_error("format not implemented yet");
        }
    }

    CU8Converter::CU8Converter() {
        cu8_alias u;
        
        unsigned i,q;
        for (i = 0; i < 256; ++i) {
            double d_i = (i - 127.5);
            for (q = 0; q < 256; ++q) {
                double d_q = (q - 127.5);
                u.iq[0] = i;
                u.iq[1] = q;
                lookup_[u.iq16] = scaled_atan2(d_q, d_i);
            }
        }
    }

    void CU8Converter::Convert(const uat::Bytes &in, uat::PhaseBuffer &out) {
        const cu8_alias *in_iq = reinterpret_cast<const cu8_alias*>(in.data());
        
        // unroll the loop
        const auto n = in.size() / 2;
        const auto n8 = n / 8;
        const auto n7 = n & 7;

        out.reserve(out.size() + n);
        for (unsigned i = 0; i < n8; ++i, in_iq += 8) {
            out.push_back(lookup_[in_iq[0].iq16]);
            out.push_back(lookup_[in_iq[1].iq16]);
            out.push_back(lookup_[in_iq[2].iq16]);
            out.push_back(lookup_[in_iq[3].iq16]);
            out.push_back(lookup_[in_iq[4].iq16]);
            out.push_back(lookup_[in_iq[5].iq16]);
            out.push_back(lookup_[in_iq[6].iq16]);
            out.push_back(lookup_[in_iq[7].iq16]);
        }
        for (unsigned i = 0; i < n7; ++i, ++in_iq) {
            out.push_back(lookup_[in_iq[0].iq16]);
        }
    }

    CS8Converter::CS8Converter() {
        cs8_alias u;
        
        int i,q;
        for (i = -128; i <= 127; ++i) {
            for (q = -128; q <= 127; ++q) {
                u.iq[0] = i;
                u.iq[1] = q;
                lookup_[u.iq16] = scaled_atan2(q, i);
            }
        }
    }

    void CS8Converter::Convert(const uat::Bytes &in, uat::PhaseBuffer &out) {
        const cs8_alias *in_iq = reinterpret_cast<const cs8_alias*>(in.data());

        // unroll the loop
        const auto n = in.size() / 2;
        const auto n8 = n / 8;
        const auto n7 = n & 7;

        out.reserve(out.size() + n);        
        for (unsigned i = 0; i < n8; ++i, in_iq += 8) {
            out.push_back(lookup_[in_iq[0].iq16]);
            out.push_back(lookup_[in_iq[1].iq16]);
            out.push_back(lookup_[in_iq[2].iq16]);
            out.push_back(lookup_[in_iq[3].iq16]);
            out.push_back(lookup_[in_iq[4].iq16]);
            out.push_back(lookup_[in_iq[5].iq16]);
            out.push_back(lookup_[in_iq[6].iq16]);
            out.push_back(lookup_[in_iq[7].iq16]);
        }
        for (unsigned i = 0; i < n7; ++i, ++in_iq) {
            out.push_back(lookup_[in_iq[0].iq16]);
        }
    }

    void CS16HConverter::Convert(const uat::Bytes &in, uat::PhaseBuffer &out) {
        const std::int16_t *in_iq = reinterpret_cast<const std::int16_t*>(in.data());

        // unroll the loop
        const auto n = in.size() / 4;
        const auto n8 = n / 8;
        const auto n7 = n & 7;

        out.reserve(out.size() + n);        
        for (unsigned i = 0; i < n8; ++i, in_iq += 8) {
            out.push_back(scaled_atan2(in_iq[1], in_iq[0]));
            out.push_back(scaled_atan2(in_iq[3], in_iq[2]));
            out.push_back(scaled_atan2(in_iq[5], in_iq[4]));
            out.push_back(scaled_atan2(in_iq[7], in_iq[6]));
            out.push_back(scaled_atan2(in_iq[9], in_iq[8]));
            out.push_back(scaled_atan2(in_iq[11], in_iq[10]));
            out.push_back(scaled_atan2(in_iq[13], in_iq[12]));
            out.push_back(scaled_atan2(in_iq[15], in_iq[14]));
        }
        for (unsigned i = 0; i < n7; ++i, ++in_iq) {
            out.push_back(scaled_atan2(in_iq[1], in_iq[0]));
        }
    }

    void CF32HConverter::Convert(const uat::Bytes &in, uat::PhaseBuffer &out) {
        const double *in_iq = reinterpret_cast<const double*>(in.data());

        // unroll the loop
        const auto n = in.size() / 8;
        const auto n8 = n / 8;
        const auto n7 = n & 7;

        out.reserve(out.size() + n);        
        for (unsigned i = 0; i < n8; ++i, in_iq += 8) {
            out.push_back(scaled_atan2(in_iq[1], in_iq[0]));
            out.push_back(scaled_atan2(in_iq[3], in_iq[2]));
            out.push_back(scaled_atan2(in_iq[5], in_iq[4]));
            out.push_back(scaled_atan2(in_iq[7], in_iq[6]));
            out.push_back(scaled_atan2(in_iq[9], in_iq[8]));
            out.push_back(scaled_atan2(in_iq[11], in_iq[10]));
            out.push_back(scaled_atan2(in_iq[13], in_iq[12]));
            out.push_back(scaled_atan2(in_iq[15], in_iq[14]));
        }
        for (unsigned i = 0; i < n7; ++i, ++in_iq) {
            out.push_back(scaled_atan2(in_iq[15], in_iq[14]));
        }
    }
}
