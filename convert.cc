// Copyright (c) 2019, FlightAware LLC.
// All rights reserved.
// Licensed under the 2-clause BSD license; see the LICENSE file

#include "convert.h"

#include <assert.h>
#include <cmath>

namespace dump978 {
    static inline std::uint16_t scaled_atan2(double y, double x) {
        double ang = std::atan2(y, x) + M_PI; // atan2 returns [-pi..pi], normalize to [0..2*pi]
        double scaled_ang = std::round(32768 * ang / M_PI);
        return scaled_ang < 0 ? 0 : scaled_ang > 65535 ? 65535 : (std::uint16_t)scaled_ang;
    }

    static inline double magsq(double i, double q) { return i * i + q * q; }

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

    CU8Converter::CU8Converter() : SampleConverter(SampleFormat::CU8) {

        cu8_alias u;

        unsigned i, q;
        for (i = 0; i < 256; ++i) {
            double d_i = (i - 127.5) / 128.0;
            for (q = 0; q < 256; ++q) {
                double d_q = (q - 127.5) / 128.0;
                u.iq[0] = i;
                u.iq[1] = q;
                lookup_phase_[u.iq16] = scaled_atan2(d_q, d_i);
                lookup_magsq_[u.iq16] = magsq(d_i, d_q);
            }
        }
    }

    void CU8Converter::ConvertPhase(uat::Bytes::const_iterator begin, uat::Bytes::const_iterator end, uat::PhaseBuffer::iterator out) {
        const cu8_alias *in_iq = reinterpret_cast<const cu8_alias *>(&*begin);

        // unroll the loop
        const auto n = std::distance(begin, end) / 2;
        const auto n8 = n / 8;
        const auto n7 = n & 7;

        for (auto i = 0; i < n8; ++i, in_iq += 8) {
            *out++ = lookup_phase_[in_iq[0].iq16];
            *out++ = lookup_phase_[in_iq[1].iq16];
            *out++ = lookup_phase_[in_iq[2].iq16];
            *out++ = lookup_phase_[in_iq[3].iq16];
            *out++ = lookup_phase_[in_iq[4].iq16];
            *out++ = lookup_phase_[in_iq[5].iq16];
            *out++ = lookup_phase_[in_iq[6].iq16];
            *out++ = lookup_phase_[in_iq[7].iq16];
        }
        for (auto i = 0; i < n7; ++i, ++in_iq) {
            *out++ = lookup_phase_[in_iq[0].iq16];
        }
    }

    void CU8Converter::ConvertMagSq(uat::Bytes::const_iterator begin, uat::Bytes::const_iterator end, std::vector<double>::iterator out) {
        const cu8_alias *in_iq = reinterpret_cast<const cu8_alias *>(&*begin);

        // unroll the loop
        const auto n = std::distance(begin, end) / 2;
        const auto n8 = n / 8;
        const auto n7 = n & 7;

        for (auto i = 0; i < n8; ++i, in_iq += 8) {
            *out++ = lookup_magsq_[in_iq[0].iq16];
            *out++ = lookup_magsq_[in_iq[1].iq16];
            *out++ = lookup_magsq_[in_iq[2].iq16];
            *out++ = lookup_magsq_[in_iq[3].iq16];
            *out++ = lookup_magsq_[in_iq[4].iq16];
            *out++ = lookup_magsq_[in_iq[5].iq16];
            *out++ = lookup_magsq_[in_iq[6].iq16];
            *out++ = lookup_magsq_[in_iq[7].iq16];
        }
        for (auto i = 0; i < n7; ++i, ++in_iq) {
            *out++ = lookup_magsq_[in_iq[0].iq16];
        }
    }

    CS8Converter::CS8Converter() : SampleConverter(SampleFormat::CS8) {
        cs8_alias u;

        int i, q;
        for (i = -128; i <= 127; ++i) {
            double d_i = i / 128.0;
            for (q = -128; q <= 127; ++q) {
                double d_q = q / 128.0;
                u.iq[0] = i;
                u.iq[1] = q;
                lookup_phase_[u.iq16] = scaled_atan2(d_q, d_i);
                lookup_magsq_[u.iq16] = magsq(d_i, d_q);
            }
        }
    }

    void CS8Converter::ConvertPhase(uat::Bytes::const_iterator begin, uat::Bytes::const_iterator end, uat::PhaseBuffer::iterator out) {
        auto in_iq = reinterpret_cast<const cs8_alias *>(&*begin);

        // unroll the loop
        const auto n = std::distance(begin, end) / 2;
        const auto n8 = n / 8;
        const auto n7 = n & 7;

        for (auto i = 0; i < n8; ++i, in_iq += 8) {
            *out++ = lookup_phase_[in_iq[0].iq16];
            *out++ = lookup_phase_[in_iq[1].iq16];
            *out++ = lookup_phase_[in_iq[2].iq16];
            *out++ = lookup_phase_[in_iq[3].iq16];
            *out++ = lookup_phase_[in_iq[4].iq16];
            *out++ = lookup_phase_[in_iq[5].iq16];
            *out++ = lookup_phase_[in_iq[6].iq16];
            *out++ = lookup_phase_[in_iq[7].iq16];
        }
        for (auto i = 0; i < n7; ++i, ++in_iq) {
            *out++ = lookup_phase_[in_iq[0].iq16];
        }
    }

    void CS8Converter::ConvertMagSq(uat::Bytes::const_iterator begin, uat::Bytes::const_iterator end, std::vector<double>::iterator out) {
        auto in_iq = reinterpret_cast<const cs8_alias *>(&*begin);

        // unroll the loop
        const auto n = std::distance(begin, end) / 2;
        const auto n8 = n / 8;
        const auto n7 = n & 7;

        for (auto i = 0; i < n8; ++i, in_iq += 8) {
            *out++ = lookup_magsq_[in_iq[0].iq16];
            *out++ = lookup_magsq_[in_iq[1].iq16];
            *out++ = lookup_magsq_[in_iq[2].iq16];
            *out++ = lookup_magsq_[in_iq[3].iq16];
            *out++ = lookup_magsq_[in_iq[4].iq16];
            *out++ = lookup_magsq_[in_iq[5].iq16];
            *out++ = lookup_magsq_[in_iq[6].iq16];
            *out++ = lookup_magsq_[in_iq[7].iq16];
        }
        for (auto i = 0; i < n7; ++i, ++in_iq) {
            *out++ = lookup_magsq_[in_iq[0].iq16];
        }
    }

    void CS16HConverter::ConvertPhase(uat::Bytes::const_iterator begin, uat::Bytes::const_iterator end, uat::PhaseBuffer::iterator out) {
        auto in_iq = reinterpret_cast<const std::int16_t *>(&*begin);

        // unroll the loop
        const auto n = std::distance(begin, end) / 4;
        const auto n8 = n / 8;
        const auto n7 = n & 7;

        for (auto i = 0; i < n8; ++i, in_iq += 16) {
            *out++ = scaled_atan2(in_iq[1], in_iq[0]);
            *out++ = scaled_atan2(in_iq[3], in_iq[2]);
            *out++ = scaled_atan2(in_iq[5], in_iq[4]);
            *out++ = scaled_atan2(in_iq[7], in_iq[6]);
            *out++ = scaled_atan2(in_iq[9], in_iq[8]);
            *out++ = scaled_atan2(in_iq[11], in_iq[10]);
            *out++ = scaled_atan2(in_iq[13], in_iq[12]);
            *out++ = scaled_atan2(in_iq[15], in_iq[14]);
        }
        for (auto i = 0; i < n7; ++i, in_iq += 2) {
            *out++ = scaled_atan2(in_iq[1], in_iq[0]);
        }
    }

    void CS16HConverter::ConvertMagSq(uat::Bytes::const_iterator begin, uat::Bytes::const_iterator end, std::vector<double>::iterator out) {
        auto in_iq = reinterpret_cast<const std::int16_t *>(&*begin);

        // unroll the loop
        const auto n = std::distance(begin, end) / 4;
        const auto n8 = n / 8;
        const auto n7 = n & 7;

        for (auto i = 0; i < n8; ++i, in_iq += 16) {
            *out++ = magsq(in_iq[1], in_iq[0]) / 32768.0 / 32768.0;
            *out++ = magsq(in_iq[3], in_iq[2]) / 32768.0 / 32768.0;
            *out++ = magsq(in_iq[5], in_iq[4]) / 32768.0 / 32768.0;
            *out++ = magsq(in_iq[7], in_iq[6]) / 32768.0 / 32768.0;
            *out++ = magsq(in_iq[9], in_iq[8]) / 32768.0 / 32768.0;
            *out++ = magsq(in_iq[11], in_iq[10]) / 32768.0 / 32768.0;
            *out++ = magsq(in_iq[13], in_iq[12]) / 32768.0 / 32768.0;
            *out++ = magsq(in_iq[15], in_iq[14]) / 32768.0 / 32768.0;
        }
        for (auto i = 0; i < n7; ++i, in_iq += 2) {
            *out++ = magsq(in_iq[1], in_iq[0]) / 32768.0 / 32768.0;
        }
    }

    void CF32HConverter::ConvertPhase(uat::Bytes::const_iterator begin, uat::Bytes::const_iterator end, uat::PhaseBuffer::iterator out) {
        auto in_iq = reinterpret_cast<const double *>(&*begin);

        // unroll the loop
        const auto n = std::distance(begin, end) / 8;
        const auto n8 = n / 8;
        const auto n7 = n & 7;

        for (auto i = 0; i < n8; ++i, in_iq += 16) {
            *out++ = scaled_atan2(in_iq[1], in_iq[0]);
            *out++ = scaled_atan2(in_iq[3], in_iq[2]);
            *out++ = scaled_atan2(in_iq[5], in_iq[4]);
            *out++ = scaled_atan2(in_iq[7], in_iq[6]);
            *out++ = scaled_atan2(in_iq[9], in_iq[8]);
            *out++ = scaled_atan2(in_iq[11], in_iq[10]);
            *out++ = scaled_atan2(in_iq[13], in_iq[12]);
            *out++ = scaled_atan2(in_iq[15], in_iq[14]);
        }
        for (auto i = 0; i < n7; ++i, in_iq += 2) {
            *out++ = scaled_atan2(in_iq[1], in_iq[0]);
        }
    }

    void CF32HConverter::ConvertMagSq(uat::Bytes::const_iterator begin, uat::Bytes::const_iterator end, std::vector<double>::iterator out) {
        auto in_iq = reinterpret_cast<const double *>(&*begin);

        // unroll the loop
        const auto n = std::distance(begin, end) / 8;
        const auto n8 = n / 8;
        const auto n7 = n & 7;

        for (auto i = 0; i < n8; ++i, in_iq += 16) {
            *out++ = magsq(in_iq[1], in_iq[0]);
            *out++ = magsq(in_iq[3], in_iq[2]);
            *out++ = magsq(in_iq[5], in_iq[4]);
            *out++ = magsq(in_iq[7], in_iq[6]);
            *out++ = magsq(in_iq[9], in_iq[8]);
            *out++ = magsq(in_iq[11], in_iq[10]);
            *out++ = magsq(in_iq[13], in_iq[12]);
            *out++ = magsq(in_iq[15], in_iq[14]);
        }
        for (auto i = 0; i < n7; ++i, in_iq += 2) {
            *out++ = magsq(in_iq[1], in_iq[0]);
        }
    }
} // namespace dump978
