// -*- c++ -*-

// Copyright (c) 2019, FlightAware LLC.
// All rights reserved.
// Licensed under the 2-clause BSD license; see the LICENSE file

#ifndef UAT_COMMON_H
#define UAT_COMMON_H

#include <cmath>
#include <cstdint>
#include <vector>

namespace uat {
    typedef std::vector<std::uint8_t> Bytes;
    typedef std::vector<std::uint16_t> PhaseBuffer;

    inline static double RoundN(double value, unsigned dp) {
        const double scale = std::pow(10, dp);
        return std::round(value * scale) / scale;
    }
}; // namespace uat

#endif
