// Copyright (c) 2019, FlightAware LLC.
// All rights reserved.
// Licensed under the 2-clause BSD license; see the LICENSE file

#include "fec.h"
#include "uat_protocol.h"

extern "C" {
#include "fec/rs.h"
}

using namespace flightaware::uat;
using namespace flightaware::uat::fec;

FEC::FEC(void) {
    rs_downlink_short_ = ::init_rs_char(
        /* symsize */ 8, /* gfpoly */ DOWNLINK_SHORT_POLY, /* fcr */ 120,
        /* prim */ 1, /* nroots */ DOWNLINK_SHORT_ROOTS,
        /* pad */ DOWNLINK_SHORT_PAD);
    rs_downlink_long_ = ::init_rs_char(
        /* symsize */ 8, /* gfpoly */ DOWNLINK_LONG_POLY, /* fcr */ 120,
        /* prim */ 1, /* nroots */ DOWNLINK_LONG_ROOTS,
        /* pad */ DOWNLINK_LONG_PAD);
    rs_uplink_ = ::init_rs_char(/* symsize */ 8, /* gfpoly */ UPLINK_BLOCK_POLY,
                                /* fcr */ 120, /* prim */ 1,
                                /* nroots */ UPLINK_BLOCK_ROOTS,
                                /* pad */ UPLINK_BLOCK_PAD);
}

FEC::~FEC(void) {
    ::free_rs_char(rs_downlink_short_);
    ::free_rs_char(rs_downlink_long_);
    ::free_rs_char(rs_uplink_);
}

std::tuple<bool, Bytes, unsigned> FEC::CorrectDownlink(const Bytes &raw, const std::vector<std::size_t> &erasures) {
    using R = std::tuple<bool, Bytes, unsigned>;

    if (raw.size() != DOWNLINK_LONG_BYTES) {
        return R{false, {}, 0};
    }

    if (erasures.size() > DOWNLINK_LONG_ROOTS) {
        // too many
        return R{false, {}, 0};
    }

    // Try decoding as a Long UAT.
    Bytes corrected;
    std::copy(raw.begin(), raw.end(), std::back_inserter(corrected));

    int erasures_array[DOWNLINK_LONG_ROOTS];
    for (std::size_t i = 0; i < erasures.size(); ++i) {
        erasures_array[i] = erasures[i] + DOWNLINK_LONG_PAD;
        corrected[erasures[i]] = 0;
    }
    int n_corrected = ::decode_rs_char(rs_downlink_long_, corrected.data(), erasures_array, erasures.size());
    if (n_corrected >= 0 && n_corrected <= DOWNLINK_LONG_ROOTS && (corrected[0] >> 3) != 0) {
        // Valid long frame.
        corrected.resize(DOWNLINK_LONG_DATA_BYTES);
        return R{true, std::move(corrected), n_corrected};
    }

    // Retry as Basic UAT
    // We rely on decode_rs_char not modifying the data if there were
    // uncorrectable errors in the previous step.

    // Only pass in erasures that lie within the short message length
    int short_erasures = 0;
    for (auto e : erasures) {
        if (e < DOWNLINK_SHORT_BYTES) {
            if (short_erasures < DOWNLINK_SHORT_ROOTS) {
                erasures_array[short_erasures] = e + DOWNLINK_SHORT_PAD;
            }
            ++short_erasures;
        }
    }

    if (short_erasures > DOWNLINK_SHORT_ROOTS) {
        // too many
        return R{false, {}, 0};
    }

    n_corrected = ::decode_rs_char(rs_downlink_short_, corrected.data(), erasures_array, short_erasures);
    if (n_corrected >= 0 && n_corrected <= DOWNLINK_SHORT_ROOTS && (corrected[0] >> 3) == 0) {
        // Valid short frame
        corrected.resize(DOWNLINK_SHORT_DATA_BYTES);
        return R{true, std::move(corrected), n_corrected};
    }

    // Failed.
    return R{false, {}, 0};
}

std::tuple<bool, Bytes, unsigned> FEC::CorrectUplink(const Bytes &raw, const std::vector<std::size_t> &erasures) {
    using R = std::tuple<bool, Bytes, unsigned>;

    if (raw.size() != UPLINK_BYTES) {
        return R{false, {}, 0};
    }

    // uplink messages consist of 6 blocks, interleaved; each block consists of a
    // data section then an ECC section; we need to deinterleave, check/correct
    // the data, then join the blocks removing the ECC sections.
    unsigned total_errors = 0;
    Bytes corrected;
    Bytes blockdata;

    corrected.reserve(UPLINK_DATA_BYTES);
    blockdata.resize(UPLINK_BLOCK_BYTES);

    for (unsigned block = 0; block < UPLINK_BLOCKS_PER_FRAME; ++block) {
        // deinterleave
        for (unsigned i = 0; i < UPLINK_BLOCK_BYTES; ++i) {
            blockdata[i] = raw[i * UPLINK_BLOCKS_PER_FRAME + block];
        }

        // build erasures for this block
        int block_erasures[UPLINK_BLOCK_ROOTS];
        int num_erasures = 0;
        for (auto index : erasures) {
            if (index % UPLINK_BLOCKS_PER_FRAME == block) {
                if (num_erasures < UPLINK_BLOCK_ROOTS)
                    block_erasures[num_erasures] = index / UPLINK_BLOCKS_PER_FRAME + UPLINK_BLOCK_PAD;
                num_erasures++;
            }
        }

        // too many erasures in this block?
        if (num_erasures > UPLINK_BLOCK_ROOTS) {
            return R{false, {}, 0};
        }

        // error-correct
        int n_corrected = ::decode_rs_char(rs_uplink_, blockdata.data(), block_erasures, num_erasures);
        if (n_corrected < 0 || n_corrected > UPLINK_BLOCK_ROOTS) {
            // Failed
            return R{false, {}, 0};
        }

        total_errors += n_corrected;

        // copy the data into the right place
        std::copy(blockdata.begin(), blockdata.begin() + UPLINK_BLOCK_DATA_BYTES, std::back_inserter(corrected));
    }

    return R{true, std::move(corrected), total_errors};
}
