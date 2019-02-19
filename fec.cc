// Copyright (c) 2019, FlightAware LLC.
// All rights reserved.
// Licensed under the 2-clause BSD license; see the LICENSE file

#include "fec.h"
#include "uat_protocol.h"

extern "C" {
#include "fec/rs.h"
}

uat::FEC::FEC(void)
{
    rs_downlink_short_ = ::init_rs_char(8, /* gfpoly */ DOWNLINK_POLY, /* fcr */ 120, /* prim */ 1, /* nroots */ 12, /* pad */ 225);
    rs_downlink_long_ = ::init_rs_char(8, /* gfpoly */ DOWNLINK_POLY, /* fcr */ 120, /* prim */ 1, /* nroots */ 14, /* pad */ 207);
    rs_uplink_ = ::init_rs_char(8, /* gfpoly */ UPLINK_POLY, /* fcr */ 120, /* prim */ 1, /* nroots */ 20, /* pad */ 163);
}

uat::FEC::~FEC(void)
{
    ::free_rs_char(rs_downlink_short_);
    ::free_rs_char(rs_downlink_long_);
    ::free_rs_char(rs_uplink_);
}

std::tuple<bool,uat::Bytes,unsigned> uat::FEC::CorrectDownlink(const Bytes &raw)
{
    using R = std::tuple<bool,uat::Bytes,unsigned>;

    if (raw.size() != DOWNLINK_LONG_BYTES) {
        return R { false, {}, 0 };
    }

    // Try decoding as a Long UAT.
    Bytes corrected;
    std::copy(raw.begin(), raw.end(), std::back_inserter(corrected));
    
    int n_corrected = ::decode_rs_char(rs_downlink_long_, corrected.data(), NULL, 0);
    if (n_corrected >= 0 && n_corrected <= 7 && (corrected[0]>>3) != 0) {
        // Valid long frame.
        corrected.resize(DOWNLINK_LONG_DATA_BYTES);
        return R { true, std::move(corrected), n_corrected };
    }

    // Retry as Basic UAT
    // We rely on decode_rs_char not modifying the data if there were
    // uncorrectable errors in the previous step.
    n_corrected = ::decode_rs_char(rs_downlink_short_, corrected.data(), NULL, 0);
    if (n_corrected >= 0 && n_corrected <= 6 && (corrected[0]>>3) == 0) {
        // Valid short frame
        corrected.resize(DOWNLINK_SHORT_DATA_BYTES);
        return R { true, std::move(corrected), n_corrected };
    }

    // Failed.
    return R { false, {}, 0 };
}

std::tuple<bool,uat::Bytes,unsigned> uat::FEC::CorrectUplink(const Bytes &raw)
{
    using R = std::tuple<bool,uat::Bytes,unsigned>;

    if (raw.size() != UPLINK_BYTES) {
        return R { false, {}, 0 };
    }

    // uplink messages consist of 6 blocks, interleaved; each block consists of a data section
    // then an ECC section; we need to deinterleave, check/correct the data, then join the blocks
    // removing the ECC sections.
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

        // error-correct
        int n_corrected = ::decode_rs_char(rs_uplink_, blockdata.data(), NULL, 0);
        if (n_corrected < 0 || n_corrected > 10) {
            // Failed
            return R { false, {}, 0 };
        }

        total_errors += n_corrected;

        // copy the data into the right place
        std::copy(blockdata.begin(), blockdata.begin() + UPLINK_BLOCK_DATA_BYTES, std::back_inserter(corrected));
    }

    return R { true, std::move(corrected), total_errors };
}
