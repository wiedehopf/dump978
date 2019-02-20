// Copyright 2015, Oliver Jowett <oliver@mutability.co.uk>
// Copyright (c) 2019, FlightAware LLC.
// All rights reserved.
// Licensed under the 2-clause BSD license; see the LICENSE file

#include "demodulator.h"

#include <assert.h>
#include <iostream>
#include <iomanip>

using namespace uat;

namespace dump978 {
    SingleThreadReceiver::SingleThreadReceiver(SampleFormat format)
        : converter_(SampleConverter::Create(format)),
          demodulator_(new TwoMegDemodulator())
    {}

    // Handle samples in 'buffer' by:
    //   converting them to a phase buffer
    //   demodulating the phase buffer
    //   dispatching any demodulated messages
    //   preserving the end of the phase buffer for reuse in the next call
    void SingleThreadReceiver::HandleSamples(std::uint64_t timestamp, const Bytes &buffer) {
        assert(converter_);
        converter_->Convert(buffer, phase_); // appends to phase_

        auto messages = demodulator_->Demodulate(timestamp, phase_); // FIXME: not correct because of the preamble
        if (messages && !messages->empty()) {
            DispatchMessages(messages);
        }

        // preserve the tail of the phase buffer for next time
        const auto tail_size = demodulator_->NumTrailingSamples();
        if (phase_.size() > tail_size) {
            std::copy(phase_.end() - tail_size, phase_.end(), phase_.begin());
            phase_.resize(tail_size);
        }
    }

    static inline std::int16_t PhaseDifference(std::uint16_t from, std::uint16_t to) {
        int32_t difference = to - from; // lies in the range -65535 .. +65535
        if (difference >= 32768)        //   +32768..+65535
            return difference - 65536;  //   -> -32768..-1: always in range
        else if (difference < -32768)   //   -65535..-32769
            return difference + 65536;  //   -> +1..32767: always in range
        else
            return difference;
    }

    static inline bool SyncWordMatch(std::uint64_t word, std::uint64_t expected) {
        std::uint64_t diff;

        if (word == expected)
            return 1;

        diff = word ^ expected; // guaranteed nonzero

        // This is a bit-twiddling popcount
        // hack, tweaked as we only care about
        // "<N" or ">=N" set bits for fixed N -
        // so we can bail out early after seeing N
        // set bits.
        //
        // It relies on starting with a nonzero value
        // with zero or more trailing clear bits
        // after the last set bit:
        //
        //    010101010101010000
        //                 ^
        // Subtracting one, will flip the
        // bits starting at the last set bit:
        //
        //    010101010101001111
        //                 ^
        // then we can use that as a bitwise-and
        // mask to clear the lowest set bit:
        //
        //    010101010101000000
        //                 ^
        // And repeat until the value is zero
        // or we have seen too many set bits.

        // >= 1 bit
        diff &= (diff-1);   // clear lowest set bit
        if (!diff)
            return 1; // 1 bit error

        // >= 2 bits
        diff &= (diff-1);   // clear lowest set bit
        if (!diff)
            return 1; // 2 bits error

        // >= 3 bits
        diff &= (diff-1);   // clear lowest set bit
        if (!diff)
            return 1; // 3 bits error

        // >= 4 bits
        diff &= (diff-1);   // clear lowest set bit
        if (!diff)
            return 1; // 4 bits error

        // > 4 bits in error, give up
        return 0;
    }

    // check that there is a valid sync word starting at 'start'
    // that matches the sync word 'pattern'. Return a pair:
    // first element is true if the sync word looks OK; second
    // element has the dphi threshold to use for bit slicing
    static inline std::pair<bool,std::int16_t> CheckSyncWord(const PhaseBuffer &buffer, unsigned start, std::uint64_t pattern) {
        const unsigned MAX_SYNC_ERRORS = 4;

        std::int32_t dphi_zero_total = 0;
        int zero_bits = 0;
        std::int32_t dphi_one_total = 0;
        int one_bits = 0;

        // find mean dphi for zero and one bits;
        // take the mean of the two as our central value

        for (unsigned i = 0; i < SYNC_BITS; ++i) {
            auto dphi = PhaseDifference(buffer[start + i * 2], buffer[start + i * 2 + 1]);
            if (pattern & (1UL << (35-i))) {
                ++one_bits;
                dphi_one_total += dphi;
            } else {
                ++zero_bits;
                dphi_zero_total += dphi;
            }
        }

        dphi_zero_total /= zero_bits;
        dphi_one_total /= one_bits;

        std::int16_t center = (dphi_one_total + dphi_zero_total) / 2;

        // recheck sync word using our center value
        unsigned error_bits = 0;
        for (unsigned i = 0; i < SYNC_BITS; ++i) {
            auto dphi = PhaseDifference(buffer[start + i * 2], buffer[start + i * 2 + 1]);

            if (pattern & (1UL << (35-i))) {
                if (dphi < center)
                    ++error_bits;
            } else {
                if (dphi > center)
                    ++error_bits;
            }
        }

        return std::make_pair<>((error_bits <= MAX_SYNC_ERRORS), center);
    }

    // demodulate 'bytes' bytes from samples at 'start' using 'center' as the bit slicing threshold
    static inline Bytes DemodBits(const PhaseBuffer &buffer, unsigned start, unsigned bytes, std::int16_t center)
    {
        Bytes result;
        result.reserve(bytes);

        if (start + bytes * 8 * 2 >= buffer.size()) {
            throw std::runtime_error("would overrun source buffer");
        }

        auto *phase = buffer.data() + start;
        for (unsigned i = 0; i < bytes; ++i) {
            std::uint8_t b = 0;
            if (PhaseDifference(phase[0], phase[1]) > center) b |= 0x80;
            if (PhaseDifference(phase[2], phase[3]) > center) b |= 0x40;
            if (PhaseDifference(phase[4], phase[5]) > center) b |= 0x20;
            if (PhaseDifference(phase[6], phase[7]) > center) b |= 0x10;
            if (PhaseDifference(phase[8], phase[9]) > center) b |= 0x08;
            if (PhaseDifference(phase[10], phase[11]) > center) b |= 0x04;
            if (PhaseDifference(phase[12], phase[13]) > center) b |= 0x02;
            if (PhaseDifference(phase[14], phase[15]) > center) b |= 0x01;
            result.push_back(b);
            phase += 16;
        }

        return result;
    }

    unsigned TwoMegDemodulator::NumTrailingSamples() {
        return (SYNC_BITS + UPLINK_BITS) * 2;
    }

    // Try to demodulate messages from `buffer` and return a list of messages.
    // Messages that start near the end of `buffer` may not be demodulated
    // (less than (SYNC_BITS + UPLINK_BITS)*2 before the end of the buffer)
    SharedMessageVector TwoMegDemodulator::Demodulate(std::uint64_t timestamp, const PhaseBuffer &buffer) {
        // We expect samples at twice the UAT bitrate.
        // We look at phase difference between pairs of adjacent samples, i.e.
        //  sample 1 - sample 0   -> sync0
        //  sample 2 - sample 1   -> sync1
        //  sample 3 - sample 2   -> sync0
        //  sample 4 - sample 3   -> sync1
        // ...
        //
        // We accumulate bits into two buffers, sync0 and sync1.
        // Then we compare those buffers to the expected 36-bit sync word that
        // should be at the start of each UAT frame. When (if) we find it,
        // that tells us which sample to start decoding from.

        // Stop when we run out of remaining samples for a max-sized frame.
        // Arrange for our caller to pass the trailing data back to us next time;
        // ensure we don't consume any partial sync word we might be part-way
        // through. This means we don't need to maintain state between calls.

        SharedMessageVector messages = std::make_shared<MessageVector>();

        const auto trailing_samples = (SYNC_BITS + UPLINK_BITS) * 2 - 2;
        if (buffer.size() <= trailing_samples) {
            return messages;
        }
        const auto limit = buffer.size() - trailing_samples;

        unsigned sync_bits = 0;
        std::uint64_t sync0 = 0, sync1 = 0;
        const std::uint64_t SYNC_MASK = ((((std::uint64_t)1)<<SYNC_BITS)-1);

        for (unsigned i = 0; i < limit; i += 2) {
            auto d0 = PhaseDifference(buffer[i], buffer[i + 1]);
            auto d1 = PhaseDifference(buffer[i + 1], buffer[i + 2]);

            sync0 = ((sync0 << 1) | (d0 > 0 ? 1 : 0)) & SYNC_MASK;
            sync1 = ((sync1 << 1) | (d1 > 0 ? 1 : 0)) & SYNC_MASK;

            if (++sync_bits < SYNC_BITS)
                continue; // haven't fully populated sync0/1 yet

            // see if we have (the start of) a valid sync word
            // when we find a match, try to demodulate both with that match
            // and with the next position, and pick the one with fewer
            // errors.
            if (SyncWordMatch(sync0, DOWNLINK_SYNC_WORD)) {
                auto start = i - SYNC_BITS * 2 + 2;
                auto start_timestamp = timestamp + start * 1000 / 2083333;
                auto message = DemodBest(buffer, start, true /* downlink */, start_timestamp);
                if (message) {
                    i = start + message.BitLength() * 2;
                    sync_bits = 0;
                    messages->emplace_back(std::move(message));
                    continue;
                }
            }

            if (SyncWordMatch(sync1, DOWNLINK_SYNC_WORD)) {
                auto start = i - SYNC_BITS * 2 + 3;
                auto start_timestamp = timestamp + start * 1000 / 2083333;
                auto message = DemodBest(buffer, start, true /* downlink */, start_timestamp);
                if (message) {
                    i = start + message.BitLength() * 2;
                    sync_bits = 0;
                    messages->emplace_back(std::move(message));
                    continue;
                }
            }

            if (SyncWordMatch(sync0, UPLINK_SYNC_WORD)) {
                auto start = i - SYNC_BITS * 2 + 2;
                auto start_timestamp = timestamp + start * 1000 / 2083333;
                auto message = DemodBest(buffer, start, false /* !downlink */, start_timestamp);
                if (message) {
                    i = start + message.BitLength() * 2;
                    sync_bits = 0;
                    messages->emplace_back(std::move(message));
                    continue;
                }
            }

            if (SyncWordMatch(sync1, UPLINK_SYNC_WORD)) {
                auto start = i - SYNC_BITS * 2 + 3;
                auto start_timestamp = timestamp + start * 1000 / 2083333;
                auto message = DemodBest(buffer, start, false /* !downlink */, start_timestamp);
                if (message) {
                    i = start + message.BitLength() * 2;
                    sync_bits = 0;
                    messages->emplace_back(std::move(message));
                    continue;
                }
            }
        }

        return messages;
    }

    RawMessage TwoMegDemodulator::DemodBest(const PhaseBuffer &buffer, unsigned start, bool downlink, std::uint64_t timestamp) {
        auto message0 = downlink ? DemodOneDownlink(buffer, start, timestamp) : DemodOneUplink(buffer, start, timestamp);
        auto message1 = downlink ? DemodOneDownlink(buffer, start + 1, timestamp) : DemodOneUplink(buffer, start + 1, timestamp);

        unsigned errors0 = (message0 ? message0.Errors() : 9999);
        unsigned errors1 = (message1 ? message1.Errors() : 9999);

        if (errors0 <= errors1)
            return message0; // should be move-eligible
        else
            return message1; // should be move-eligible
    }

    RawMessage TwoMegDemodulator::DemodOneDownlink(const PhaseBuffer &buffer, unsigned start, std::uint64_t timestamp) {
        auto sync = CheckSyncWord(buffer, start, DOWNLINK_SYNC_WORD);
        if (!sync.first) {
            // Sync word had errors
            return RawMessage();
        }

        Bytes raw = DemodBits(buffer, start + SYNC_BITS*2, DOWNLINK_LONG_BYTES, sync.second);

        bool success;
        uat::Bytes corrected;
        unsigned errors;
        std::tie(success, corrected, errors) = fec_.CorrectDownlink(raw);
        if (!success) {
            // Error correction failed
            return RawMessage();
        }

        return RawMessage(std::move(corrected), timestamp, errors, 0);
    }

    RawMessage TwoMegDemodulator::DemodOneUplink(const PhaseBuffer &buffer, unsigned start, std::uint64_t timestamp) {
        auto sync = CheckSyncWord(buffer, start, UPLINK_SYNC_WORD);
        if (!sync.first) {
            // Sync word had errors
            return RawMessage();
        }

        Bytes raw = DemodBits(buffer, start + SYNC_BITS*2, UPLINK_BYTES, sync.second);

        bool success;
        uat::Bytes corrected;
        unsigned errors;
        std::tie(success, corrected, errors) = fec_.CorrectUplink(raw);

        if (!success) {
            // Error correction failed
            return RawMessage();
        }

        return RawMessage(std::move(corrected), timestamp, errors, 0);
    }
}
