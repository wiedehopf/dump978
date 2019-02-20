// Copyright (c) 2019, FlightAware LLC.
// All rights reserved.
// Licensed under the 2-clause BSD license; see the LICENSE file

#include "track.h"

using namespace uat;

#include <iostream>
#include <iomanip>

void AircraftState::UpdateFromMessage(std::uint64_t at, const uat::AdsbMessage &message)
{
#define UPDATE(x) do { if (message.x) { x.MaybeUpdate(at, *message.x); } } while(0)

    UPDATE(position);  // latitude, longitude
    UPDATE(pressure_altitude);
    UPDATE(geometric_altitude);
    UPDATE(nic);
    UPDATE(airground_state);
    UPDATE(north_velocity);
    UPDATE(east_velocity);
    UPDATE(vertical_velocity_barometric);
    UPDATE(vertical_velocity_geometric);
    UPDATE(ground_speed);
    UPDATE(magnetic_heading);
    UPDATE(true_heading);
    UPDATE(true_track);
    UPDATE(aircraft_size); // length, width
    UPDATE(gps_lateral_offset);
    UPDATE(gps_longitudinal_offset);
    UPDATE(gps_position_offset_applied);
    UPDATE(utc_coupled);

    UPDATE(emitter_category);
    UPDATE(callsign);
    UPDATE(flightplan_id);   // aka Mode 3/A squawk
    UPDATE(emergency);
    UPDATE(mops_version);
    UPDATE(sil);
    UPDATE(transmit_mso);
    UPDATE(sda);
    UPDATE(nac_p);
    UPDATE(nac_v);
    UPDATE(nic_baro);
    UPDATE(capability_codes);
    UPDATE(operational_modes);        
    UPDATE(sil_supplement);
    UPDATE(gva);
    UPDATE(single_antenna);
    UPDATE(nic_supplement);

    UPDATE(selected_altitude_type);
    UPDATE(selected_altitude);
    UPDATE(barometric_pressure_setting);
    UPDATE(selected_heading);
    UPDATE(mode_indicators);

    last_message_time = std::max(last_message_time, at);

#undef UPDATE
}

void Tracker::Start()
{
    PurgeOld(); // starts timer
}

void Tracker::Stop()
{
    timer_.cancel();
}

void Tracker::PurgeOld()
{
    static auto unix_epoch = std::chrono::system_clock::from_time_t(0);
    auto expires = std::chrono::system_clock::now() - timeout_;
    std::uint64_t expires_timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(expires - unix_epoch).count();

    std::cerr << "starting to expiry across " << aircraft_.size() << " aircraft, expiry time " << expires_timestamp << std::endl;
    for (auto i = aircraft_.begin(); i != aircraft_.end(); ) {
        if (i->second.last_message_time < expires_timestamp) {
            std::cerr << "expire " << std::hex << std::setfill('0') << std::setw(6) << i->second.address << std::dec << std::setfill(' ') << " with last time " << i->second.last_message_time << std::endl;            
            i = aircraft_.erase(i);
        } else {
            ++i;
        }
    }
    std::cerr << "done expiring, now have " << aircraft_.size() << " aircraft" << std::endl;

    auto self(shared_from_this());
    timer_.expires_from_now(timeout_ / 4);
    timer_.async_wait(strand_.wrap([this,self](const boost::system::error_code &ec) {
                if (!ec) {
                    PurgeOld();
                }
            }));
}

void Tracker::HandleMessages(SharedMessageVector messages)
{
    auto self(shared_from_this());
    strand_.dispatch([this,self,messages]() {
            for (const auto &message : *messages) {
                if (message.Type() == MessageType::DOWNLINK_SHORT || message.Type() == MessageType::DOWNLINK_LONG) {
                    HandleMessage(message.ReceivedAt(), AdsbMessage(message));
                }
            }
        });
}

void Tracker::HandleMessage(std::uint64_t at, const uat::AdsbMessage &message)
{
    AddressKey key { message.address_qualifier, message.address };
    auto i = aircraft_.find(key);
    if (i == aircraft_.end()) {
        std::cerr << "new aircraft: " << (int)message.address_qualifier << "/" << std::hex << std::setfill('0') << std::setw(6) << message.address << std::dec << std::setfill(' ') << std::endl;
        aircraft_[key] = { message.address_qualifier, message.address };
    }

    aircraft_[key].UpdateFromMessage(at, message);
}
