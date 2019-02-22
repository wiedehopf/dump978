// Copyright (c) 2019, FlightAware LLC.
// All rights reserved.
// Licensed under the 2-clause BSD license; see the LICENSE file

#include "faup978_reporter.h"

#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>

using namespace uat;
using namespace faup978;

static const char *const TSV_VERSION = "4U";

void Reporter::Start() {
    std::cout << "tsv_version\t" << TSV_VERSION << std::endl;
    PeriodicReport();
}

void Reporter::Stop() { timer_.cancel(); }

template <typename T> static std::string value_map(T value, const std::map<T, std::string> &mappings, const std::string &default_value) {
    auto i = mappings.find(value);
    if (i == mappings.end()) {
        return default_value;
    } else {
        return i->second;
    }
}

template <typename T> static std::function<void(std::ostream &)> simple_emit(const T &field, int precision = 0) {
    return [&field, precision](std::ostream &os) { os << std::fixed << std::setprecision(precision) << field.Value(); };
}

void Reporter::PeriodicReport() {
    static auto unix_epoch = std::chrono::system_clock::from_time_t(0);
    const std::uint64_t now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - unix_epoch).count();

    for (const auto &entry : tracker_->Aircraft()) {
        ReportOneAircraft(entry.first, entry.second, now);
    }

    for (auto i = reported_.begin(); i != reported_.end();) {
        if (tracker_->Aircraft().count(i->first) == 0) {
            i = reported_.erase(i);
        } else {
            ++i;
        }
    }

    auto self(shared_from_this());
    timer_.expires_from_now(interval_);
    timer_.async_wait(strand_.wrap([this, self](const boost::system::error_code &ec) {
        if (!ec) {
            PeriodicReport();
        }
    }));
}

void Reporter::ReportOneAircraft(const uat::Tracker::AddressKey &key, const AircraftState &aircraft, std::uint64_t now) {
    auto &last = reported_[key];
    auto &last_state = last.report_state;

    bool changed = false;
    changed |= (last_state.pressure_altitude && aircraft.pressure_altitude && std::abs(last_state.pressure_altitude.Value() - aircraft.pressure_altitude.Value()) >= 50);
    changed |= (last_state.geometric_altitude && aircraft.geometric_altitude && std::abs(last_state.pressure_altitude.Value() - aircraft.pressure_altitude.Value()) >= 50);
    changed |= (last_state.vertical_velocity_barometric && aircraft.vertical_velocity_barometric && std::abs(last_state.vertical_velocity_barometric.Value() - aircraft.vertical_velocity_barometric.Value()) >= 500);
    changed |= (last_state.vertical_velocity_geometric && aircraft.vertical_velocity_geometric && std::abs(last_state.vertical_velocity_geometric.Value() - aircraft.vertical_velocity_geometric.Value()) >= 500);
    changed |= (last_state.true_track && aircraft.true_track && std::abs(last_state.true_track.Value() - aircraft.true_track.Value()) >= 2);
    changed |= (last_state.true_heading && aircraft.true_heading && std::abs(last_state.true_heading.Value() - aircraft.true_heading.Value()) >= 2);
    changed |= (last_state.magnetic_heading && aircraft.magnetic_heading && std::abs(last_state.magnetic_heading.Value() - aircraft.magnetic_heading.Value()) >= 2);
    changed |= (last_state.ground_speed && aircraft.ground_speed && std::abs(last_state.ground_speed.Value() - aircraft.ground_speed.Value()) >= 25);

    bool immediate = false;
    immediate |= (aircraft.selected_altitude_type.Changed() > last.report_time);
    immediate |= (aircraft.selected_altitude.Changed() > last.report_time);
    immediate |= (aircraft.selected_heading.Changed() > last.report_time);
    immediate |= (aircraft.mode_indicators.Changed() > last.report_time);
    immediate |= (aircraft.barometric_pressure_setting.Changed() > last.report_time);
    immediate |= (aircraft.callsign.Changed() > last.report_time);
    immediate |= (aircraft.flightplan_id.Changed() > last.report_time);
    immediate |= (aircraft.airground_state.Changed() > last.report_time);
    immediate |= (aircraft.emergency.Changed() > last.report_time);

    boost::optional<int> altitude = -10000; // keep the compiler happier
    if (aircraft.pressure_altitude.UpdateAge(now) < 30000)
        altitude = aircraft.pressure_altitude.Value();
    else if (aircraft.geometric_altitude.UpdateAge(now) < 30000)
        altitude = aircraft.geometric_altitude.Value();
    else
        altitude = boost::none;

    boost::optional<AirGroundState> airground = AirGroundState::RESERVED;
    if (aircraft.airground_state.UpdateAge(now) < 30000)
        airground = aircraft.airground_state.Value();
    else
        airground = boost::none;

    boost::optional<int> groundspeed = -10000;
    if (aircraft.ground_speed.UpdateAge(now) < 30000)
        groundspeed = aircraft.ground_speed.Value();
    else
        groundspeed = boost::none;

    std::uint64_t minAge;
    if (immediate) {
        // a change we want to emit right away
        minAge = 0;
    } else if (airground && airground.get() == AirGroundState::ON_GROUND) {
        // we are probably on the ground, increase the update rate
        minAge = 1000;
    } else if (altitude && altitude.get() < 500 && (!groundspeed || groundspeed.get() < 200)) {
        // we are probably on the ground, increase the update rate
        minAge = 1000;
    } else if (groundspeed && groundspeed.get() < 100 && (!altitude || altitude.get() < 1000)) {
        // we are probably on the ground, increase the update rate
        minAge = 1000;
    } else if (!altitude || altitude.get() < 10000) {
        // Below 10000 feet, emit up to every 5s when changing, 10s otherwise
        minAge = changed ? 5000 : 10000;
    } else {
        // Above 10000 feet, emit up to every 10s when changing, 30s otherwise
        minAge = changed ? 10000 : 30000;
    }

    bool force_slow = (now - last.slow_report_time) > 300000;

    if ((now - last.report_time) < minAge) {
        // Not this time.
        return;
    }

    std::vector<std::pair<std::string, std::string>> kv;

    static std::map<AddressQualifier, std::string> source_map = {{AddressQualifier::ADSB_ICAO, "A"}, {AddressQualifier::ADSB_OTHER, "A"}, {AddressQualifier::ADSR_OTHER, "A"}, {AddressQualifier::TISB_ICAO, "T"}, {AddressQualifier::TISB_OTHER, "T"}};
    std::string source = value_map(aircraft.address_qualifier, source_map, "?");

    auto add_slow_field = [&kv, &source, &last, force_slow](const std::string &k, const AgedFieldBase &f, std::function<void(std::ostream &)> stringize) {
        if (f.Valid() && (force_slow || f.Changed() > last.report_time)) {
            std::ostringstream os;
            stringize(os);
            kv.emplace_back(k, os.str());
        }
    };

    auto add_slow_aged_field = [&kv, &source, &last, now, force_slow](const std::string &k, const AgedFieldBase &f, std::function<void(std::ostream &)> stringize) {
        if (f.Valid() && (force_slow || f.Changed() > last.report_time)) {
            std::ostringstream os;
            stringize(os);
            os << ' ' << (f.UpdateAge(now) / 1000) << ' ' << source;
            kv.emplace_back(k, os.str());
        }
    };

    auto add_aged_field = [&kv, &source, &last, now](const std::string &k, const AgedFieldBase &f, std::function<void(std::ostream &)> stringize) {
        if (f.Valid() && f.Updated() > last.report_time) {
            std::ostringstream os;
            stringize(os);
            os << ' ' << (f.UpdateAge(now) / 1000) << ' ' << source;
            kv.emplace_back(k, os.str());
        }
    };

    add_slow_field("adsb_version", aircraft.mops_version, simple_emit(aircraft.mops_version));
    add_slow_field("category", aircraft.emitter_category, [&aircraft](std::ostream &os) { os << std::hex << std::uppercase << std::setfill('0') << std::setw(2) << (aircraft.emitter_category.Value() + 0xA0); });

    add_slow_aged_field("nac_p", aircraft.nac_p, simple_emit(aircraft.nac_p));
    add_slow_aged_field("nac_v", aircraft.nac_v, simple_emit(aircraft.nac_v));
    add_slow_aged_field("sil", aircraft.sil, simple_emit(aircraft.sil));
    add_slow_aged_field("sil_type", aircraft.sil_supplement, [&aircraft](std::ostream &os) {
        static std::map<SILSupplement, std::string> supplement_map = {
            {SILSupplement::PER_HOUR, "per_hour"},
            {SILSupplement::PER_SAMPLE, "per_sample"},
        };
        os << value_map(aircraft.sil_supplement.Value(), supplement_map, "unknown");
    });
    add_slow_aged_field("nic_baro", aircraft.nic_baro, simple_emit(aircraft.nic_baro));

    add_aged_field("airGround", aircraft.airground_state, [&aircraft](std::ostream &os) {
        static std::map<AirGroundState, std::string> airground_map = {{AirGroundState::AIRBORNE_SUBSONIC, "A+"}, {AirGroundState::AIRBORNE_SUPERSONIC, "A+"}, {AirGroundState::ON_GROUND, "A+"}};
        os << value_map(aircraft.airground_state.Value(), airground_map, "?");
    });
    add_aged_field("squawk", aircraft.flightplan_id, [&aircraft](std::ostream &os) { os << '{' << aircraft.flightplan_id.Value() << '}'; });
    add_aged_field("ident", aircraft.callsign, [&aircraft](std::ostream &os) { os << '{' << aircraft.callsign.Value() << '}'; });
    add_aged_field("alt", aircraft.pressure_altitude, simple_emit(aircraft.pressure_altitude));
    add_aged_field("position", aircraft.position, [&aircraft](std::ostream &os) {
        auto &p = aircraft.position.Value();
        unsigned nic = aircraft.nic.Valid() ? aircraft.nic.Value() : 0; // should always be valid if the position is
        double rc = aircraft.horizontal_containment.Valid() ? aircraft.horizontal_containment.Value() : 0;
        os << '{' << std::fixed << std::setprecision(5) << p.first << ' ' << p.second << ' ' << nic << ' ' << std::setprecision(0) << std::ceil(rc) << '}';
    });
    add_aged_field("alt_gnss", aircraft.geometric_altitude, simple_emit(aircraft.geometric_altitude));
    add_aged_field("vrate", aircraft.vertical_velocity_barometric, simple_emit(aircraft.vertical_velocity_barometric));
    add_aged_field("vrate_geom", aircraft.vertical_velocity_geometric, simple_emit(aircraft.vertical_velocity_geometric));
    add_aged_field("speed", aircraft.ground_speed, simple_emit(aircraft.ground_speed));
    add_aged_field("track", aircraft.true_track, simple_emit(aircraft.true_track, 1));
    add_aged_field("heading_magnetic", aircraft.magnetic_heading, simple_emit(aircraft.magnetic_heading, 1));
    add_aged_field("heading_true", aircraft.true_heading, simple_emit(aircraft.true_heading, 1));
    add_aged_field("nav_alt", aircraft.selected_altitude, simple_emit(aircraft.selected_altitude));
    add_aged_field("nav_heading", aircraft.selected_heading, simple_emit(aircraft.selected_heading));
    // todo: nav_modes
    add_aged_field("nav_qnh", aircraft.barometric_pressure_setting, simple_emit(aircraft.barometric_pressure_setting, 1));
    add_aged_field("emergency", aircraft.emergency, [&aircraft](std::ostream &os) {
        static std::map<EmergencyPriorityStatus, std::string> emergency_map = {{EmergencyPriorityStatus::NONE, "none"}, {EmergencyPriorityStatus::GENERAL, "general"}, {EmergencyPriorityStatus::MEDICAL, "medical"}, {EmergencyPriorityStatus::MINFUEL, "minfuel"}, {EmergencyPriorityStatus::NORDO, "nordo"}, {EmergencyPriorityStatus::UNLAWFUL, "unlawful"}, {EmergencyPriorityStatus::DOWNED, "downed"}};
        os << value_map(aircraft.emergency.Value(), emergency_map, "unknown");
    });

    // did we actually generate anything?
    if (kv.empty()) {
        return;
    }

    // generate the line
    std::cout << "clock" << '\t' << (now / 1000) << '\t';

    static std::map<AddressQualifier, std::string> idtype_map = {{AddressQualifier::ADSB_ICAO, "hexid"}, {AddressQualifier::TISB_ICAO, "hexid"}};

    std::cout << value_map(aircraft.address_qualifier, idtype_map, "otherid") << '\t' << std::hex << std::uppercase << std::setfill('0') << std::setw(6) << aircraft.address << std::dec << std::nouppercase << std::setfill(' ') << '\t';

    if (force_slow) {
        static std::map<AddressQualifier, std::string> qualifier_map = {{AddressQualifier::ADSB_ICAO, "adsb_icao"}, {AddressQualifier::ADSB_OTHER, "adsb_other"}, {AddressQualifier::TISB_ICAO, "tisb_icao"}, {AddressQualifier::TISB_OTHER, "tisb_other"}, {AddressQualifier::VEHICLE, "vehicle"}, {AddressQualifier::FIXED_BEACON, "fixed_beacon"}, {AddressQualifier::ADSR_OTHER, "adsr_other"}};
        std::cout << "addrtype" << '\t' << value_map(aircraft.address_qualifier, qualifier_map, "unknown");
    }

    for (const auto &entry : kv) {
        std::cout << '\t' << entry.first << '\t' << entry.second;
    }

    std::cout << std::endl;

    if (force_slow)
        last.slow_report_time = now;
    last.report_time = now;
    last.report_state = aircraft;
}
