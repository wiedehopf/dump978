#include "skyview_writer.h"

#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

#include "json.hpp"

#include "track.h"

using namespace uat;
using namespace uat::skyview;

void SkyviewWriter::Start() {
    nlohmann::json receiver_json;

    receiver_json["version"] = "dump978-WIP";
    receiver_json["refresh"] = interval_.count();
    receiver_json["history"] = 1;

    std::ofstream receiver_file((dir_ / "receiver.json").native());
    receiver_file << std::setw(4) << receiver_json << std::endl;

    PeriodicWrite();
}

void SkyviewWriter::Stop() { timer_.cancel(); }

void SkyviewWriter::PeriodicWrite() {
    using json = nlohmann::json;
    json aircraft_json;

    auto now = now_millis();

    aircraft_json["now"] = now / 1000.0;
    aircraft_json["messages"] = 100; // TODO

    auto &aircraft_list = aircraft_json["aircraft"] = json::array();

    for (const auto &entry : tracker_->Aircraft()) {
        auto &aircraft = entry.second;

        aircraft_list.emplace_back(json::object());
        auto &ac_json = aircraft_list.back();

        std::ostringstream os;
        if (aircraft.address_qualifier != AddressQualifier::ADSB_ICAO && aircraft.address_qualifier != AddressQualifier::TISB_ICAO)
            os << '~';
        os << std::hex << std::setfill('0') << std::setw(6) << aircraft.address;
        ac_json["hex"] = os.str();

        // qualifier..
        switch (aircraft.address_qualifier) {
        case AddressQualifier::TISB_ICAO:
            ac_json["type"] = "tisb_icao";
            ac_json["tisb"] = json::array({"lat", "lon"});
            break;
        case AddressQualifier::TISB_TRACKFILE:
            ac_json["type"] = "tisb_trackfile";
            ac_json["tisb"] = json::array({"lat", "lon"});
            break;
        case AddressQualifier::ADSB_ICAO:
            ac_json["type"] = "adsb_icao";
            break;
        default:
            break;
        }

        if (aircraft.position) {
            ac_json["lat"] = aircraft.position.Value().first;
            ac_json["lon"] = aircraft.position.Value().second;
            ac_json["seen_pos"] = aircraft.position.UpdateAge(now) / 1000.0;
        }
        if (aircraft.pressure_altitude) {
            ac_json["alt_baro"] = aircraft.pressure_altitude.Value();
        }
        if (aircraft.geometric_altitude) {
            ac_json["alt_geom"] = aircraft.pressure_altitude.Value();
        }
        if (aircraft.nic) {
            ac_json["nic"] = aircraft.nic.Value();
        }
        if (aircraft.airground_state && aircraft.airground_state.Value() == AirGroundState::ON_GROUND) {
            ac_json["alt_baro"] = "ground";
        }
        if (aircraft.vertical_velocity_barometric) {
            ac_json["baro_rate"] = aircraft.vertical_velocity_barometric.Value();
        }
        if (aircraft.vertical_velocity_geometric) {
            ac_json["geom_rate"] = aircraft.vertical_velocity_geometric.Value();
        }
        if (aircraft.ground_speed) {
            ac_json["gs"] = aircraft.ground_speed.Value();
        }
        if (aircraft.magnetic_heading) {
            ac_json["mag_heading"] = aircraft.magnetic_heading.Value();
        }
        if (aircraft.true_heading) {
            ac_json["true_heading"] = aircraft.true_heading.Value();
        }
        if (aircraft.true_track) {
            ac_json["track"] = aircraft.true_track.Value();
        }
        if (aircraft.emitter_category) {
            std::ostringstream os;
            os << std::hex << std::setfill('0') << std::setw(2) << (aircraft.emitter_category.Value() + 0xA0);
            ac_json["category"] = os.str();
        }
        if (aircraft.callsign) {
            ac_json["flight"] = aircraft.callsign.Value();
        }
        if (aircraft.flightplan_id) {
            ac_json["squawk"] = aircraft.flightplan_id.Value();
        }
        if (aircraft.emergency) {
            ac_json["emergency"] = aircraft.emergency.Value();
        }
        if (aircraft.mops_version) {
            ac_json["version"] = aircraft.mops_version.Value(); // FIXME
        }
        if (aircraft.sil) {
            ac_json["sil"] = aircraft.sil.Value();
        }
        if (aircraft.sda) {
            ac_json["sda"] = aircraft.sda.Value();
        }
        if (aircraft.sda) {
            ac_json["nac_p"] = aircraft.nac_p.Value();
        }
        if (aircraft.sda) {
            ac_json["nac_v"] = aircraft.nac_v.Value();
        }
        if (aircraft.sda) {
            ac_json["nic_baro"] = aircraft.nic_baro.Value();
        }
        if (aircraft.sil_supplement) {
            ac_json["sil_type"] = aircraft.sil_supplement.Value();
        }
        if (aircraft.gva) {
            ac_json["gva"] = aircraft.gva.Value();
        }
        if (aircraft.selected_altitude) {
            ac_json["nav_altitude"] = aircraft.selected_altitude.Value();
        }
        if (aircraft.barometric_pressure_setting) {
            ac_json["nav_qnh"] = aircraft.barometric_pressure_setting.Value();
        }
        if (aircraft.selected_heading) {
            ac_json["nav_heading"] = aircraft.selected_heading.Value();
        }
        if (aircraft.mode_indicators) {
            auto &modes = aircraft.mode_indicators.Value();
            auto &modes_json = ac_json["nav_modes"] = json::array();
            if (modes.autopilot) {
                modes_json.emplace_back("autopliot");
            }
            if (modes.vnav) {
                modes_json.emplace_back("vnav");
            }
            if (modes.altitude_hold) {
                modes_json.emplace_back("althold");
            }
            if (modes.approach) {
                modes_json.emplace_back("approach");
            }
            if (modes.lnav) {
                modes_json.emplace_back("lnav");
            }
        }
        // FIXME: tcas
        if (aircraft.horizontal_containment) {
            ac_json["rc"] = aircraft.horizontal_containment.Value();
        }

        ac_json["messages"] = 10;
        ac_json["seen"] = (now - aircraft.last_message_time) / 1000.0;
        ac_json["rssi"] = 0;
    }

    std::ofstream aircraft_file((dir_ / "aircraft.json").native());
    aircraft_file << std::setw(4) << aircraft_json << std::endl;
    aircraft_file.close();

    std::cerr << "wrote " << tracker_->Aircraft().size() << " entries" << std::endl;

    auto self(shared_from_this());
    timer_.expires_from_now(interval_);
    timer_.async_wait(strand_.wrap([this, self](const boost::system::error_code &ec) {
        if (!ec) {
            PeriodicWrite();
        }
    }));
}
