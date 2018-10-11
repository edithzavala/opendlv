/**
 * Copyright (C) 2017 Chalmers Revere
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
 * USA.
 */

#include <chrono>
#include <ctype.h>
#include <cstring>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <math.h>
#include <sys/time.h>

#include "opendavinci/odcore/base/KeyValueConfiguration.h"
#include "opendlv/data/environment/WGS84Coordinate.h"
#include "opendlv/data/environment/Point3.h"
#include "opendlv/data/environment/EgoState.h"

#include "opendavinci/odcore/data/Container.h"
#include "opendavinci/odcore/data/TimeStamp.h"
#include "opendavinci/odcore/strings/StringToolbox.h"

#include "V2vRequest.h"
#include "Voice.h"
#include "buffer.hpp"
#include "v2vcam.hpp"

namespace opendlv {
namespace logic {
namespace knowledge {

/**
 * Constructor.
 *
 * @param a_argc Number of command line arguments.
 * @param a_argv Command line arguments.
 */
V2vCam::V2vCam(const int32_t &a_argc, char **a_argv) :
    TimeTriggeredConferenceClientModule(a_argc, a_argv, "knowledge-v2vcam"), m_sendLog(), m_receiveLog(), m_timeType2004(), m_printInbound(), m_printOutbound(), m_record(), m_debug(
        false), m_receivedGeolocation(false), m_messageId(2), m_stationId(), m_generationDeltaTime(
        0), m_containerMask(0), m_stationType(0), m_latitude(90.0000001), m_longitude(
        180.0000001), m_semiMajorConfidence(-1), m_semiMinorConfidence(-1), m_semiMajorOrientation(
        -1), m_altitude(8000.01), m_heading(-1), m_headingConfidence(-1), m_speed(
        -1), m_speedConfidence(-1), m_vehicleLength(-1), m_vehicleWidth(-1), m_longitudinalAcc(
        -1), m_longitudinalAccConf(-1), m_yawRateValue(0), m_yawRateConfidence(
        -1), m_vehicleRole(0), m_simulation(false), m_active(false)

{
}

V2vCam::~V2vCam() {
}

void V2vCam::setUp() {
  std::cout << "V2V cam started" << std::endl;
  odcore::base::KeyValueConfiguration kv = getKeyValueConfiguration();

  m_printOutbound = kv.getValue<bool>("knowledge-v2vcam.printOutbound");
  m_printInbound = kv.getValue<bool>("knowledge-v2vcam.printInbound");
  m_record = kv.getValue<bool>("knowledge-v2vcam.record");
  m_debug = kv.getValue<bool>("knowledge-v2vcam.debug");
  m_simulation = kv.getValue<bool>("knowledge-v2vcam.simulation");

  std::cout << "Print outbound: " << m_printOutbound << " Print inbound: "
      << m_printInbound << " Record: " << m_record << " Debug: " << m_debug
      << " Simulation: " << m_simulation << std::endl;

  m_stationId = getIdentifier();
}

void V2vCam::tearDown() {
  m_sendLog.close();
  m_receiveLog.close();
}

odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode V2vCam::body() {

  while (getModuleStateAndWaitForRemainingTimeInTimeslice()
      == odcore::data::dmcp::ModuleStateMessage::RUNNING) {

    if (m_active) {
      std::shared_ptr<Buffer> outBuffer(new Buffer());
      if (m_simulation) {
        if (m_receivedGeolocation) {
          // Reverser for big and little endian specification of V2V.
          outBuffer->Reversed();
          outBuffer->AppendInteger(GetStationId()); //stationId
          outBuffer->AppendInteger(GenerateGenerationDeltaTime()); //generationDeltaTime
          outBuffer->AppendInteger(GetLatitude()); //latitude
          outBuffer->AppendInteger(GetLongitude()); //longitude
          if (m_printOutbound) {
            std::cout << "Cam object -" << " Message Id: "
                << std::to_string(GetMessageId()) << " Station Id: "
                << std::to_string(GetStationId()) << " Generation delta time: "
                << std::to_string(GenerateGenerationDeltaTime())
                << " Latitude: " << std::to_string(GetLatitude())
                << " Longitude: " << std::to_string(GetLongitude())
                << std::endl;
          }
        }
      } else {
        if (m_printOutbound) {
//          std::cout << "Cam object -" << " Message Id: "
//              << std::to_string(GetMessageId()) << " Frontal distance: -1"
//              << " Traffic factor: 3" << std::endl;
        }
      }

      outBuffer->AppendByte(GetMessageId()); //messageId
      std::vector<unsigned char> bytes = outBuffer->GetBytes();
      std::string bytesString(bytes.begin(), bytes.end());
      Voice nextMessage("cam", bytesString.size(), bytesString);
      odcore::data::Container c(nextMessage);
      timeval curTime;
      gettimeofday(&curTime, NULL);
      int milli = curTime.tv_usec / 1000;
      char b[80];
      strftime(b, 80, "%Y-%m-%d %H:%M:%S", localtime(&curTime.tv_sec)); //change to localtime_r
      char currentTime[84] = "";
      sprintf(currentTime, "%s:%d", b, milli);
      std::cout << currentTime << " Send position and traffic info"
          << std::endl;
//      printf("%s Send position and traffic info", currentTime);
//      std::cout << "Send position and traffic info" << std::endl;
      getConference().send(c);
    }
  }
  return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
}

/**
 * Receives geolocation-, voice containers.
 * Sends .
 */
void V2vCam::nextContainer(odcore::data::Container &a_c) {
//  std::cout << "Receive message of type " << a_c.getDataType() << " from "
//          << a_c.getSenderStamp() << std::endl;
  if (a_c.getDataType() == opendlv::data::environment::EgoState::ID()) {
    opendlv::data::environment::EgoState ego = a_c.getData<
        opendlv::data::environment::EgoState>();
    if (a_c.getSenderStamp() == getIdentifier()) {
      ReadEgoState(ego);
    }
  } else if (a_c.getDataType() == (opendlv::knowledge::Insight::ID() + 400)) {
    opendlv::knowledge::Insight insight = a_c.getData<
        opendlv::knowledge::Insight>();
    ReadInsight(insight);
  } else if (a_c.getDataType() == opendlv::sensation::Geolocation::ID()) {
    opendlv::sensation::Geolocation geolocation = a_c.getData<
        opendlv::sensation::Geolocation>();
    ReadGeolocation(geolocation);
  } else if (a_c.getDataType() == opendlv::model::DynamicState::ID()) {
    opendlv::model::DynamicState dynamicState = a_c.getData<
        opendlv::model::DynamicState>();
    ReadDynamicState(dynamicState);
  } else if (a_c.getDataType() == V2vRequest::ID()) {
    if (getIdentifier() == 0) {
      timeval curTime;
      gettimeofday(&curTime, NULL);
      int milli = curTime.tv_usec / 1000;
      char b[80];
      strftime(b, 80, "%Y-%m-%d %H:%M:%S", localtime(&curTime.tv_sec)); //change to localtime_r
      char currentTime[84] = "";
      sprintf(currentTime, "%s:%d", b, milli);
      std::cout << currentTime << " Request data/stop to other vehicles"
          << std::endl;
//      printf("%s Request data/stop to other vehicles", currentTime);
//      std::cout << "Request data/stop to other vehicles" << std::endl;
    } else {
      V2vRequest vr = a_c.getData<V2vRequest>();
      ReadV2vRequest(vr);
    }
  }
}

void V2vCam::ReadEgoState(const opendlv::data::environment::EgoState &ego) {
  m_latitude = ego.getPosition().getY();
  m_longitude = ego.getPosition().getX();
  m_receivedGeolocation = true;
  if (m_debug) {
//    std::cout << "Receive egostate" << std::endl;
  }
}

void V2vCam::ReadV2vRequest(const V2vRequest &request) {
  bool active;
  istringstream(request.getData()) >> active;
  m_active = active;
  if (m_debug) {
    timeval curTime;
    gettimeofday(&curTime, NULL);
    int milli = curTime.tv_usec / 1000;
    char b[80];
    strftime(b, 80, "%Y-%m-%d %H:%M:%S", localtime(&curTime.tv_sec)); //change to localtime_r
    char currentTime[84] = "";
    sprintf(currentTime, "%s:%d", b, milli);
    if (active) {
      std::cout << currentTime << " Receive request to send cam message"
          << std::endl;
//    printf("%s Receive request to send cam message", currentTime);
    } else {
      std::cout << currentTime << " Receive request to stop cam message"
          << std::endl;
//      printf("%s Receive request to stop cam message", currentTime);
    }
//    std::cout << "Receive request to send cam message?:" << b << std::endl;
  }
}

void V2vCam::ReadInsight(const opendlv::knowledge::Insight &a_insight) {
  std::string str = a_insight.getInsight();
  std::vector<std::string> information = odcore::strings::StringToolbox::split(
      str, '=');
  if (information.size() > 0) {
    if (information[0] == "stationId") {
      m_stationId = std::stoi(information[1]);
    } else if (information[0] == "stationType") {
      m_stationType = std::stoi(information[1]);
    } else if (information[0] == "vehicleLength") {
      m_vehicleLength = std::stod(information[1]);
    } else if (information[0] == "vehicleWidth") {
      m_vehicleWidth = std::stod(information[1]);
    } else if (information[0] == "vehicleRole") {
      m_vehicleRole = std::stoi(information[1]);
    }
  }

  if (m_debug) {
    std::cout << "Insight received - '" << str << "'" << std::endl;
  }
}

void V2vCam::ReadDynamicState(
    const opendlv::model::DynamicState &a_dynamicState) {
  int16_t frameId = a_dynamicState.getFrameId();
  if (frameId == 0) {
    m_speed = a_dynamicState.getVelocity().getX();
    m_speedConfidence = a_dynamicState.getVelocityConfidence().getX();
    m_yawRateValue = a_dynamicState.getAngularVelocity().getZ();
    m_yawRateConfidence = a_dynamicState.getAngularVelocityConfidence().getZ();
    m_longitudinalAcc = a_dynamicState.getAcceleration().getX();
    m_longitudinalAccConf = a_dynamicState.getAccelerationConfidence().getX();

    if (m_debug) {
      std::cout << "DynamicState received (frame 0) - speed=" << m_speed
          << " yawRatevalue=" << m_yawRateValue << " longitudinalAcceleration="
          << m_longitudinalAcc << std::endl;
    }
  }
}

void V2vCam::ReadGeolocation(
    const opendlv::sensation::Geolocation &a_geolocation) {
  m_latitude = a_geolocation.getLatitude();
  m_longitude = a_geolocation.getLongitude();
  m_altitude = a_geolocation.getAltitude();
  m_heading = a_geolocation.getHeading();
  while (m_heading < 0) {
    m_heading += 2.0 * M_PI;
  }
  while (m_heading > 2 * M_PI) {
    m_heading -= 2.0 * M_PI;
  }
  m_headingConfidence = a_geolocation.getHeadingConfidence();
  m_semiMajorConfidence = a_geolocation.getLatitudeConfidence();
  m_semiMinorConfidence = a_geolocation.getLongitudeConfidence();

  m_receivedGeolocation = true;

  if (m_debug) {
    std::cout << "Geolocation received - longitude=" << m_longitude
        << " latitude=" << m_latitude << " altitude=" << m_altitude
        << " heading=" << m_heading << std::endl;
  }
}

void V2vCam::ReadVoice(const Voice &a_voice) {
  std::cout << a_voice.getType() << std::endl;
}

void V2vCam::SendWGS84Coordinate() {
  opendlv::data::environment::WGS84Coordinate coordPacket(m_latitude,
      m_longitude);
  odcore::data::Container nextC(coordPacket);
  getConference().send(nextC);
}

unsigned char V2vCam::GetMessageId() const {
  return m_messageId;
}

int32_t V2vCam::GetStationId() const {
  return m_stationId;
}

uint64_t V2vCam::GenerateGenerationTime() const {
  std::chrono::system_clock::time_point start2004TimePoint =
      std::chrono::system_clock::from_time_t(m_timeType2004);
  uint64_t millisecondsSince2004Epoch =
      std::chrono::system_clock::now().time_since_epoch()
          / std::chrono::milliseconds(1)
          - start2004TimePoint.time_since_epoch()
              / std::chrono::milliseconds(1);
  return millisecondsSince2004Epoch;
}

int32_t V2vCam::GenerateGenerationDeltaTime() {
  m_generationDeltaTime = GenerateGenerationTime() % 65536;
  return m_generationDeltaTime;
}

unsigned char V2vCam::V2vCam::GetContainerMask() const {
  return m_containerMask;
}

int32_t V2vCam::GetStationType() const {
  return m_stationType;
}

int32_t V2vCam::GetLatitude() const {
  if (m_simulation) {
    return m_latitude;
  } else {
    int32_t scale = std::pow(10, 7);
    double val = m_latitude * scale;
    if (val < -900000000 || val > 900000000) {
      return 900000001;
    } else {
      return static_cast<int32_t>(std::round(val));
    }
  }
}

int32_t V2vCam::GetLongitude() const {
  if (m_simulation) {
    return m_longitude;
  } else {
    int32_t scale = std::pow(10, 7);
    double val = m_longitude * scale;
    if (val < -1800000000 || val > 1800000000) {
      return 1800000001;
    } else {
      return static_cast<int32_t>(std::round(val));
    }
  }
}

int32_t V2vCam::GetSemiMajorConfidence() const {
  int32_t scale = std::pow(10, 2);
  double val = m_semiMajorConfidence * scale;
  if (val < 0) {
    return 4095;
  } else if (val < 1) {
    return 1;
  } else if (val > 4093) {
    return 4094;
  } else {
    return static_cast<int32_t>(std::round(val));
  }
}

int32_t V2vCam::GetSemiMinorConfidence() const {
  int32_t scale = std::pow(10, 2);
  double val = m_semiMinorConfidence * scale;
  if (val < 0) {
    return 4095;
  } else if (val < 1) {
    return 1;
  } else if (val > 4093) {
    return 4094;
  } else {
    return static_cast<int32_t>(std::round(val));
  }
}

int32_t V2vCam::GetSemiMajorOrientation() const {
  double conversion = opendlv::Constants::RAD2DEG;
  int32_t scale = std::pow(10, 2);
  double val = m_semiMajorOrientation * scale * conversion;
  if (val < 0 || val > 3600) {
    return 3601;
  } else {
    return static_cast<int32_t>(std::round(val));
  }
}

int32_t V2vCam::GetAltitude() const {
  int32_t scale = std::pow(10, 2);
  double val = m_altitude * scale;
  if (val < -100000 || val > 800000) {
    return 800001;
  } else {
    return static_cast<int32_t>(std::round(val));
  }
}

int32_t V2vCam::GetHeading() const {
  double conversion = opendlv::Constants::RAD2DEG;
  int32_t scale = std::pow(10, 1);
  double val = m_heading * scale * conversion;
  if (val < 0 || val > 3600 || std::isnan(val)) {
    return 3601;
  } else {
    return static_cast<int32_t>(std::round(val));
  }
}

int32_t V2vCam::GetHeadingConfidence() const {
  double conversion = opendlv::Constants::RAD2DEG;
  int32_t scale = std::pow(10, 1);
  double val = m_headingConfidence * scale * conversion;
// std::cout << val << std::endl;
  if (val < 0) {
    return 127;
  } else if (val < 1) {
    return 1;
  } else if (val > 125) {
    return 126;
  } else {
    return static_cast<int32_t>(std::round(val));

  }
}

int32_t V2vCam::GetSpeed() const {
  int32_t scale = std::pow(10, 2);
  double val = m_speed * scale;
  if (val < 0) {
    return 16383;
  } else if (val > 16382) {
    return 16382;
  } else {
    return static_cast<int32_t>(std::round(val));
  }
}

int32_t V2vCam::GetSpeedConfidence() const {
// std::cout << m_speedConfidence << std::endl;
  int32_t scale = std::pow(10, 2);
  double val = m_speedConfidence * scale;
  if (val < 0) {
    return 127;
  } else if (val < 1) {
    return 1;
  } else if (val > 125) {
    return 126;
  } else {
    return static_cast<int32_t>(std::round(val));
  }
}

int32_t V2vCam::GetVehicleLength() const {
  int32_t scale = std::pow(10, 1);
  double val = m_vehicleLength * scale;
  if (val < 0) {
    return 1023;
  } else if (val > 1022) {
    return 1022;
  } else {
    return static_cast<int32_t>(std::round(val));
  }
}

int32_t V2vCam::GetVehicleWidth() const {
  int32_t scale = std::pow(10, 1);
  double val = m_vehicleWidth * scale;
  if (val < 0) {
    return 62;
  } else if (val > 61) {
    return 61;
  } else {
    return static_cast<int32_t>(std::round(val));
  }

}

int32_t V2vCam::GetLongitudinalAcc() const {
  int32_t scale = std::pow(10, 1);
  double val = m_longitudinalAcc * scale;
// std::cout << val << std::endl;
  if (m_longitudinalAccConf < 0) {
    return 161;
  } else if (val < -160) {
    return -160;
  } else if (val > 160) {
    return 160;
  } else {
    return static_cast<int32_t>(std::round(val));
  }
}

int32_t V2vCam::GetLongitudinalAccConf() const {
  int32_t scale = std::pow(10, 1);
  double val = m_longitudinalAccConf * scale;

  if (val < 0) {
    return 102;
  } else if (val < 1) {
    return 1;
  } else if (val > 100) {
    return 101;
  } else {
    return static_cast<int32_t>(std::round(val));
  }
}

int32_t V2vCam::GetYawRateValue() const {
  if (m_yawRateConfidence < 0) {
    return 32767;
  }
  int32_t scale = std::pow(10, 2);
  double conversion = opendlv::Constants::RAD2DEG;
  double val = m_yawRateValue * scale * conversion;
  if (val < -32766) {
    return -32766;
  } else if (val > 32766) {
    return 32766;
  } else {
    return static_cast<int32_t>(std::round(val));
  }
}

int32_t V2vCam::GetYawRateConfidence() const {
  double conversion = opendlv::Constants::RAD2DEG;
  double val = m_yawRateConfidence * conversion;
  if (m_yawRateConfidence < 0) {
    return 8;
  } else if (val <= 0.01) {
    return 0;
  } else if (val <= 0.05) {
    return 1;
  } else if (val <= 0.1) {
    return 2;
  } else if (val <= 1) {
    return 3;
  } else if (val <= 5) {
    return 4;
  } else if (val <= 10) {
    return 5;
  } else if (val <= 100) {
    return 6;
  } else {
    return 7;
  }
}

int32_t V2vCam::GetVehicleRole() const {
  return m_vehicleRole;
}

}
}
}
