/**
 * Copyright (C) 2018 Chalmers Revere, UPC
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

#include <ctype.h>
#include <cstring>
#include <iostream>
#include <stdio.h>
#include <math.h>

#include <sys/socket.h>
#include <sys/types.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <netdb.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <arpa/inet.h>
#include <fstream>

#include "opendavinci/odcore/wrapper/SharedMemoryFactory.h"
#include "opendavinci/odcore/wrapper/SharedMemory.h"
#include "opendavinci/odcore/base/KeyValueConfiguration.h"
#include "odvdopendlvdata/GeneratedHeaders_ODVDOpenDLVData.h"
#include "opendavinci/odcore/data/TimeStamp.h"
#include "automotivedata/GeneratedHeaders_AutomotiveData.h"
#include "opendavinci/GeneratedHeaders_OpenDaVINCI.h"
#include "opendlv/data/environment/WGS84Coordinate.h"

#include "V2vRequest.h"
#include "../include/ksamclient.hpp"
#include "Voice.h"
#include "buffer.hpp"
#include "MonitorAdaptation.h"

#define PI 3.14159265

namespace opendlv {
namespace logic {
namespace adaptation {

/**
 * Constructor.
 *
 * @param a_argc Number of command line arguments.
 * @param a_argv Command line arguments.
 */
KsamClient::KsamClient(const int32_t &a_argc, char **a_argv) :
    DataTriggeredConferenceClientModule(a_argc, a_argv,
        "adaptation-ksamclient"), m_initialized(false), m_simulation(false), m_v2vcamRequest(
        false), m_forwardData(false), m_laneFollowerIsActive(false), m_v2vcamIsActive(
        false), m_v2vdenmIsActive(false), m_cameraIsActive(false), m_gpsIsActive(
        false), m_lidarIsActive(false), m_canIsActive(false), m_cameraIsFaulty(
        false), m_gpsIsFaulty(false), m_lidarIsFaulty(false), m_canIsFaulty(
        false), m_routeId(1), m_faultyI(0), m_currentFaultyI(0)
//    , m_mtx()
//    , m_debug()
{
}

KsamClient::~KsamClient() {
}
void KsamClient::setUp() {
  std::cout << "Ksam client started" << std::endl;

  odcore::base::KeyValueConfiguration kv = getKeyValueConfiguration();
  m_initialized = true;
  m_simulation = kv.getValue<bool>("adaptation-ksamclient.simulation");
  m_forwardData = kv.getValue<bool>("adaptation-ksamclient.forwardData");

//  m_laneFollowerIsActive = kv.getValue<bool>("global.lanefollower.active");
  m_v2vcamIsActive = kv.getValue<bool>("global.v2vcam.active");
  m_v2vdenmIsActive = kv.getValue<bool>("global.v2vdenm.active");

  if (!m_simulation) {
    m_cameraIsActive = kv.getValue<bool>("global.camera-axis.active");
    m_gpsIsActive = kv.getValue<bool>("global.applanix.active");
    m_lidarIsActive = kv.getValue<bool>("global.velodyne32.active");
    m_canIsActive = kv.getValue<bool>("global.xc90.active");

    m_cameraIsFaulty = kv.getValue<bool>("global.camera-axis.faulty");
    m_gpsIsFaulty = kv.getValue<bool>("global.applanix.faulty");
    m_lidarIsFaulty = kv.getValue<bool>("global.velodyne32.faulty");
    m_canIsFaulty = kv.getValue<bool>("global.xc90.faulty");
    m_faultyI = kv.getValue<int32_t>("global.iterations.faulty");
  }

//  m_debug = (kv.getValue<int32_t>("logic-adaptation-ksam.debug") == 1);
}

void KsamClient::tearDown() {
}

void KsamClient::nextContainer(odcore::data::Container &a_c) {
//  std::cout << "Type " << a_c.getDataType() << std::endl;

  if (a_c.getDataType() == Voice::ID()) {
    processV2VData(a_c);
  } else if (!m_simulation) {
    if (a_c.getDataType() == MonitorAdaptation::ID()) {
      MonitorAdaptation ma = a_c.getData<MonitorAdaptation>();
      processAdaptation(ma);
    } else {
      processRealData(a_c);
    }
  } else {
    processSimulationData(a_c);
  }
}

void KsamClient::processAdaptation(MonitorAdaptation &a_ma) {
  timeval curTime;
  gettimeofday(&curTime, NULL);
  int milli = curTime.tv_usec / 1000;
  char b[80];
  strftime(b, 80, "%Y-%m-%d %H:%M:%S", localtime(&curTime.tv_sec)); //change to localtime_r
  char currentTime[84] = "";
  sprintf(currentTime, "%s:%d", b, milli);
  if (a_ma.getMonitorName().compare("axiscamera") == 0) {
    if (a_ma.getAction().compare("add") == 0) {
//      printf("%s Axis camera added", currentTime);
      std::cout << std::to_string(currentTime) << " Axis camera added"
          << std::endl;
      m_cameraIsActive = true;
    } else if (a_ma.getAction().compare("remove") == 0) {
//      printf("%s Axis camera removed", currentTime);
      std::cout << std::to_string(currentTime) << " Axis camera removed"
          << std::endl;
      m_cameraIsActive = false;
    }
  } else if (a_ma.getMonitorName().compare("applanixGps") == 0) {
    if (a_ma.getAction().compare("add") == 0) {
//      printf("%s Applanix gps added", currentTime);
      std::cout << std::to_string(currentTime) << " Applanix gps added"
          << std::endl;
      m_gpsIsActive = true;
    } else if (a_ma.getAction().compare("remove") == 0) {
      m_gpsIsActive = false;
//      printf("%s Applanix gps removed", currentTime);
      std::cout << std::to_string(currentTime) << " Applanix gps removed"
          << std::endl;
    }
  } else if (a_ma.getMonitorName().compare("velodyne32Lidar") == 0) {
    if (a_ma.getAction().compare("add") == 0) {
//      printf("%s Velodyne32 lidar added", currentTime);
      std::cout << std::to_string(currentTime) << " Velodyne32 lidar added"
          << std::endl;
      m_lidarIsActive = true;
    } else if (a_ma.getAction().compare("remove") == 0) {
//      printf("%s Velodyne32 lidar removed", currentTime);
      std::cout << std::to_string(currentTime) << " Velodyne32 lidar removed"
          << std::endl;
      m_lidarIsActive = false;
    }
  } else if (a_ma.getMonitorName().compare("can") == 0) {
    if (a_ma.getAction().compare("add") == 0) {
//      printf("%s CAN added", currentTime);
      std::cout << std::to_string(currentTime) << " CAN added" << std::endl;
      m_canIsActive = true;
    } else if (a_ma.getAction().compare("remove") == 0) {
//      printf("%s CAN removed", currentTime);
      std::cout << std::to_string(currentTime) << " CAN removed" << std::endl;
      m_canIsActive = false;
    }
  }
}

void KsamClient::processV2VData(odcore::data::Container &a_c) {
  Voice voice = a_c.getData<Voice>();
  int32_t trafficF;
  if (m_routeId == 1) {
    trafficF = 6;
  } else {
    trafficF = -2;
  }
  if (!m_simulation) {
    //m_v2vcamIsActive, m_v2vdenmIsActive not revised. If a message is received we assume they has been activated.
    if (voice.getType() == "cam") { //all cam messages are sent by the Truck
      std::string data(
          "{'systemId' : 'openDlvMonitorv0','timeStamp':'"
              + std::to_string(a_c.getSampleTimeStamp().toMicroseconds()) + "',"
              + "'context': [{'services': [");
      if (m_laneFollowerIsActive) {
        data += "'laneFollower'";
      }
      data +=
          "]}],'monitors': [{'monitorId':'V2VCam_FrontCenter','measurements': [{'varId':'frontaldistance','measures': [{'mTimeStamp': '"
              + std::to_string(a_c.getSampleTimeStamp().toMicroseconds())
              + "','value':'" + std::to_string(-1)
              + "'}]},{'varId':'trafficFactor','measures': [{'mTimeStamp': '"
              + std::to_string(a_c.getSampleTimeStamp().toMicroseconds())
              + "','value':'" + std::to_string(trafficF) + "'}]}]}]}";
      forwardDataToKsam(data);
    } else if (voice.getType() == "denm") { //all denm messages are sent by the Truck
      std::string data(
          "{'systemId' : 'openDlvMonitorv0','timeStamp':'"
              + std::to_string(a_c.getSampleTimeStamp().toMicroseconds()) + "',"
              + "'context': [{'services': [");
      if (m_laneFollowerIsActive) {
        data += "'laneFollower'";
      }
      data +=
          "]}],'monitors': [{'monitorId':'V2VDenm_Event','measurements': [{'varId':'Event','measures': [{'mTimeStamp': '"
              + std::to_string(a_c.getSampleTimeStamp().toMicroseconds())
              + "','value':'CRASH'}]}]}]}";
      forwardDataToKsam(data);
    }
  } else {
    //  && a_c.getSenderStamp() == 1) {
    if (voice.getType() == "cam") { //all cam messages are sent by the 2nd simvehicle
      m_v2vcamIsActive = true;
    } else if (voice.getType() == "denm") { //all denm messages are sent by the 2nd simvehicle
      std::string data(
          "{'systemId' : 'openDlvMonitorv0','timeStamp':'"
              + std::to_string(a_c.getSampleTimeStamp().toMicroseconds()) + "',"
              + "'context': [{'services': [");
      if (m_laneFollowerIsActive) {
        data += "'laneFollower'";
      }
      data +=
          "]}],'monitors': [{'monitorId':'V2VDenm_Event','measurements': [{'varId':'Event','measures': [{'mTimeStamp': '"
              + std::to_string(a_c.getSampleTimeStamp().toMicroseconds())
              + "','value':'CRASH'}]}]}]}";
      forwardDataToKsam(data);
    }
  }
}

void KsamClient::forwardDataToKsam(std::string &a_data) {
//  std::cout << a_data << std::endl;
//  **********************************************
  if (m_forwardData) {
    int socket_client = socket(AF_INET, SOCK_STREAM, 0);

    struct sockaddr_in server;
    server.sin_family = AF_INET;
    server.sin_port = htons(8083);
    server.sin_addr.s_addr = inet_addr("127.0.0.1");

    if (socket_client < 0) {
      std::cout << "Socket could not be created" << std::endl;
    }

    if (connect(socket_client, (struct sockaddr *) &server, sizeof(server))
        < 0) {
      std::cout << "Connection failed due to port and ip problems" << std::endl;
    }

    //        std::cout << a_data << std::endl;
    if (write(socket_client, a_data.c_str(), strlen(a_data.c_str())) < 0) {
      std::cout << "Data send failed" << std::endl;
    }

    close(socket_client);
  }
//*************************************************
}

void KsamClient::processRealData(odcore::data::Container &a_c) {
  if (a_c.getDataType() == opendlv::proxy::ImageReading::ID()) {
    processCameraData(a_c);
  } else if (a_c.getDataType() == opendlv::proxy::GroundSpeedReading::ID()) {
    processCanData(a_c);
  } else if (a_c.getDataType() == opendlv::proxy::PointCloudReading::ID()) {
    processLidarData(a_c);
  } else if (a_c.getDataType()
      == opendlv::data::environment::WGS84Coordinate::ID()
      || a_c.getDataType() == opendlv::device::gps::pos::Grp1Data::ID()) {
    processGpsData(a_c);
  } else if (a_c.getDataType() == automotive::VehicleControl::ID()) {
    processVehicleControlData(a_c);
  }
}

void KsamClient::processCameraData(odcore::data::Container &a_c) {
  if (m_cameraIsActive) {
    opendlv::proxy::ImageReading is =
        a_c.getData<opendlv::proxy::ImageReading>();
    double imgSize;
    double frontaldistance;
    if (m_cameraIsFaulty && m_currentFaultyI >= m_faultyI) {
      imgSize = -1;
      frontaldistance = -2;
    } else {
      imgSize = is.getWidth() * is.getHeight();
      frontaldistance = -1;
      if (m_cameraIsFaulty) {
        m_currentFaultyI++;
      }
    }

    std::string data(
        "{'systemId' : 'openDlvMonitorv0','timeStamp':'"
            + std::to_string(a_c.getSampleTimeStamp().toMicroseconds()) + "',"
            + "'context': [{'services': [");
    if (m_laneFollowerIsActive) {
      data += "'laneFollower'";
    }
    data +=
        "]}],'monitors': [{'monitorId':'axiscamera','measurements': [{'varId':'frontaldistance','measures': [{'mTimeStamp': '"
            + std::to_string(a_c.getSampleTimeStamp().toMicroseconds())
            + "','value':'" + std::to_string(frontaldistance)
            + "'}]},{'varId':'imgSize','measures': [{'mTimeStamp': '"
            + std::to_string(a_c.getSampleTimeStamp().toMicroseconds())
            + "','value':'" + std::to_string(imgSize) + "'}]}]}]}";
//        std::cout << "Send axiscamera data" << std::endl;
    forwardDataToKsam(data);
  } else {
//    std::cout << "Axis camera off" << std::endl;
  }
}
void KsamClient::processGpsData(odcore::data::Container &a_c) {
  if (m_gpsIsActive) {
//    if (a_c.getDataType()
//        == opendlv::data::environment::WGS84Coordinate::ID()) {
    opendlv::data::environment::WGS84Coordinate gps = a_c.getData<
        opendlv::data::environment::WGS84Coordinate>();
    double lat;
    double lon;
    if (m_gpsIsFaulty && (m_currentFaultyI >= m_faultyI)) {
      lat = -1;
      lon = -1;
    } else {
      lat = gps.getLatitude();
      lon = gps.getLongitude();
      if (m_gpsIsFaulty) {
        m_currentFaultyI++;
      }
    }

    std::string data(
        "{'systemId' : 'openDlvMonitorv0','timeStamp':'"
            + std::to_string(a_c.getSampleTimeStamp().toMicroseconds()) + "',"
            + "'context': [{'services': [");
    if (m_laneFollowerIsActive) {
      data += "'laneFollower'";
    }
    data +=
        "]}],'monitors': [{'monitorId':'applanixGps','measurements': [{'varId':'latitude','measures': [{'mTimeStamp': '"
            + std::to_string(a_c.getSampleTimeStamp().toMicroseconds())
            + "','value':'" + std::to_string(lat)
            + "'}]},{'varId':'longitude','measures': [{'mTimeStamp': '"
            + std::to_string(a_c.getSampleTimeStamp().toMicroseconds())
            + "','value':'" + std::to_string(lon) + "'}]}]}]}";
    forwardDataToKsam(data);
//      std::cout << "Send applanixGps lat/lon data" << std::endl;
//    } else {
//      opendlv::device::gps::pos::Grp1Data grp1 = a_c.getData<
//          opendlv::device::gps::pos::Grp1Data>();
//      float speed = grp1.getSpeed() * 3600 / 1000; //speed is translated into km/h (speed*3600/1000)
//      std::string data(
//          "{'systemId' : 'openDlvMonitorv0','timeStamp':'"
//              + std::to_string(a_c.getSampleTimeStamp().toMicroseconds()) + "',"
//              + "'context': [{'services': [");
//      if (m_laneFollowerIsActive) {
//        data += "'laneFollower'";
//      }
//      data +=
//          "]}],'monitors': [{'monitorId':'applanixGps','measurements': [{'varId':'speed','measures': [{'mTimeStamp': '"
//              + std::to_string(a_c.getSampleTimeStamp().toMicroseconds())
//              + "','value':'" + std::to_string(speed) + "'}]}]}]}";
//      forwardDataToKsam(data);
//      std::cout << "Send applanixGps speed data" << std::endl;
//    }
  } else {
//    std::cout << "Applanix gps off" << std::endl;
  }
}
void KsamClient::processLidarData(odcore::data::Container &a_c) {
  if (m_lidarIsActive) {
    opendlv::proxy::PointCloudReading pc = a_c.getData<
        opendlv::proxy::PointCloudReading>();

    int8_t numberOfLayersInMessage = pc.getEntriesPerAzimuth();

    if (numberOfLayersInMessage == 9) {
      std::string distances;
      string distancesAll;
      double endAzimuth;
      double startAzimuth;
      //      int8_t numberOfBitsForIntensity = pc.getNumberOfBitsForIntensity(); // 0
      double frontalDistance;
      double rearDistance;
      double rightDistance;
      double leftDistance;

      if (m_lidarIsFaulty && (m_currentFaultyI >= m_faultyI)) {
        endAzimuth = -1;
        startAzimuth = -1;
        frontalDistance = -2;
        rearDistance = -2;
        rightDistance = -2;
        leftDistance = -2;
      } else {
        distances = pc.getDistances();
        endAzimuth = pc.getEndAzimuth();
        startAzimuth = pc.getStartAzimuth();
        frontalDistance = 0.0;
        rearDistance = 0.0;
        rightDistance = 0.0;
        leftDistance = 0.0;
        if (m_lidarIsFaulty) {
          m_currentFaultyI++;
        }

        std::vector<unsigned char> distancesData(distances.begin(),
            distances.end());
        std::shared_ptr<const Buffer> buffer(new Buffer(distancesData));
        std::shared_ptr<Buffer::Iterator> inIterator = buffer->GetIterator();
        //Long and little endian reverser
        inIterator->ItReversed();
        int numberOfPoints = buffer->GetSize() / 2;
//        std::cout << "numberOfPoints: " << std::to_string(numberOfPoints)
//            << std::endl;

        int numberOfPointsPerLayer = numberOfPoints / numberOfLayersInMessage;
//        std::cout << "numberOfPointsPerLayer: "
//            << std::to_string(numberOfPointsPerLayer)
//            << std::endl;

        double azimuthIncrement = (endAzimuth - startAzimuth)
            / numberOfPointsPerLayer;
        double azimuth = startAzimuth;

        double approxNinetyDegrees = startAzimuth
            + ((numberOfPointsPerLayer / 4) * azimuthIncrement);
        double approxOneHundredEightyDegrees = startAzimuth
            + ((numberOfPointsPerLayer / 2) * azimuthIncrement);
        double approxTwoHundredSeventyDegrees = startAzimuth
            + ((numberOfPointsPerLayer / 4) * 3 * azimuthIncrement);

        for (int i = 0; i < numberOfPoints; i++) {
          short distanceTemp = inIterator->ReadShort();
          distancesAll += std::to_string(distanceTemp) + ";";
//          std::cout << "distanceTemp: " << numberOfPointsPerLayer << std::endl;

          /***layer 3-4 of buffer corresponds to layer 14 of 32***/
          if (i > numberOfPointsPerLayer * 3
              && i <= numberOfPointsPerLayer * 4) {
//            double verticalAngleLayer14 = -12.0;
//            double xyDistance = distanceTemp
//                    * cos(verticalAngleLayer14 * PI / 180.0);
//            double x = xyDistance * sin(azimuth * PI / 180.0);
//            double y = xyDistance * cos(azimuth * PI / 180.0);
//            double z = distanceTemp * sin(verticalAngleLayer14 * PI / 180.0);

            if (azimuth <= startAzimuth) {
              frontalDistance = distanceTemp;
//              std::cout << "Distance front: " << frontalDistance << " azimuth: "
//                  << azimuth << std::endl;
            } else if (azimuth <= (approxNinetyDegrees + azimuthIncrement)
                && azimuth >= approxNinetyDegrees) {
              rightDistance = distanceTemp;
//              std::cout << "Distance right: " << rightDistance << " azimuth: "
//                  << azimuth << std::endl;
            } else if (azimuth
                <= (approxOneHundredEightyDegrees + azimuthIncrement)
                && azimuth >= approxOneHundredEightyDegrees) {
              rearDistance = distanceTemp;
//              std::cout << "Distance rear: " << rearDistance << " azimuth: "
//                  << azimuth << std::endl;
            } else if (azimuth
                <= (approxTwoHundredSeventyDegrees + azimuthIncrement)
                && azimuth >= approxTwoHundredSeventyDegrees) {
              leftDistance = distanceTemp;
//              std::cout << "Distance left: " << leftDistance << " azimuth: "
//                  << azimuth << std::endl;
            }
            azimuth += azimuthIncrement;
          }

        }
      }

      ofstream myfile;
      myfile.open("allDistancesStringLog.txt", std::ios_base::app);
      myfile << "Sampled at "
          << std::to_string(a_c.getSampleTimeStamp().toMicroseconds())
          << " | Received at "
          << std::to_string(a_c.getReceivedTimeStamp().toMicroseconds())
          << ": all distances string: " << distancesAll << "\n";
      myfile.close();

//      std::cout << "Sampled at "
//          << std::to_string(a_c.getSampleTimeStamp().toMicroseconds())
//          << " | Received at "
//          << std::to_string(a_c.getReceivedTimeStamp().toMicroseconds())
//          << ": all distances string: " << distancesAll << std::endl;

      std::string data(
          "{'systemId' : 'openDlvMonitorv0','timeStamp':'"
              + std::to_string(a_c.getSampleTimeStamp().toMicroseconds()) + "',"
              + "'context': [{'services': [");
      if (m_laneFollowerIsActive) {
        data += "'laneFollower'";
      }

      data +=
          "]}],'monitors': [{'monitorId':'velodyne32Lidar','measurements': [{'varId':'startAzimuth','measures': [{'mTimeStamp': '"
              + std::to_string(a_c.getSampleTimeStamp().toMicroseconds())
              + "','value':'" + std::to_string(startAzimuth)
              + "'}]},{'varId':'endAzimuth','measures': [{'mTimeStamp': '"
              + std::to_string(a_c.getSampleTimeStamp().toMicroseconds())
              + "','value':'" + std::to_string(endAzimuth)
              + "'}]},{'varId':'frontaldistance','measures': [{'mTimeStamp': '"
              + std::to_string(a_c.getSampleTimeStamp().toMicroseconds())
              + "','value':'" + std::to_string(frontalDistance)
              + "'}]},{'varId':'rightdistance','measures': [{'mTimeStamp': '"
              + std::to_string(a_c.getSampleTimeStamp().toMicroseconds())
              + "','value':'" + std::to_string(rightDistance)
              + "'}]},{'varId':'leftdistance','measures': [{'mTimeStamp': '"
              + std::to_string(a_c.getSampleTimeStamp().toMicroseconds())
              + "','value':'" + std::to_string(leftDistance)
              + "'}]},{'varId':'reardistance','measures': [{'mTimeStamp': '"
              + std::to_string(a_c.getSampleTimeStamp().toMicroseconds())
              + "','value':'" + std::to_string(rearDistance) + "'}]}]}]}";

//      std::cout << "Start azimuth " << std::to_string(startAzimuth)
//          << std::endl;
//      std::cout << "End azimuth " << std::to_string(endAzimuth) << std::endl;
//
//      std::cout << "Number of points " << numberOfPoints << std::endl;
//      std::cout << "Number of points per layer " << numberOfPointsPerLayer
//          << std::endl;
//      std::cout << "Azimuth increment " << azimuthIncrement << std::endl;
//      std::cout << distances << std::endl;
//      std::cout << std::to_string(distances.length()) << std::endl; //39042
//      std::cout << std::to_string(inIterator->ReadDouble()) << " - "
//          << std::to_string(inIterator->ReadDouble()) << std::endl; //0.0,0.0
//      std::cout << std::to_string(bitsForInt) << std::endl; //0
//      std::cout << "Send velodyne32Lidar data" << std::endl;
      forwardDataToKsam(data);
    }
  } else {
//    std::cout << "Velodyne32 lidar off" << std::endl;
  }
}
void KsamClient::processCanData(odcore::data::Container &a_c) {
  if (m_canIsActive) {
    opendlv::proxy::GroundSpeedReading vd = a_c.getData<
        opendlv::proxy::GroundSpeedReading>();
    // Assume vehicle is never in reverse
    double speed;
    if (m_canIsFaulty && (m_currentFaultyI >= m_faultyI)) {
      speed = -1;
    } else {
      speed = vd.getGroundSpeed();
      if (speed < 0) {
        speed = 0;
      }
      if (m_canIsFaulty) {
        m_currentFaultyI++;
      }
    }

    std::string data(
        "{'systemId' : 'openDlvMonitorv0','timeStamp':'"
            + std::to_string(a_c.getSampleTimeStamp().toMicroseconds()) + "',"
            + "'context': [{'services': [");
    if (m_laneFollowerIsActive) {
      data += "'laneFollower'";
    }
    data +=
        "]}],'monitors': [{'monitorId':'can','measurements': [{'varId':'speed','measures': [{'mTimeStamp': '"
            + std::to_string(a_c.getSampleTimeStamp().toMicroseconds())
            + "','value':'" + std::to_string(speed) + "'}]}]}]}";
    //            std::cout << data << std::endl;
    forwardDataToKsam(data);
  } else {
//    std::cout << "CAN off" << std::endl;
  }
}

void KsamClient::processSimulationData(odcore::data::Container &a_c) {
  if (a_c.getSenderStamp() == 0) { //All no Voice messages produced by the 2nd vehicle are ignored
    if (a_c.getDataType() == automotive::VehicleControl::ID()) {
      processVehicleControlData(a_c);
    } else if (a_c.getDataType() == V2vRequest::ID()) {
      V2vRequest vr = a_c.getData<V2vRequest>();
      istringstream(vr.getData()) >> m_v2vcamRequest;
    } else if (a_c.getDataType() == odcore::data::image::SharedImage::ID()) {
      processSimCameraData(a_c);
    } else if (a_c.getDataType()
        == automotive::miniature::SensorBoardData::ID()) {
      processIrusData(a_c);
    } else if (a_c.getDataType() == automotive::VehicleData::ID()) {
      processVehiclePositionData(a_c);
    }
  }
}

void KsamClient::processIrusData(odcore::data::Container &a_c) {
  std::string data(
      "{'systemId' : 'openDlvMonitorv" + std::to_string(a_c.getSenderStamp())
          + "','timeStamp':'"
          + std::to_string(a_c.getSampleTimeStamp().toMicroseconds()) + "',"
          + "'context': [{'services': [");
  if (m_laneFollowerIsActive) {
    data += "'laneFollower'";
  }
  data += "]}],'monitors': [";

  automotive::miniature::SensorBoardData sbd = a_c.getData<
      automotive::miniature::SensorBoardData>();
  double i_frontRightDistance = sbd.getValueForKey_MapOfDistances(0);
  double i_rearDistance = sbd.getValueForKey_MapOfDistances(1);
  double i_rearRightDistance = sbd.getValueForKey_MapOfDistances(2);
  double u_frontCenterDistance = sbd.getValueForKey_MapOfDistances(3);
  double u_frontRightDistance = sbd.getValueForKey_MapOfDistances(4);
  double u_rearRightDistance = sbd.getValueForKey_MapOfDistances(5);
  data +=
      "{'monitorId':'Infrared_FrontRight','measurements': [{'varId':'FrontRightDistance','measures': [{'mTimeStamp': '"
          + std::to_string(a_c.getSampleTimeStamp().toMicroseconds())
          + "','value':'" + std::to_string(i_frontRightDistance) + "'}]}]},";
  if (m_v2vcamIsActive && m_v2vcamRequest) {
    double v2v = sbd.getValueForKey_MapOfDistances(6);
    data +=
        "{'monitorId':'V2VCam_FrontCenter','measurements': [{'varId':'FrontCenterDistance','measures': [{'mTimeStamp': '"
            + std::to_string(a_c.getSampleTimeStamp().toMicroseconds())
            + "','value':'" + std::to_string(v2v) + "'}]}]},";
  }
  data +=
      "{'monitorId':'Infrared_Rear','measurements': [{'varId':'RearDistance','measures': [{'mTimeStamp': '"
          + std::to_string(a_c.getSampleTimeStamp().toMicroseconds())
          + "','value':'" + std::to_string(i_rearDistance) + "'}]}]},"
          + "{'monitorId':'Infrared_RearRight','measurements': [{'varId':'RearRightDistance','measures': [{'mTimeStamp': '"
          + std::to_string(a_c.getSampleTimeStamp().toMicroseconds())
          + "','value':'" + std::to_string(i_rearRightDistance) + "'}]}]},"
          + "{'monitorId':'UltraSonic_FrontCenter','measurements': [{'varId':'FrontCenterDistance','measures': [{'mTimeStamp': '"
          + std::to_string(a_c.getSampleTimeStamp().toMicroseconds())
          + "','value':'" + std::to_string(u_frontCenterDistance) + "'}]}]},"
          + "{'monitorId':'UltraSonic_FrontRight','measurements': [{'varId':'FrontRightDistance','measures': [{'mTimeStamp': '"
          + std::to_string(a_c.getSampleTimeStamp().toMicroseconds())
          + "','value':'" + std::to_string(u_frontRightDistance) + "'}]}]},"
          + "{'monitorId':'UltraSonic_RearRight','measurements': [{'varId':'RearRightDistance','measures': [{'mTimeStamp': '"
          + std::to_string(a_c.getSampleTimeStamp().toMicroseconds())
          + "','value':'" + std::to_string(u_rearRightDistance) + "'}]}]}]}";
//      std::cout << data << std::endl;
  forwardDataToKsam(data);
}

void KsamClient::processSimCameraData(odcore::data::Container &a_c) {
  std::string data(
      "{'systemId' : 'openDlvMonitorv" + std::to_string(a_c.getSenderStamp())
          + "','timeStamp':'"
          + std::to_string(a_c.getSampleTimeStamp().toMicroseconds()) + "',"
          + "'context': [{'services': [");
  if (m_laneFollowerIsActive) {
    data += "'laneFollower'";
  }
  data += "]}],'monitors': [";

  odcore::data::image::SharedImage sharedImg = a_c.getData<
      odcore::data::image::SharedImage>();
  data +=
      "{'monitorId':'odsimcamera','measurements': [{'varId':'imgSize','measures': [{'mTimeStamp': '"
          + std::to_string(a_c.getSampleTimeStamp().toMicroseconds())
          + "','value':'" + std::to_string(sharedImg.getSize())
          + "'}]},{'varId':'FrontCenterDistance','measures': [{'mTimeStamp': '"
          + std::to_string(a_c.getSampleTimeStamp().toMicroseconds())
          + "','value':'" + std::to_string(-1) + +"'}]}]}]}";
//          std::cout << data << std::endl;
  forwardDataToKsam(data);
}

void KsamClient::processVehiclePositionData(odcore::data::Container &a_c) {
  std::string data(
      "{'systemId' : 'openDlvMonitorv" + std::to_string(a_c.getSenderStamp())
          + "','timeStamp':'"
          + std::to_string(a_c.getSampleTimeStamp().toMicroseconds()) + "',"
          + "'context': [{'services': [");
  if (m_laneFollowerIsActive) {
    data += "'laneFollower'";
  }
  data += "]}],'monitors': [";

  automotive::VehicleData vd = a_c.getData<automotive::VehicleData>();
  double x = vd.getPosition().getP()[0];
  double y = vd.getPosition().getP()[1];
// Assume vehicle is never in reverse
  double speed = vd.getSpeed();
  if (speed < 0) {
    speed = 0;
  }
  data +=
      "{'monitorId':'imuodsimcvehicle','measurements': [{'varId':'speed','measures': [{'mTimeStamp': '"
          + std::to_string(a_c.getSampleTimeStamp().toMicroseconds())
          + "','value':'" + std::to_string(speed)
          + "'}]},{'varId':'longitude','measures': [{'mTimeStamp': '"
          + std::to_string(a_c.getSampleTimeStamp().toMicroseconds())
          + "','value':'" + std::to_string(x)
          + "'}]},{'varId':'latitude','measures': [{'mTimeStamp': '"
          + std::to_string(a_c.getSampleTimeStamp().toMicroseconds())
          + "','value':'" + std::to_string(y) + "'}]}]}]}";
//            std::cout << data << std::endl;
  forwardDataToKsam(data);
}

void KsamClient::processVehicleControlData(odcore::data::Container &a_c) {
  automotive::VehicleControl vc = a_c.getData<automotive::VehicleControl>();
  if (vc.getSpeed() > 0) {
    m_laneFollowerIsActive = true;
    m_routeId = vc.getSpeed();
  } else if (m_simulation) {
    m_laneFollowerIsActive = false;
  } else if (m_laneFollowerIsActive) {
    m_laneFollowerIsActive = false;
  } else {
    m_laneFollowerIsActive = true;
  }
}

}
}
}
