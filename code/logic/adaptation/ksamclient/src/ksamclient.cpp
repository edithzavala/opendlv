/**
 * Copyright (C) 2017 Chalmers Revere, UPC
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

#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <netdb.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <arpa/inet.h>

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

namespace opendlv {
namespace logic {
namespace adaptation {

//using namespace std;
//using namespace odcore::base;
//using namespace odcore::base::module;
//using namespace odcore::data;
//using namespace automotive;

/**
 * Constructor.
 *
 * @param a_argc Number of command line arguments.
 * @param a_argv Command line arguments.
 */
KsamClient::KsamClient(const int32_t &a_argc, char **a_argv) :
        DataTriggeredConferenceClientModule(a_argc, a_argv,
                "adaptation-ksamclient"), m_initialized(false), m_v2vcam(false), m_v2vcamRequest(
                false), m_laneFollower(false), m_simulation(false)
//    , m_mtx()
//    , m_debug()
{
}

//Ksam::Ksam(Ksam const &a_ksam): DataTriggeredConferenceClientModule(){
//	this = a_ksam;
//}

KsamClient::~KsamClient() {
}
void KsamClient::setUp() {
  odcore::base::KeyValueConfiguration kv = getKeyValueConfiguration();
  m_simulation = kv.getValue<bool>("adaptation-ksamclient.simulation");
  m_initialized = true;
//  m_debug = (kv.getValue<int32_t>("logic-adaptation-ksam.debug") == 1);
}

void KsamClient::tearDown() {
}

void KsamClient::nextContainer(odcore::data::Container &a_c) {
  std::cout << "Type " << a_c.getDataType() << std::endl;
  bool sendMessage = false;
  std::string data;

  if (!m_simulation) {
    if (a_c.getDataType() == automotive::VehicleData::ID()) {
      /**--------------CAN DATA---------------------**/
      automotive::VehicleData vd = a_c.getData<automotive::VehicleData>();

      // Assume vehicle is never in reverse
      double speed = vd.getSpeed();
      if (speed < 0) {
        speed = 0;
      }

      data +=
              "{'systemId' : 'openDlvMonitorv0','timeStamp':'"
                      + std::to_string(
                              a_c.getReceivedTimeStamp().toMicroseconds())
                      + "',"
                      + "'context': [{'services': ['laneFollower']}],'monitors': [{'monitorId':'can','measurements': [{'varId':'speed','measures': [{'mTimeStamp': '"
                      + std::to_string(
                              a_c.getSampleTimeStamp().toMicroseconds())
                      + "','value':'" + std::to_string(speed) + "'}]}]}]}";
      //            std::cout << data << std::endl;
      sendMessage = true;
    } else if (a_c.getDataType() == opendlv::proxy::PointCloudReading::ID()) {
      /**--------------VELODYNE DATA ---------------------**/
      opendlv::proxy::PointCloudReading pc = a_c.getData<
              opendlv::proxy::PointCloudReading>();
      int8_t layers = pc.getEntriesPerAzimuth();
//      int8_t bitsForInt = pc.getNumberOfBitsForIntensity();

      if (layers == 9) {
//        std::string distances = pc.getDistances();

        std::string distances = pc.getDistances();
        std::vector<unsigned char> distancesData(distances.begin(),
                distances.end());
        std::shared_ptr<const Buffer> buffer(new Buffer(distancesData));
        std::shared_ptr < Buffer::Iterator > inIterator = buffer->GetIterator();
        //Long and little endian reverser
//        inIterator->ItReversed();

//        std::cout << distances << std::endl;
//        std::cout << std::to_string(distances.length()) << std::endl; //39042
//        std::cout << std::to_string(inIterator->ReadDouble()) << " - "
//                << std::to_string(inIterator->ReadDouble()) << std::endl; //0.0,0.0
        //std::cout << std::to_string(bitsForInt) << std::endl; //0
      data +=
              "{'systemId' : 'openDlvMonitorv0','timeStamp':'"
                      + std::to_string(
                              a_c.getReceivedTimeStamp().toMicroseconds())
                      + "',"
                      + "'context': [{'services': ['laneFollower']}],'monitors': [{'monitorId':'velodyne32Lidar','measurements': [{'varId':'startAzimuth','measures': [{'mTimeStamp': '"
                      + std::to_string(
                              a_c.getSampleTimeStamp().toMicroseconds())
                      + "','value':'" + std::to_string(pc.getStartAzimuth())
                      + "'}]},{'varId':'endAzimuth','measures': [{'mTimeStamp': '"
                      + std::to_string(
                              a_c.getSampleTimeStamp().toMicroseconds())
                      + "','value':'" + std::to_string(pc.getEndAzimuth())
                        + "'}]},{'varId':'frontaldistance','measures': [{'mTimeStamp': '"
                        + std::to_string(
                                a_c.getSampleTimeStamp().toMicroseconds())
                        + "','value':'"
                        + std::to_string(inIterator->ReadShort())
                        + "'}]},{'varId':'rightdistance','measures': [{'mTimeStamp': '"
                        + std::to_string(
                                a_c.getSampleTimeStamp().toMicroseconds())
                        + "','value':'"
                        + std::to_string(inIterator->ReadShort())
                        + "'}]},{'varId':'leftdistance','measures': [{'mTimeStamp': '"
                        + std::to_string(
                                a_c.getSampleTimeStamp().toMicroseconds())
                        + "','value':'"
                        + std::to_string(inIterator->ReadShort())
                        + "'}]},{'varId':'reardistance','measures': [{'mTimeStamp': '"
                        + std::to_string(
                                a_c.getSampleTimeStamp().toMicroseconds())
                        + "','value':'"
                        + std::to_string(inIterator->ReadShort())
                        + "'}]}]}]}";
      sendMessage = true;
      }

    } else if (a_c.getDataType()
            == opendlv::data::environment::WGS84Coordinate::ID()) {
      /**--------------APPLANIX GPS DATA---------------------**/
      opendlv::data::environment::WGS84Coordinate gps = a_c.getData<
              opendlv::data::environment::WGS84Coordinate>();
      data +=
              "{'systemId' : 'openDlvMonitorv0','timeStamp':'"
                      + std::to_string(
                              a_c.getReceivedTimeStamp().toMicroseconds())
                      + "',"
                      + "'context': [{'services': ['laneFollower']}],'monitors': [{'monitorId':'applanixGps','measurements': [{'varId':'latitude','measures': [{'mTimeStamp': '"
                      + std::to_string(
                              a_c.getSampleTimeStamp().toMicroseconds())
                      + "','value':'" + std::to_string(gps.getLatitude())
                      + "'}]},{'varId':'longitude','measures': [{'mTimeStamp': '"
                      + std::to_string(
                              a_c.getSampleTimeStamp().toMicroseconds())
                      + "','value':'" + std::to_string(gps.getLongitude())
                      + "'}]}]}]}";
      sendMessage = true;

    } else if (a_c.getDataType() == opendlv::device::gps::pos::Grp1Data::ID()) {
      /**--------------APPLANIX IMU DATA---------------------**/
      opendlv::device::gps::pos::Grp1Data grp1 = a_c.getData<
              opendlv::device::gps::pos::Grp1Data>();
      //speed is translated into km/h (speed*3600/1000)
      data +=
              "{'systemId' : 'openDlvMonitorv0','timeStamp':'"
                      + std::to_string(
                              a_c.getReceivedTimeStamp().toMicroseconds())
                      + "',"
                      + "'context': [{'services': ['laneFollower']}],'monitors': [{'monitorId':'applanixGps','measurements': [{'varId':'speed','measures': [{'mTimeStamp': '"
                      + std::to_string(
                              a_c.getSampleTimeStamp().toMicroseconds())
                      + "','value':'"
                      + std::to_string(grp1.getSpeed() * 3600 / 1000)
                      + "'}]}]}]}";
      sendMessage = true;

    }
  } else {
    /**--------------IS SIMULATION---------------------**/
    if (a_c.getDataType() == Voice::ID() && a_c.getSenderStamp() == 1) {
      /**--------------V2VCAM/DENM DATA FROM EXTERNAL SYSTEMS---------------------**/
      Voice voice = a_c.getData<Voice>();

      if (voice.getType() == "cam") {
        m_v2vcam = true;
      } else if (voice.getType() == "denm") {
        //Denm message received from v1 to report by v0 to ksam
        data += "{'systemId' : 'openDlvMonitorv0','timeStamp':'"
                + std::to_string(a_c.getReceivedTimeStamp().toMicroseconds())
                + "'," + "'context': [{'services': [";
        if (m_laneFollower) {
          data += "'laneFollower'";
        }
        data +=
                "]}],'monitors': [{'monitorId':'V2VDenm_Event','measurements': [{'varId':'Event','measures': [{'mTimeStamp': '"
                        + std::to_string(
                                a_c.getSampleTimeStamp().toMicroseconds())
                        + "','value':'CRASH'}]}]}]}";
        sendMessage = true;
      }

    } else if (a_c.getSenderStamp() == 0) {
      /**--------------SENSOR DATA FROM INTERNAL SYSTEMS---------------------**/
      if (a_c.getDataType() == automotive::VehicleControl::ID()) {
        /**--------------SELF-DRIVING/LANEFOLLOWER DATA---------------------**/
        automotive::VehicleControl vc =
                a_c.getData<automotive::VehicleControl>();
        if (vc.getSpeed() > 0) {
          m_laneFollower = true;
        } else {
          m_laneFollower = false;
        }
      }

      data += "{'systemId' : 'openDlvMonitorv"
              + std::to_string(a_c.getSenderStamp()) + "','timeStamp':'"
              + std::to_string(a_c.getReceivedTimeStamp().toMicroseconds())
              + "'," + "'context': [{'services': [";
      if (m_laneFollower) {
        data += "'laneFollower'";
      }
      data += "]}],'monitors': [";

      if (a_c.getDataType() == V2vRequest::ID()) {
        /**--------------V2VREQUEST DATA---------------------**/
        V2vRequest vr = a_c.getData<V2vRequest>();
        istringstream(vr.getData()) >> m_v2vcamRequest;

      } else if (a_c.getDataType() == odcore::data::image::SharedImage::ID()) {
        /**--------------ODSIMCAMERA DATA---------------------**/
        odcore::data::image::SharedImage sharedImg = a_c.getData<
                odcore::data::image::SharedImage>();
        data +=
                "{'monitorId':'odsimcamera','measurements': [{'varId':'imgSize','measures': [{'mTimeStamp': '"
                        + std::to_string(
                                a_c.getSampleTimeStamp().toMicroseconds())
                        + "','value':'" + std::to_string(sharedImg.getSize())
                        + "'}]}]}]}";
        //		  	std::cout << data << std::endl;
        sendMessage = true;
      } else if (a_c.getDataType()
              == automotive::miniature::SensorBoardData::ID()) {
        /**--------------IRUS DATA---------------------**/
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
                        + std::to_string(
                                a_c.getSampleTimeStamp().toMicroseconds())
                        + "','value':'" + std::to_string(i_frontRightDistance)
                        + "'}]}]},";
        if (m_v2vcam && m_v2vcamRequest) {
          double v2v = sbd.getValueForKey_MapOfDistances(6);
          data +=
                  "{'monitorId':'V2VCam_FrontCenter','measurements': [{'varId':'FrontCenterDistance','measures': [{'mTimeStamp': '"
                          + std::to_string(
                                  a_c.getSampleTimeStamp().toMicroseconds())
                          + "','value':'" + std::to_string(v2v) + "'}]}]},";
        }
        data +=
                "{'monitorId':'Infrared_Rear','measurements': [{'varId':'RearDistance','measures': [{'mTimeStamp': '"
                        + std::to_string(
                                a_c.getSampleTimeStamp().toMicroseconds())
                        + "','value':'" + std::to_string(i_rearDistance)
                        + "'}]}]},"
                        + "{'monitorId':'Infrared_RearRight','measurements': [{'varId':'RearRightDistance','measures': [{'mTimeStamp': '"
                        + std::to_string(
                                a_c.getSampleTimeStamp().toMicroseconds())
                        + "','value':'" + std::to_string(i_rearRightDistance)
                        + "'}]}]},"
                        + "{'monitorId':'UltraSonic_FrontCenter','measurements': [{'varId':'FrontCenterDistance','measures': [{'mTimeStamp': '"
                        + std::to_string(
                                a_c.getSampleTimeStamp().toMicroseconds())
                        + "','value':'" + std::to_string(u_frontCenterDistance)
                        + "'}]}]},"
                        + "{'monitorId':'UltraSonic_FrontRight','measurements': [{'varId':'FrontRightDistance','measures': [{'mTimeStamp': '"
                        + std::to_string(
                                a_c.getSampleTimeStamp().toMicroseconds())
                        + "','value':'" + std::to_string(u_frontRightDistance)
                        + "'}]}]},"
                        + "{'monitorId':'UltraSonic_RearRight','measurements': [{'varId':'RearRightDistance','measures': [{'mTimeStamp': '"
                        + std::to_string(
                                a_c.getSampleTimeStamp().toMicroseconds())
                        + "','value':'" + std::to_string(u_rearRightDistance)
                        + "'}]}]}]}";
        //		std::cout << data << std::endl;
        sendMessage = true;
      } else if (a_c.getDataType() == automotive::VehicleData::ID()) {
        /**--------------ODSIMVEHICLE DATA---------------------**/
        automotive::VehicleData vd = a_c.getData<automotive::VehicleData>();

        // Assume vehicle is never in reverse
        double speed = vd.getSpeed();
        if (speed < 0) {
          speed = 0;
        }

        double x = vd.getPosition().getP()[0];
        double y = vd.getPosition().getP()[1];

        data +=
                "{'monitorId':'imuodsimcvehicle','measurements': [{'varId':'speed','measures': [{'mTimeStamp': '"
                        + std::to_string(
                                a_c.getSampleTimeStamp().toMicroseconds())
                        + "','value':'" + std::to_string(speed)
                        + "'}]},{'varId':'longitude','measures': [{'mTimeStamp': '"
                        + std::to_string(
                                a_c.getSampleTimeStamp().toMicroseconds())
                        + "','value':'" + std::to_string(x)
                        + "'}]},{'varId':'latitude','measures': [{'mTimeStamp': '"
                        + std::to_string(
                                a_c.getSampleTimeStamp().toMicroseconds())
                        + "','value':'" + std::to_string(y) + "'}]}]}]}";
        //            std::cout << data << std::endl;
        sendMessage = true;
      }
    }
  }

  if (sendMessage) {
//    std::cout << data << std::endl;
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

    //        std::cout << data << std::endl;
    if (write(socket_client, data.c_str(), strlen(data.c_str())) < 0) {
      std::cout << "Data send failed" << std::endl;
    }

    close(socket_client);

    sendMessage = false;
  }

}

}
}
}
