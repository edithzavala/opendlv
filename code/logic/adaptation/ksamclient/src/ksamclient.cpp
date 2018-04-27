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

#include "V2vRequest.h"
#include "../include/ksamclient.hpp"
#include "Voice.h"

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
KsamClient::KsamClient(int32_t const &a_argc, char **a_argv)
    : DataTriggeredConferenceClientModule(
      a_argc, a_argv,
                "adaptation-ksamclient")
    , m_initialized(false)
    , m_v2vcam(false)
    , m_v2vcamRequest(
                false)
//    , m_mtx()
//    , m_debug()
{
}

//Ksam::Ksam(Ksam const &a_ksam): DataTriggeredConferenceClientModule(){
//	this = a_ksam;
//}

KsamClient::~KsamClient()
{
}
void KsamClient::setUp()
{
  odcore::base::KeyValueConfiguration kv = getKeyValueConfiguration();
//  kv.getValue<int32_t>("logic-adaptation-ksamclient.ok");
  m_initialized = true;
//  m_debug = (kv.getValue<int32_t>("logic-adaptation-ksam.debug") == 1);
}


void KsamClient::tearDown()
{
}

/**
 * Receives images from vision source and detect lanes.
 * Sends drivalble lanes objects.
 */
void KsamClient::nextContainer(odcore::data::Container &a_c)
{

  bool sendMessage = false;
  std::string data;
//  if (a_c.getDataType() == V2vRequest::ID()
//          && a_c.getSenderStamp() == 0) {
//      V2vRequest vr = a_c.getData<V2vRequest>();
//    istringstream(vr.getData()) >> m_v2vcamRequest;
//
//  }

  if (a_c.getDataType() == Voice::ID()
          && a_c.getSenderStamp() == 1) { //remove this hack for sending data to ksam of both vehicles
    Voice voice = a_c.getData<Voice>();

    if (voice.getType() == "cam") {
    m_v2vcam = true;
    } else if (voice.getType() == "denm") {
      //Denm message received from v1 to report by v0 to ksam
      data +=
              "{'systemId' : 'openDlvMonitorv0','timeStamp':'"
                      + std::to_string(
                              a_c.getReceivedTimeStamp().toMicroseconds())
                      + "','monitors': [{'monitorId':'V2VDenm_Event','measurements': [{'varId':'Event','measures': [{'mTimeStamp': '"
                        + std::to_string(
                                a_c.getSampleTimeStamp().toMicroseconds())
                      + "','value':'CRASH'}]}]}]}";
        sendMessage = true;
    }

  } else if (a_c.getSenderStamp() == 0) { //remove this hack for sending data to ksam of both vehicles
    data +=
            "{'systemId' : 'openDlvMonitorv"
                    + std::to_string(a_c.getSenderStamp()) + "','timeStamp':'"
                    + std::to_string(
                            a_c.getReceivedTimeStamp().toMicroseconds())
            + "','monitors': [";

      if (a_c.getDataType() == V2vRequest::ID()) {
        V2vRequest vr = a_c.getData<V2vRequest>();
        istringstream(vr.getData()) >> m_v2vcamRequest;

      } else if (a_c.getDataType() == odcore::data::image::SharedImage::ID()) {
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
                        + "','value':'" + std::to_string(v2v)
                        + "'}]}]},";
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
                      + "'}]}]}" + "]}";
      //		std::cout << data << std::endl;
      sendMessage = true;
    }
    //	if (a_c.getDataType() == automotive::VehicleData::ID()) {
    //		automotive::VehicleData vd = a_c.getData<automotive::VehicleData> ();
    //
    //	  	const double speed = vd.getSpeed();
    //		data += "'simvehicle','measurements': [{'varId':'speed','measures': [{'mTimeStamp': '" + a_c.getSampleTimeStamp().getYYYYMMDD_HHMMSSms()+ "','value':'" + std::to_string(speed) + "'}]}]}]}";
    //		std::cout << data << std::endl;
    //		sendMessage  = true;
    //	}
    }

    if (sendMessage) {
      //        std::cout << data << std::endl;
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
        std::cout << "Connection failed due to port and ip problems"
                << std::endl;
      }

      //        std::cout << data << std::endl;
      if (write(socket_client, data.c_str(), strlen(data.c_str())) < 0) {
        std::cout << "Data send failed" << std::endl;
      }

      close(socket_client);

//      sendMessage = false;
  }

}

}
}
}