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

#include <stdio.h> // standard input and output library
#include <stdlib.h> // this includes functions regarding memory allocation
#include <string.h> // contains string functions
#include <unistd.h> //contains various constants
#include <sys/types.h> //contains a number of basic derived types that should be used whenever appropriate
#include <arpa/inet.h> // defines in_addr structure
#include <sys/socket.h> // for socket creation
#include <netinet/in.h> //contains constants and structures needed for internet domain addresses
#include <iostream>
#include <thread>
#include <list>

#include "opendavinci/odcore/wrapper/SharedMemoryFactory.h"
#include "opendavinci/odcore/wrapper/SharedMemory.h"
#include "opendavinci/odcore/base/KeyValueConfiguration.h"
#include "opendavinci/odcore/data/Container.h"
//#include "odvdopendlvdata/GeneratedHeaders_ODVDOpenDLVData.h"
#include "opendavinci/odcore/data/TimeStamp.h"
#include "MonitorAdaptation.h"

#include "V2vRequest.h"
#include "../include/ksamServer.hpp"

namespace opendlv {
namespace logic {
namespace adaptation {

using namespace odcore::data;
/**
 * Constructor.
 *
 * @param a_argc Number of command line arguments.
 * @param a_argv Command line arguments.
 */
KsamServer::KsamServer(const int32_t &argc, char **argv) :
        TimeTriggeredConferenceClientModule(argc, argv,
                "adaptation-ksamserver"), m_simulation(false) {
}

KsamServer::~KsamServer() {
}

void KsamServer::setUp() {
  odcore::base::KeyValueConfiguration kv = getKeyValueConfiguration();
  m_simulation = kv.getValue<bool>("adaptation-ksamserver.simulation");
}

void KsamServer::tearDown() {
}

void KsamServer::nextContainer(odcore::data::Container &c) {
  c.getDataType();
}

odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode KsamServer::body() {
  while (getModuleStateAndWaitForRemainingTimeInTimeslice()
          == odcore::data::dmcp::ModuleStateMessage::RUNNING) {
    std::thread t1(&KsamServer::runServer, this);
    t1.join();
  }

  return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
}

[[noreturn]] void KsamServer::runServer() {
  char buffer[1025];
  int socketServer, socketClient, c;
  struct sockaddr_in server, client;
  char * recAdapt;
  char * vehicleId;

  uint32_t vehIdInt;

  if ((socketServer = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
    std::cout << "Could not create socket" << std::endl;
  }

  server.sin_family = AF_INET;
  server.sin_addr.s_addr = INADDR_ANY;
  server.sin_port = htons(8082);

  if ((bind(socketServer, (struct sockaddr*) &server, sizeof(server))) < 0) {
    std::cout << "Bind failed" << std::endl;
  }

  listen(socketServer, 5);

  std::cout << "Server (opendlv) up" << std::endl;
  c = sizeof(struct sockaddr_in);

  while (1) {
    socketClient = accept(socketServer, (struct sockaddr*) &client,
            (socklen_t*) &c);
//    	std::cout << "Client connected" << std::endl;

    read(socketClient, buffer, 1024);
    recAdapt = strdup(buffer);
    std::cout << "Receive message: " << buffer << std::endl;
    std::string v2vAdapt("");
//    std::string cameraAdapt("");
//    std::string gpsAdapt("");

    if (strsep(&recAdapt, ";:") != NULL) { //vehicleId tag
      vehicleId = strsep(&recAdapt, ";:");
      vehIdInt = (uint32_t) atoi(vehicleId);
      std::cout << "OpenDLV-Adapt vehicle with id: " << std::to_string(vehIdInt)
              << std::endl;
    }

    if (strsep(&recAdapt, ";:") != NULL) { //monitorsToAddTag
      char * monitorsToAdd = strsep(&recAdapt, ";:");
      std::cout << "List of monitors to add: " << monitorsToAdd << std::endl;
      char * monitorIdToAdapt;
      while ((monitorIdToAdapt = strsep(&monitorsToAdd, ",")) != NULL) {
        std::cout << "OpenDLV-Monitor to add: " << monitorIdToAdapt
                << std::endl;
        MonitorAdaptation mAdapt;
        mAdapt.setVehicleId(vehIdInt);
        mAdapt.setMonitorName(monitorIdToAdapt);
        mAdapt.setAction("add");
        Container data = Container(mAdapt);
//        data.setSenderStamp(vehIdInt);
        getConference().send(data);
        std::string monId(monitorIdToAdapt);
        if (monId.compare("V2VCam_FrontCenter") == 0) {
          v2vAdapt = "add";
        }
//        else if (monitorIdToAdapt.compare("axiscamera") != 0) {
//          cameraAdapt = "add";
//        } else if (monitorIdToAdapt.compare("applanixGps") != 0) {
//          gpsAdapt = "add";
//        }
      }

    }

    if (strsep(&recAdapt, ";:") != NULL) { //monitorsToRemoveTag
      char * monitorsToRemove = strsep(&recAdapt, ";:");
      std::cout << "List of monitors to remove: " << monitorsToRemove
              << std::endl;
      char * monitorIdToAdapt;
      while ((monitorIdToAdapt = strsep(&monitorsToRemove, ",")) != NULL) {
        std::cout << "OpenDLV-Monitors to remove: " << monitorIdToAdapt
                << std::endl;
        MonitorAdaptation mAdapt;
        mAdapt.setVehicleId(vehIdInt);
        mAdapt.setMonitorName(monitorIdToAdapt);
        mAdapt.setAction("remove");
        Container data = Container(mAdapt);
//        data.setSenderStamp(vehIdInt);
        getConference().send(data);
        std::string monId(monitorIdToAdapt);
        if (monId.compare("V2VCam_FrontCenter") == 0) {
          v2vAdapt = "remove";
        }
//        else if (monitorIdToAdapt.compare("axiscamera") != 0) {
//          cameraAdapt = "remove";
//        } else if (monitorIdToAdapt.compare("applanixGps") != 0) {
//          gpsAdapt = "remove";
//        }
      }
    }

    if (!m_simulation) {
      if (v2vAdapt.compare("add") == 0) {
        std::string request("1");
        V2vRequest nextMessage(request.size(), request);
        odcore::data::Container cReq(nextMessage);
        getConference().send(cReq);

      } else if (v2vAdapt.compare("remove") == 0) {
        std::string request("0");
        V2vRequest nextMessage(request.size(), request);
        odcore::data::Container cReq(nextMessage);
        getConference().send(cReq);
      }

//      if (cameraAdapt.compare("add") != 0) {
//
//      } else if (cameraAdapt.compare("remove") != 0) {
//
//      }
//
//      if (gpsAdapt.compare("add") != 0) {
//
//      } else if (gpsAdapt.compare("remove") != 0) {
//
//      }

    }

    //AdaptationId:MONITORFAULT_openDlvMonitorv0,MonitorsToAdd:[v2vRear]
    /*
     *message opendlv.adaptation.MonitorAdaptation [id = 190] {
     uint32 vehicleId [id = 1];
     string monitorName [id = 2];
     string action [id = 3];
     }
     */
//    std::cout << "OpenDLV-Monitor adapt message: " << mAdapt.getMonitorName()
//            << std::endl;
    close(socketClient);
    //        sleep(1);
  }
}

}
}
}

