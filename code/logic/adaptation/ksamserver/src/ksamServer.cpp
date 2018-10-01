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
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sys/time.h>

#include "opendavinci/odcore/wrapper/SharedMemoryFactory.h"
#include "opendavinci/odcore/wrapper/SharedMemory.h"
#include "opendavinci/odcore/base/KeyValueConfiguration.h"
#include "opendavinci/odcore/data/Container.h"
#include "opendavinci/odcore/data/TimeStamp.h"

#include "MonitorAdaptation.h"
#include "TrafficData.h"
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
    TimeTriggeredConferenceClientModule(argc, argv, "adaptation-ksamserver"), m_simulation(
        false) {
}

KsamServer::~KsamServer() {
}

void KsamServer::setUp() {
  std::cout << "Ksam server started" << std::endl;
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

  /**********************Server up**********************************/
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
  c = sizeof(struct sockaddr_in);
  std::cout << "Server (opendlv) up" << std::endl;
  /****************************************************************/
  while (1) {
    socketClient = accept(socketServer, (struct sockaddr*) &client,
        (socklen_t*) &c);
//    	std::cout << "Client connected" << std::endl;
    read(socketClient, buffer, 1024);
    timeval curTime;
    gettimeofday(&curTime, NULL);
    int milli = curTime.tv_usec / 1000;
    char b[80];
    strftime(b, 80, "%Y-%m-%d %H:%M:%S", localtime(&curTime.tv_sec)); //change to localtime_r
    char currentTime[84] = "";
    sprintf(currentTime, "%s:%d", b, milli);
//    printf("%s - Receive adaptation request:", currentTime);
    std::cout << buffer << std::endl;

    char * mssgRec = strdup(buffer);
    char * mssgType;
    mssgType = strsep(&mssgRec, ":");
    if (mssgType != NULL) { //1st tag
      if (strcmp(mssgType, "TrafficFactor") == 0) {
        processTrafficData(buffer);
      } else {
        processAdaptation(buffer);
      }
    }
    close(socketClient);
    //        sleep(1);
  }
}

void KsamServer::processTrafficData(char *a_buffer) {
  char * recTF = strdup(a_buffer);
  double tf;

  if (strsep(&recTF, ":") != NULL) { //tf tag
    tf = (uint32_t) atoi(strsep(&recTF, ":"));
  }

  TrafficData tData;
  tData.setTrafficFactor(tf);
  Container data = Container(tData);
  getConference().send(data);
}

void KsamServer::processAdaptation(char *a_buffer) {
  char * recAdapt = strdup(a_buffer);
  char * vehicleId;
  uint32_t vehIdInt;
  std::string v2vAdapt("");

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
//        std::cout << "OpenDLV-Monitor to add: " << monitorIdToAdapt
//                << std::endl;
      MonitorAdaptation mAdapt;
      mAdapt.setVehicleId(vehIdInt);
      mAdapt.setMonitorName(monitorIdToAdapt);
      mAdapt.setAction("add");
      Container data = Container(mAdapt);
      getConference().send(data);
      std::string monId(monitorIdToAdapt);
      if (monId.compare("V2VCam_FrontCenter") == 0) {
        v2vAdapt = "add";
      }
    }

  }

  if (strsep(&recAdapt, ";:") != NULL) { //monitorsToRemoveTag
    char * monitorsToRemove = strsep(&recAdapt, ";:");
    std::cout << "List of monitors to remove: " << monitorsToRemove
        << std::endl;
    char * monitorIdToAdapt;
    while ((monitorIdToAdapt = strsep(&monitorsToRemove, ",")) != NULL) {
//        std::cout << "OpenDLV-Monitors to remove: " << monitorIdToAdapt
//                << std::endl;
      MonitorAdaptation mAdapt;
      mAdapt.setVehicleId(vehIdInt);
      mAdapt.setMonitorName(monitorIdToAdapt);
      mAdapt.setAction("remove");
      Container data = Container(mAdapt);
      getConference().send(data);
      std::string monId(monitorIdToAdapt);
      if (monId.compare("V2VCam_FrontCenter") == 0) {
        v2vAdapt = "remove";
      }
    }
  }

  if (!m_simulation) { //if simulation, request is sent by the IRUS system
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
  }
}

}
}
}

