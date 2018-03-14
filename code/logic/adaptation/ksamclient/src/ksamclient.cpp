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

#include "../include/ksamclient.hpp"

namespace opendlv {
namespace logic {
namespace adaptation {

/**
  * Constructor.
  *
  * @param a_argc Number of command line arguments.
  * @param a_argv Command line arguments.
  */
KsamClient::KsamClient(int32_t const &a_argc, char **a_argv)
    : DataTriggeredConferenceClientModule(
      a_argc, a_argv, "logic-adaptation-ksamclient")
    , m_initialized(false)
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
	std::string data("{'systemId':'opendDlvMonitor', 'Monitor':'Camera', 'measure':'");
	bool sendMessage = false;

	if (a_c.getDataType() == odcore::data::image::SharedImage::ID()) {
		odcore::data::image::SharedImage sharedImg =
		        a_c.getData<odcore::data::image::SharedImage>();
		std::shared_ptr<odcore::wrapper::SharedMemory> sharedMem(
		      odcore::wrapper::SharedMemoryFactory::attachToSharedMemory(
		    		  sharedImg.getName()));

	  if (sharedMem->isValid()) {
		  	const uint32_t nrChannels = sharedImg.getBytesPerPixel();
		  	data += std::to_string(nrChannels) + "'}";
//		  	sendMessage  = true;
	  } else {
		std::cout << "[" << getName() << "] " << "Sharedmem is not valid." << std::endl;
	  }
	}


	if(sendMessage){
		int socket_client = socket(AF_INET, SOCK_STREAM, 0);

		struct sockaddr_in server;
		server.sin_family = AF_INET;
		server.sin_port = htons(8083);
		server.sin_addr.s_addr = inet_addr("127.0.0.1");

		if(socket_client < 0)
		{
			std::cout << "Socket could not be created" << std::endl;
		}

		if(connect(socket_client, (struct sockaddr *)&server, sizeof(server))<0)
		{
			std::cout << "Connection failed due to port and ip problems" << std::endl;
		}

		std::cout << data << std::endl;
		if( write(socket_client, data.c_str(), strlen(data.c_str())) < 0)
		{
			std::cout << "Data send failed" << std::endl;
		}

		close(socket_client);
	}


}

}
}
}
