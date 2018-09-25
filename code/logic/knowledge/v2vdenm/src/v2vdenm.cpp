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

#include <chrono>
#include <ctype.h>
#include <cstring>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include "opendavinci/odcore/base/KeyValueConfiguration.h"

#include "opendavinci/odcore/data/Container.h"
#include "opendavinci/odcore/data/TimeStamp.h"

#include "odvdopendlvdata/GeneratedHeaders_ODVDOpenDLVData.h"

#include "buffer.hpp"
#include "v2vdenm.hpp"
#include "Voice.h"

namespace opendlv {
namespace logic {
namespace knowledge {

/**
 * Constructor.
 *
 * @param a_argc Number of command line arguments.
 * @param a_argv Command line arguments.
 */
V2vDenm::V2vDenm(const int32_t &a_argc, char **a_argv) :
    TimeTriggeredConferenceClientModule(a_argc, a_argv, "knowledge-v2vdenm"),
    // m_sendLog(),
    m_receiveLog(), m_timeType2004(), m_simulation(false), m_role(""), m_crash(
        false) {
}

V2vDenm::~V2vDenm() {
}

void V2vDenm::setUp() {
  std::cout << "V2V denm started" << std::endl;
  m_simulation = getKeyValueConfiguration().getValue<bool>(
      "knowledge-v2vdenm.simulation");
  m_role = getKeyValueConfiguration().getValue<string>(
      "knowledge-v2vdenm.role");
  m_crash = getKeyValueConfiguration().getValue<bool>(
      "knowledge-v2vdenm.crash");
}

void V2vDenm::tearDown() {
}

odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode V2vDenm::body() {
  while (getModuleStateAndWaitForRemainingTimeInTimeslice()
      == odcore::data::dmcp::ModuleStateMessage::RUNNING) {
    if (m_role == "witness" && m_crash) {
      std::shared_ptr<Buffer> outBuffer(new Buffer());
      // Reverser for big and little endian specification of V2V.
      outBuffer->Reversed();
      outBuffer->AppendByte(1); //messageId
      outBuffer->AppendInteger(getIdentifier()); //stationId
      outBuffer->AppendInteger(GenerateGenerationTime()); //generationTime
      outBuffer->AppendInteger(50); //latitude of event (y)
      outBuffer->AppendInteger(0); //longitude of event (x)
      outBuffer->AppendInteger(4); //serverity (1,2,3,4)
      outBuffer->AppendInteger(0); //cause code (0 - traffic stopped)
      outBuffer->AppendInteger(0); //sub-cause code (0 - crash)

      std::vector<unsigned char> bytes = outBuffer->GetBytes();
      std::string bytesString(bytes.begin(), bytes.end());
      Voice nextMessage("denm", bytesString.size(), bytesString);
      odcore::data::Container c(nextMessage);
      std::cout << "Send message type " << std::to_string(c.getDataType())
          << ",CRASH!" << std::endl;
      getConference().send(c);
      m_crash = false;
    }
  }

  return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
}

void V2vDenm::nextContainer(odcore::data::Container &a_c) {
//  if (m_simulation) {
  if (a_c.getDataType() == Voice::ID()
      && !(a_c.getSenderStamp() == getIdentifier())) {
    Voice voice = a_c.getData<Voice>();
    if (voice.getType() == "denm") {
      std::cout << "Process denm event message (crash)" << std::endl;
    }
  }
//  }
}

uint64_t V2vDenm::GenerateGenerationTime() const {
  std::chrono::system_clock::time_point start2004TimePoint =
      std::chrono::system_clock::from_time_t(m_timeType2004);
  uint64_t millisecondsSince2004Epoch =
      std::chrono::system_clock::now().time_since_epoch()
          / std::chrono::milliseconds(1)
          - start2004TimePoint.time_since_epoch()
              / std::chrono::milliseconds(1);
  return millisecondsSince2004Epoch;
}

}
}
}
