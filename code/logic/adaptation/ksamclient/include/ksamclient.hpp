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

#ifndef LOGIC_ADAPTATION_KSAMCLIENT_KSAMCLIENT_HPP
#define LOGIC_ADAPTATION_KSAMCLIENT_KSAMCLIENT_HPP

#include <deque>
#include <fstream>
#include <iostream>
#include <memory>
#include <utility>

#include <opencv2/core/core.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/highgui/highgui.hpp>


#include "opendavinci/GeneratedHeaders_OpenDaVINCI.h"
#include "opendavinci/odcore/base/module/DataTriggeredConferenceClientModule.h"
#include "opendavinci/odcore/base/Mutex.h"
#include "opendavinci/odcore/data/Container.h"
#include "opendavinci/odcore/data/TimeStamp.h"

namespace opendlv {
namespace logic {
namespace adaptation {

class KsamClient
: public odcore::base::module::DataTriggeredConferenceClientModule {
 public:
    KsamClient(const int32_t &, char **);
    KsamClient(const KsamClient &) = delete;
    KsamClient &operator=(const KsamClient &) = delete;
  virtual ~KsamClient();
  virtual void nextContainer(odcore::data::Container &);

 private:
  void setUp();
  void tearDown();
  bool m_initialized;
    bool m_v2vcam;
    bool m_v2vcamRequest;
    bool m_laneFollower;
    bool m_simulation;
    bool m_cameraActive;
    bool m_gpsActive;
};

}
}
}

#endif
