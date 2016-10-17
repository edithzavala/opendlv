/**
 * Copyright (C) 2015 Chalmers REVERE
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
#include <cmath>
#include <iostream>

#include "opendavinci/odcore/data/Container.h"
#include "opendavinci/odcore/data/TimeStamp.h"

#include "odvdopendlvdata/GeneratedHeaders_ODVDOpenDLVData.h"

#include "opendavinci/odcore/strings/StringToolbox.h"

#include "gcdc16/rule/intersectionleft/intersectionleft.hpp"

namespace opendlv {
namespace knowledge {
namespace gcdc16 {
namespace rule {
namespace intersectionleft {

/**
* Constructor.
*
* @param a_argc Number of command line arguments.
* @param a_argv Command line arguments.
*/
IntersectionLeft::IntersectionLeft(int32_t const &a_argc, char **a_argv)
: TimeTriggeredConferenceClientModule(
  a_argc, a_argv, "knowledge-gcdc16-intersectionleft"),
  m_enableLaneFollowing(),
  m_runScenario(false),
  m_desiredGroundSpeed(),
  m_previousAzimuthFollowed()
{
}

IntersectionLeft::~IntersectionLeft()
{
}


odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode IntersectionLeft::body()
{
  while (getModuleStateAndWaitForRemainingTimeInTimeslice() ==
      odcore::data::dmcp::ModuleStateMessage::RUNNING) {

  } 
  return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
}

void IntersectionLeft::ActOnEnvironment(opendlv::perception::Environment &a_environment)
{
  odcore::data::TimeStamp now;
  odcore::data::TimeStamp environmentValidUntil = a_environment.getValidUntil();
  if (now.toMicroseconds() - environmentValidUntil.toMicroseconds() > 0) {
    return;
  }

  auto listOfObjects = a_environment.getListOfObjects();
  if (listOfObjects.size() > 0) {
    ActOnMio(listOfObjects);
  }
  
  auto listOfSurfaces = a_environment.getListOfSurfaces();
  if (listOfSurfaces.size() > 0) {
    ActOnLane(listOfSurfaces);
  }
}

void IntersectionLeft::ActOnMio(std::vector<opendlv::perception::Object> &a_listOfObjects)
{
  (void) a_listOfObjects;
  /*
  opendlv::perception::Object mio;
  std::vector<float> scoreList;

  for(auto object:a_listOfObjects){
    opendlv::model::Direction direction = object.getDirection();
    float azimuth = direction.getAzimuth();
    float distance = object.getDistance();
    std::vector<std::string> sources = object.getListOfSources();

    float score = 0;
    score += (std::pow(8.0f,2.0f
      )-std::pow(azimuth*static_cast<float>(opendlv::Constants::RAD2DEG)/45.0f*8.0f,2.0f));
    score += (100-distance)*3;
    for(auto it:sources){
      score += 80;
    }

    if(distance > m_mioDistanceRange || std::abs(azimuth) > m_mioAngleRange){
      score = 0;
    }
    scoreList.push_back(score);
    std::cout << "Id: " << object.getObjectId() << " score: " << score << " scources: ";
    for(auto it:sources){
      std::cout << it << " ";
    }

    std::cout << std::endl;
  }
  auto highestScore = std::max_element(scoreList.begin(),scoreList.end());
  if(*highestScore > 0){
    auto winnerIndex = std::distance(scoreList.begin(), highestScore);
    m_mio = a_listOfObjects[winnerIndex];

    std::cout << "Winner Id: " << a_listOfObjects[winnerIndex].getObjectId() << " Score: " << *highestScore << std::endl;
    odcore::data::TimeStamp now;
    m_mioValidUntil = odcore::data::TimeStamp(now.getSeconds()+m_memoryDuration,now.getFractionalMicroseconds());

  }
  */
}

void IntersectionLeft::ActOnLane(std::vector<opendlv::perception::Surface> &a_listOfSurfaces)
{
  float azimuth_pre = m_previousAzimuthFollowed;
  float azimuth_new = 0.0f;

  for (auto surface : a_listOfSurfaces) {
    auto listOfEdges = surface.getListOfEdges();

    if (listOfEdges.size() < 4) {
      opendlv::model::Cartesian3 topLeftCorner = listOfEdges[0];
      opendlv::model::Cartesian3 topRightCorner = listOfEdges[3];

      float x_m = 0.5f * (topLeftCorner.getX() - topRightCorner.getX());
      float y_m = 0.5f * (topLeftCorner.getY() - topRightCorner.getY());

      float azimuth = atan2(y_m, x_m); 

      if (std::abs(azimuth_pre - azimuth) < std::abs(azimuth_pre - azimuth_new)) {
        azimuth_new = azimuth;
      }
    }
  }

  m_previousAzimuthFollowed = azimuth_new;
  ControlDirectionOfMovement(azimuth_new);
}

void IntersectionLeft::nextContainer(odcore::data::Container &a_container)
{
  if (a_container.getDataType() == (opendlv::knowledge::Insight::ID())) {
    opendlv::knowledge::Insight insight = a_container.getData<opendlv::knowledge::Insight>();
    if (insight.getInsight() == "scenarioReady") {
      m_runScenario = true;
    }
  } else if (a_container.getDataType() == opendlv::perception::Environment::ID()) {
    opendlv::perception::Environment message = a_container.getData<opendlv::perception::Environment>();
    ActOnEnvironment(message);
  } else if (a_container.getDataType() == opendlv::proxy::reverefh16::Propulsion::ID()) {
    auto propulsion = a_container.getData<opendlv::proxy::reverefh16::Propulsion>();
    double speedKph = propulsion.getPropulsionShaftVehicleSpeed();
    float speed = static_cast<float>(speedKph / 3.6);
    ControlGroundSpeed(speed);
  }
}

void IntersectionLeft::ControlGroundSpeed(float a_speed)
{
  std::cout << "Speed control - wants " << m_desiredGroundSpeed << " has " << a_speed << std::endl;
  odcore::data::TimeStamp now;
  if (m_runScenario) {
    opendlv::perception::StimulusGroundSpeed sgs(now, m_desiredGroundSpeed, a_speed);
    odcore::data::Container container(sgs);
    getConference().send(container);
  }
}

void IntersectionLeft::ControlDirectionOfMovement(float a_azimuth)
{
  std::cout << "Direction control - wants " << a_azimuth << std::endl;
  odcore::data::TimeStamp now;
  if (m_runScenario) {
    opendlv::model::Direction direction(a_azimuth, 0.0f);
    opendlv::perception::StimulusDirectionOfMovement sdom(now, direction, opendlv::model::Direction(0.0f, 0.0f));
    odcore::data::Container container(sdom);
    getConference().send(container);
  }
}

void IntersectionLeft::setUp()
{
  odcore::base::KeyValueConfiguration kv = getKeyValueConfiguration();
  m_desiredGroundSpeed = static_cast<float>(kv.getValue<double>("knowledge-gcdc16-rule-intersection-left.desiredAngularSize"));
  m_enableLaneFollowing = kv.getValue<bool>("knowledge-gcdc16-rule-intersection-left.enableLaneFollowing");
  bool forceScenarioStart = kv.getValue<bool>("knowledge-gcdc16-rule-intersection-left.forceScenarioStart");

  if (forceScenarioStart) {
    m_runScenario = true;
  }
}

void IntersectionLeft::tearDown()
{
}

} // intersectionleft
} // rule
} // gcdc16
} // knowledge
} // opendlv
