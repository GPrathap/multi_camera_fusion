/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include "modules/parking/parking.h"

#include <iomanip>
#include <string>

#include "ros/include/std_msgs/String.h"

#include "modules/localization/proto/localization.pb.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/common/time/time.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/parking/common/parking_gflags.h"

namespace apollo {
namespace parking {

using apollo::canbus::Chassis;
using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::VehicleStateProvider;
using apollo::common::adapter::AdapterManager;
using apollo::common::monitor::MonitorMessageItem;
using apollo::common::time::Clock;
using apollo::localization::LocalizationEstimate;
using apollo::control::PadMessage;
using apollo::control::DrivingAction;

std::string Parking::Name() const { return "parking"; }

Status Parking::Init() {
  init_time_ = Clock::NowInSeconds();

  AINFO << "Parking init, starting ...";
  CHECK(common::util::GetProtoFromFile(FLAGS_parking_conf_file, &parking_conf_))
      << "Unable to load parking conf file: " + FLAGS_parking_conf_file;

  AINFO << "Conf file: " << FLAGS_parking_conf_file << " is loaded.";

  AdapterManager::Init(FLAGS_parking_adapter_config_filename);

  common::monitor::MonitorLogBuffer buffer(&monitor_logger_);

  // lock it in case for after sub, init_vehicle not ready, but msg trigger
  // come
  CHECK(AdapterManager::GetLocalization())
      << "Localization is not initialized.";

  CHECK(AdapterManager::GetChassis()) << "Chassis is not initialized.";

  CHECK(AdapterManager::GetPad()) << "Pad is not initialized.";

  parking_state_.set_status(ParkingStatus::WAITING_START);

  return Status::OK();
}

Status Parking::Start() {
  // set initial vehicle state by cmd
  // need to sleep, because advertised channel is not ready immediately
  // simple test shows a short delay of 80 ms or so
  AINFO << "Parking resetting vehicle state, sleeping for 1000 ms ...";
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  timer_ = AdapterManager::CreateTimer(
      ros::Duration(parking_conf_.control_period()), &Parking::OnTimer, this);

  AINFO << "Parking init done!";

  return Status::OK();
}


void Parking::SendRouteRequest(double x_s, double y_s, double x, double y)
{
  apollo::routing::RoutingRequest routing_request;
  AdapterManager::FillRoutingRequestHeader("routing", &routing_request);
  routing_request.clear_waypoint();
  auto *start_point = routing_request.add_waypoint();
  //start_point->set_id(lane->id().id());
  //start_point->set_s(s);

  auto point = common::util::MakePointENU(x_s, y_s, 0.0);

  start_point->mutable_pose()->CopyFrom(point);

  auto *end_point = routing_request.add_waypoint();

  auto point2 = common::util::MakePointENU(
      x, y, 0.0);
  
  end_point->mutable_pose()->CopyFrom(point2);

  AdapterManager::PublishRoutingRequest(routing_request);

  SendPAD();
}


void Parking::SendPAD() {
  PadMessage pad;
  pad.set_action(DrivingAction::START);
  AdapterManager::FillPadHeader("parking", &pad);
  AdapterManager::PublishPad(pad);
  AINFO << "send pad_message OK";
}

void Parking::OnTimer(const ros::TimerEvent &) {

  AdapterManager::Observe();
  auto localization_adapter = AdapterManager::GetLocalization();
  if (localization_adapter->Empty()) {
    AWARN_EVERY(100) << "No Localization msg yet. ";
    return;
  }
  localization_ = localization_adapter->GetLatestObserved();

  auto chassis_adapter = AdapterManager::GetChassis();
  if (chassis_adapter->Empty()) {
    AWARN_EVERY(100) << "No Chassis msg yet. ";
    return;
  }
  chassis_ = chassis_adapter->GetLatestObserved();


  
  double x = localization_.pose().position().x();
  double y = localization_.pose().position().y();

  ADEBUG << "Current position: x=" << x << " y="<< y << " speed: " << chassis_.speed_mps();
  switch (parking_state_.status())
  {
    case ParkingStatus::WAITING_START:
      if  (x<parking_conf_.desicion_slot_conf().x_right() && x>parking_conf_.desicion_slot_conf().x_left() 
            && y>parking_conf_.desicion_slot_conf().y_bottom() && y<parking_conf_.desicion_slot_conf().y_top() && chassis_.speed_mps() < 0.1)
      {
        ADEBUG << "Reach destination. Looking for parking slot";
        parking_state_.set_status(ParkingStatus::SEEKING_PLACE);
      }
      break;

    case ParkingStatus::SEEKING_PLACE:
      if (!AdapterManager::GetPerceptionObstacles()->Empty()) {
        const auto& perception =
          AdapterManager::GetPerceptionObstacles()->GetLatestObserved();
          double max_dist = 0.0;
          for( auto park_slot: parking_conf_.slot_conf()){
            ADEBUG << "Check parking slot x=" << park_slot.x() << " y=" << park_slot.y();
            double min_dist = 1000.0;
            for( auto obstacle : perception.perception_obstacle())
            {
              ADEBUG << "Obstacle position x=" << obstacle.position().x() << " y=" << obstacle.position().y();
              double dist = sqrt((park_slot.x()-obstacle.position().x())*(park_slot.x()-obstacle.position().x()) + (park_slot.y()-obstacle.position().y())*(park_slot.y()-obstacle.position().y()));
              if (dist<min_dist)
              {
                min_dist = dist;
              }
            }
            ADEBUG << "Min dist to obstacle: " << min_dist;
            if (min_dist>max_dist)
            {
              max_dist = min_dist; 
              park_x = park_slot.x();
              park_y = park_slot.y();
            }
          }
          ADEBUG << "Choose parking slot x=" << park_x << " y=" << park_y << " with dist " << max_dist;

          parking_state_.set_status(ParkingStatus::REQUEST_SENT);
      }
      break;
    case ParkingStatus::REQUEST_SENT:
        SendRouteRequest(x,y,park_x, park_y);
        
        ADEBUG << "Send new route";
        //parking_state_.set_status(ParkingStatus::COMPLETE);
        break;
  
    default:
      break;
  }
  
}




void Parking::Stop() {}

}  // namespace control
}  // namespace apollo
