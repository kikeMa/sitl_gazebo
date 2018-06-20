/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
/*
 * Desc: Contact plugin
 * Author: Nate Koenig mod by John Hsu
 */

#include "gazebo/physics/physics.hh"
#include "gazebo_sweep_v1_plugin.h"

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"

#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdio.h>
#include <boost/algorithm/string.hpp>

using namespace gazebo;
using namespace std;

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(SweepPlugin)

/////////////////////////////////////////////////
SweepPlugin::SweepPlugin()
{
}

/////////////////////////////////////////////////
SweepPlugin::~SweepPlugin()
{
#if GAZEBO_MAJOR_VERSION >= 7
  this->parentSensor_->LaserShape()->DisconnectNewLaserScans(
#else
  this->parentSensor_->GetLaserShape()->DisconnectNewLaserScans(
#endif
      this->newLaserScansConnection);
  this->newLaserScansConnection.reset();

  this->parentSensor_.reset();
  this->world.reset();
}

/////////////////////////////////////////////////
void SweepPlugin::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  // Get then name of the parent sensor
  this->parentSensor_ =
#if GAZEBO_MAJOR_VERSION >= 7
    std::dynamic_pointer_cast<sensors::RaySensor>(_parent);
#else
    boost::dynamic_pointer_cast<sensors::RaySensor>(_parent);
#endif

  if (!this->parentSensor_)
    gzthrow("SweepPlugin requires a Ray Sensor as its parent");

#if GAZEBO_MAJOR_VERSION >= 7
  this->world = physics::get_world(this->parentSensor_->WorldName());
#else
  this->world = physics::get_world(this->parentSensor_->GetWorldName());
#endif

  this->newLaserScansConnection =
#if GAZEBO_MAJOR_VERSION >= 7
    this->parentSensor_->LaserShape()->ConnectNewLaserScans(
#else
    this->parentSensor_->GetLaserShape()->ConnectNewLaserScans(
#endif
      boost::bind(&SweepPlugin::OnNewLaserScans, this));

  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzwarn << "[gazebo_lidar_plugin] Please specify a robotNamespace.\n";

  // get minimum distance
  if (_sdf->HasElement("min_distance")) {
    min_distance_ = _sdf->GetElement("min_distance")->Get<double>();
    if (min_distance_ < SENSOR_MIN_DISTANCE) {
      min_distance_ = SENSOR_MIN_DISTANCE;
    }
  } else {
    gzwarn << "[gazebo_lidar_plugin] Using default minimum distance: 0.3\n";
    min_distance_ = DEFAULT_MIN_DISTANCE;
  }


  #if GAZEBO_MAJOR_VERSION >= 7
    this->model_ =  this->world->GetModels()[1];
  #else
    this->model_ =  this->world->GetModels()[1];
  #endif

  #if GAZEBO_MAJOR_VERSION >= 7
    gzwarn << "\nImprime orientacion x " << this->model_->GetName() << "\n";
  #else
    gzwarn << "\nImprime orientacion x " << this->model_->GetName() << "\n";
  #endif


    this->link_ = this->model_->GetLink(_parent->ParentName());

  // get maximum distance
  if (_sdf->HasElement("max_distance")) {
    max_distance_ = _sdf->GetElement("max_distance")->Get<double>();
    if (max_distance_ > SENSOR_MAX_DISTANCE) {
      max_distance_ = SENSOR_MAX_DISTANCE;
    }
  } else {
    gzwarn << "[gazebo_lidar_plugin] Using default maximum distance: 15\n";
    max_distance_ = DEFAULT_MAX_DISTANCE;
  }

  min_distance_front = max_distance_;
  min_distance_left = max_distance_;
  min_distance_back = max_distance_;
  min_distance_right = max_distance_;

  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

#if GAZEBO_MAJOR_VERSION >= 7
  const string scopedName = _parent->ParentName();
#else
  const string scopedName = _parent->GetParentName();
#endif
  vector<string> names_splitted;
  boost::split(names_splitted,scopedName,boost::is_any_of("::"));
  string topicName = "~/" + names_splitted[0] + "/link/sweep";

  sweep_pub_ = node_handle_->Advertise<sweep_msgs::msgs::sweep>(topicName, 1);
}

/////////////////////////////////////////////////
void SweepPlugin::OnNewLaserScans()
{
  sweep_message.set_time_msec(0);
  sweep_message.set_min_distance(min_distance_);
  sweep_message.set_max_distance(max_distance_);

  double current_distance;

#if GAZEBO_MAJOR_VERSION >= 7
  current_distance = parentSensor_->Range(0);
#else
  current_distance = parentSensor_->GetRange(0);
#endif

  this->pose_ = this->link_->GetRelativePose();

  float degree = this->pose_.rot.GetYaw()*(180/M_PI);

  // Los angulos los recibimos como 180 grados positivos y 180 negativos, transformación
  /*
    if (degree < 0) {
    degree = 360.0 + degree;
  }
  */

  //std::cerr << "Angulo gazebo    " << degree << "\n";


  // set distance to min/max if actual value is smaller/bigger

  if (!std::isinf(current_distance) && current_distance > min_distance_) {

    if (current_distance > max_distance_) {
      current_distance = max_distance_;
    }

    // Front
    if (degree > -45 && degree < 45  ) {
      if (min_distance_front > current_distance) {
        min_distance_front = current_distance;
      }
    }

    // Left
    if (degree > 45 && degree < 135 ) {
      if (min_distance_left > current_distance) {
        min_distance_left = current_distance;
      }
    }

    // Back
    if (degree > 135 || degree < -135 ) {
      if (min_distance_back > current_distance) {
        min_distance_back = current_distance;
      }
    }

    // Right
    if (degree > -135 && degree < -45 ) {
      if (min_distance_right > current_distance) {
        min_distance_right = current_distance;
      }
    }
  }

  // Front
  if (degree > -45 && degree < 45  ) {
    min_distance_back = max_distance_;
    sweep_message.set_degrees(3);
    sweep_message.set_current_distance(min_distance_right);
    sweep_pub_->Publish(sweep_message);
  }

  // Left
  if (degree > 45 && degree < 135 ) {
    min_distance_right = max_distance_;
    sweep_message.set_degrees(0);
    sweep_message.set_current_distance(min_distance_front);
    sweep_pub_->Publish(sweep_message);
  }

  // Back
  if (degree > 135 || degree < -135 ) {
    min_distance_front = max_distance_;
    sweep_message.set_degrees(1);
    sweep_message.set_current_distance(min_distance_left);
    sweep_pub_->Publish(sweep_message);
  }

  // Right
  if (degree > -135 && degree < -45 ) {
    min_distance_left = max_distance_;
    sweep_message.set_degrees(2);
    sweep_message.set_current_distance(min_distance_back);
    sweep_pub_->Publish(sweep_message);
  }

  //sweep_message.set_current_distance(current_distance);

  //sweep_pub_->Publish(sweep_message);
}
