
#ifndef _ROTATE_PLUGIN_HH_
#define _ROTATE_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/SensorTypes.hh"
#include "gazebo/sensors/RaySensor.hh"

namespace gazebo
{
  /// \brief A plugin to control a Velodyne sensor.
  class RotateRay : public ModelPlugin
  {

    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      // Safety check
      if (_model->GetJointCount() == 0)
      {
        std::cerr << "Invalid joint count, Velodyne plugin not loaded\n";
        return;
      }

      double velocity = 0;

      if (_sdf->HasElement("velocity"))
        velocity = _sdf->Get<double>("velocity");

      this->model = _model;
      this->joint = _model->GetJoints()[5];
      this->pid = common::PID(0.1, 0, 0);
      this->model->GetJointController()->SetVelocityPID(this->joint->GetScopedName(), this->pid);
      this->model->GetJointController()->SetVelocityTarget(this->joint->GetScopedName(), velocity);

    }

    /// \brief Pointer to the model.
    private:  physics::ModelPtr model;
              physics::JointPtr joint;
              common::PID pid;

  };
  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(RotateRay)
}
#endif
