#include <gazebo/common/Console.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Events.hh>

#include <string>

namespace gazebo
{
class MimicJointPlugin : public ModelPlugin
{
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    if (!model_)
    {
      gzerr << "[MimicJointPlugin] Model pointer is null.\n";
      return;
    }

    if (!sdf->HasElement("joint") || !sdf->HasElement("mimic_joint"))
    {
      gzerr << "[MimicJointPlugin] Missing <joint> or <mimic_joint> in SDF.\n";
      return;
    }

    joint_name_ = sdf->Get<std::string>("joint");
    mimic_joint_name_ = sdf->Get<std::string>("mimic_joint");

    if (sdf->HasElement("multiplier"))
    {
      multiplier_ = sdf->Get<double>("multiplier");
    }

    if (sdf->HasElement("offset"))
    {
      offset_ = sdf->Get<double>("offset");
    }

    if (sdf->HasElement("kp"))
    {
      kp_ = sdf->Get<double>("kp");
    }

    if (sdf->HasElement("kd"))
    {
      kd_ = sdf->Get<double>("kd");
    }

    joint_ = model_->GetJoint(joint_name_);
    mimic_joint_ = model_->GetJoint(mimic_joint_name_);

    if (!joint_)
    {
      gzerr << "[MimicJointPlugin] Joint not found: " << joint_name_ << "\n";
      return;
    }

    if (!mimic_joint_)
    {
      gzerr << "[MimicJointPlugin] Mimic joint not found: " << mimic_joint_name_ << "\n";
      return;
    }

    update_connection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&MimicJointPlugin::OnUpdate, this));
  }

private:
  void OnUpdate()
  {
    if (!joint_ || !mimic_joint_)
    {
      return;
    }

    const double source = mimic_joint_->Position(0);
    const double target = source * multiplier_ + offset_;
    const double current = joint_->Position(0);
    const double velocity = joint_->GetVelocity(0);
    const double error = target - current;
    const double torque = kp_ * error - kd_ * velocity;
    joint_->SetForce(0, torque);
  }

  physics::ModelPtr model_;
  physics::JointPtr joint_;
  physics::JointPtr mimic_joint_;
  event::ConnectionPtr update_connection_;

  std::string joint_name_;
  std::string mimic_joint_name_;
  double multiplier_{1.0};
  double offset_{0.0};
  double kp_{50.0};
  double kd_{1.0};
};

GZ_REGISTER_MODEL_PLUGIN(MimicJointPlugin)
}  // namespace gazebo
