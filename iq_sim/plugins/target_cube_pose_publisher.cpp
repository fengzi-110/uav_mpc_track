#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/common/Events.hh>

namespace gazebo
{
  class TargetCubePosePublisher : public ModelPlugin
  {
  public:
    void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
    {
      this->model = _model;

      // init ROS node
      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "target_cube_pose_publisher",
                  ros::init_options::NoSigintHandler);
      }
      this->rosNode.reset(new ros::NodeHandle("~"));

      // create publisher
      this->posePub = this->rosNode->advertise<geometry_msgs::PoseStamped>("/target_cube/pose", 10);

      // Connect to each simulation cycle callback
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&TargetCubePosePublisher::OnUpdate, this));
    }

    void OnUpdate()
    {
      // Get the current simulation time
      common::Time simTime = this->model->GetWorld()->SimTime();

      // Get the model location
      ignition::math::Pose3d pose = this->model->WorldPose();

      geometry_msgs::PoseStamped msg;
      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = "world";
      msg.pose.position.x = pose.Pos().X();
      msg.pose.position.y = pose.Pos().Y();
      msg.pose.position.z = pose.Pos().Z();
      msg.pose.orientation.x = pose.Rot().X();
      msg.pose.orientation.y = pose.Rot().Y();
      msg.pose.orientation.z = pose.Rot().Z();
      msg.pose.orientation.w = pose.Rot().W();

      this->posePub.publish(msg);
    }

  private:
    physics::ModelPtr model;
    std::unique_ptr<ros::NodeHandle> rosNode;
    ros::Publisher posePub;
    event::ConnectionPtr updateConnection;
  };

  // Plugin Registration
  GZ_REGISTER_MODEL_PLUGIN(TargetCubePosePublisher)
}

