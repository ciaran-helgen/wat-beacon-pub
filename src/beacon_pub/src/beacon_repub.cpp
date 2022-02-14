#ifndef _BEACON_REPUB_PLUGIN_HH_
#define _BEACON_REPUB_PLUGIN_HH_

#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/msgs/msgs.hh>
#include <boost/algorithm/string/replace.hpp>
#include <boost/shared_ptr.hpp>


#include <ros/ros.h>

namespace gazebo
{
class BeaconRepub : public ModelPlugin
{

  private: physics::ModelPtr model;

  //A node used for transport
  private: transport::NodePtr node;

  // A subscriber to a named topic.
  private: transport::SubscriberPtr sub;

  public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
  {
    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    this->model = _parent;

    // Create the node
    this->node = transport::NodePtr(new transport::Node());
    
    this->node->Init();
    std::string nodename = this->model->GetWorld()->Name();
    ROS_INFO(nodename.c_str());
    // TODO: Get topic name using sdf or 'Name()/GetName()/GetLink()->Name() etc. 
    // Create a topic name
    std::string topicName = "/gazebo/default/wirelessReceiver/link/wirelessReceiver/transceiver";

    // Subscribe to the topic, and register a callback
    this->sub = this->node->Subscribe(topicName, &BeaconRepub::OnMsg, this);
  }

  public: void OnMsg(ConstWirelessNodesPtr &msg)
  {
    
    if(msg!=0)
    {
      // get signal level of zeroth wirelessnode in message 
      // wireless_nodes.proto: message consists of repeated 'node' messages
      // node(n) accesses the nth WirelessNode, whose data can be accessed
      // as normal (essid, signal_level, frequency)
      const double siglevel = msg->node(0).signal_level();
      std::cout << "Signal Level: " << siglevel << std::endl;
    }
    ROS_INFO("Callback Called!");
  }

};

GZ_REGISTER_MODEL_PLUGIN(BeaconRepub)
}
#endif