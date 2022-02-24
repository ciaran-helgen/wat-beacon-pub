#ifndef _BEACON_REPUB_PLUGIN_HH_
#define _BEACON_REPUB_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/msgs/msgs.hh>
#include <boost/shared_ptr.hpp>
#include <string>

#include <ros/ros.h>
#include <beacon_pub/beacon.h>

namespace gazebo
{
class BeaconRepub : public SensorPlugin
{

  // Pointer to sensor instance
  private: sensors::WirelessReceiverPtr receiverSensor;

  // Pointer to corresponding transmitter
  private: sensors::WirelessTransmitterPtr transmitterSensor;

  // Pointer to SDF element
  private: sdf::ElementPtr sdf;

  // World the sensor is in
  private: physics::WorldPtr world;

  //A node used for transport
  private: transport::NodePtr node;

  // The parents of the transmitter and receiver. Used for pose calculation
  private: physics::EntityPtr transmitterParent;
  private: physics::EntityPtr receiverParent;

  // Subscriber to gazebo topic (WirelessNodes message)
  private: transport::SubscriberPtr sub;

  // Subscriber to get sim time
  private: transport::SubscriberPtr gz_clock_sub;

  // Incrementing sequence number for ROS msg header
  private: unsigned int sequence_ctr = 0;

  //variables to store simulation time
  private: unsigned int gz_sec;
  private: unsigned int gz_nsec;

  //ROS topic to publish to
  private: std::string topic_name;
  //ROS Frame for beacon msg header
  private: std::string frame_name;

  //ROS node for publisher
  ros::NodeHandle n;

  //ROS publisher
  ros::Publisher pub;


  public: void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
  {
    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    // Convert the base sensor pointer to a WirelessReceiver pointer
    this->receiverSensor = std::dynamic_pointer_cast<sensors::WirelessReceiver>(_sensor);

    this->sdf = _sdf;

    this->world = physics::get_world(this->receiverSensor->WorldName());

    // Create the node
    this->node = transport::NodePtr(new transport::Node());
    
    this->node->Init();
   
    // Get the frame of the ROS message from the frame_name tag in the plugin, if it exists
    if (!this->sdf->HasElement("frame_name"))
    {
      ROS_INFO("BeaconRepub plugin missing <frameName>, defaults to /world");
      this->frame_name = "/world";
    }
    else
    {
      this->frame_name = this->sdf->Get<std::string>("frame_name");
    }
  
    // Get the topic of the ROS message from the topic_name tag in the plugin, if it exists
    if (!this->sdf->HasElement("topic_name"))
    {
      ROS_INFO("BeaconRepub plugin missing <topicName>, defaults to /receiver");
      this->topic_name = "/receiver";
    }
    else
    {
      this->topic_name = this->sdf->Get<std::string>("topic_name");
    }

    // Parse all wireless transmitters in the world
    // The number of transmitters detected in channel. Ensure this == 1
    unsigned int tx_count = 0;
    sensors::Sensor_V sensors = sensors::SensorManager::Instance()->GetSensors();
    for (sensors::Sensor_V::iterator it = sensors.begin(); it != sensors.end(); ++it)
    {
      ROS_INFO("Sensor Found: %s", (*it)->Name().c_str());
      if ((*it)->Type() == "wireless_transmitter")
      {
        std::shared_ptr<gazebo::sensors::WirelessTransmitter> transmitter =
            std::static_pointer_cast<sensors::WirelessTransmitter>(*it);
        // Find a transmitter that's transmitting in the correct channel
        if ((transmitter->Freq() >= this->receiverSensor->MinFreqFiltered()) &&
                  (transmitter->Freq() <= this->receiverSensor->MaxFreqFiltered()) )
        {
          tx_count ++;
          std::cout << "Transmitter found on " << transmitter->Freq() << " MHz" <<std::endl;
          this->transmitterSensor = transmitter;
        }
        else
        { 
          std::cout << "No transmitter found in range " << this->receiverSensor->MinFreqFiltered() << 
            " - " << this->receiverSensor->MaxFreqFiltered() << " MHz" << std::endl;
        }
      }
    }
    if (tx_count > 1)
    {
      ROS_INFO("Multiple transmitters (%i) found in channel. Modify tx and/or rx frequencies!", tx_count);
    }
    // Debug output when no transmitter is found in the right frequency
    if (this->transmitterSensor == 0)
    {
      if(this->sdf->GetParent()->GetParent() != 0)
      {
        ROS_INFO("0 No transmitter found in %s", this->sdf->GetParent()->GetParent()->GetAttribute("name")->GetAsString().c_str());
      }
      else
      {
        ROS_INFO("1 No transmitter found in %s", this->sdf->GetParent()->GetAttribute("name")->GetAsString().c_str());
      }
    }
    // Only runs if a valid transmitter is found
    else
    {
      // Get the parent of the transmitter. Used to calculate pose in the global frame
      std::string transmitterParentName = this->transmitterSensor->ParentName();
      this->transmitterParent = this->world->EntityByName(transmitterParentName);

      // Get the parent of the receiver. Used to calculate pose in the global frame
      std::string receiverParentName = this->receiverSensor->ParentName();
      this->receiverParent = this->world->EntityByName(receiverParentName);



      // Gets the topic of the sensor programmatically using the Topic() method
      // Subscribe to the topic, and register a callback
      this->sub = this->node->Subscribe(this->receiverSensor->Topic(), 
                                          &BeaconRepub::BeaconMsgCB, this);

      // subscribe to /world_stats to access sim_time
      this->gz_clock_sub = this->node->Subscribe("~/world_stats", 
                                                    &BeaconRepub::ClockCB, this);

      //Begin publisher for ROS message
      this->pub = n.advertise<beacon_pub::beacon>(this->topic_name, 10);
    }

  }

  // The callback for the gazebo WirelessNodes message
  public: void BeaconMsgCB(ConstWirelessNodesPtr &gmsg)
  {
    
    if(gmsg!=0)
    {
      // simple check to ensure only one tx-rx pair per channel
      if(gmsg->node_size()>1)
      {
        ROS_INFO("Multiple messages received on same receiver, adjust tx and rx frequencies");
      }
      else
      {

        //calculate pose of corresponding transmitter
        ignition::math::Pose3d transmitterPose = this->transmitterSensor->Pose() + this->transmitterParent->WorldPose();
        //calculate the pose of the receiver
        ignition::math::Pose3d receiverPose = this->receiverSensor->Pose() + this->receiverParent->WorldPose();

        // get the distance between the transmitter and receiver
        // Tx and rx pose coordinates
        double beacon_dist;
        beacon_dist = transmitterPose.Pos().Distance(receiverPose.Pos());
        // std::cout << "Distance between beacons: " << tx_x << std::endl;

        // Simulate time delay using sped of light in air
        // C_Air = C_Vac / n_A
        // n_A is the refractive index of air; 1.0003

        const double C_air = gazebo::common::SpeedOfLight/1.0003;

        // Propagation delay of light in air
        // prop_delay = C_Air * distance

        double prop_delay = beacon_dist/C_air;

        std::cout << "Propagation delay: " << prop_delay << std::endl;

        //ROS beacon message
        beacon_pub::beacon rosmsg;

        // Variables to store message data
        // Get data from of zeroth wirelessnode in message.
        // wireless_nodes.proto message consists of repeated 'node' messages.
        // node(n) accesses the nth WirelessNode, whose data can be accessed
        // as normal (essid, signal_level, frequency)
        double gz_signal_level = gmsg->node(0).signal_level();
        double gz_frequency = gmsg->node(0).frequency();
        std::string gz_essid = gmsg->node(0).essid();

        ros::Time tmp_stamp;
        tmp_stamp.sec = this->gz_sec;
        tmp_stamp.nsec = this->gz_nsec;

        // Build the ROS message
        // Header
        rosmsg.header.seq = this->sequence_ctr;
        rosmsg.header.stamp.sec = tmp_stamp.sec;
        rosmsg.header.stamp.nsec = tmp_stamp.nsec;

        //convert timestamp to nanoseconds and subtract delay to get transmit time
        uint64_t tmp_nsec = tmp_stamp.toNSec() - prop_delay * 1000000000;

        // convert nanoseconds back to timestamp format
        tmp_stamp.fromNSec(tmp_nsec);

        // add transmit time to message
        rosmsg.transmit_time.sec = tmp_stamp.sec;
        rosmsg.transmit_time.nsec = tmp_stamp.nsec;
        
        // Add ground truth distance to message
        rosmsg.debug_distance = beacon_dist;

        rosmsg.header.frame_id = this->frame_name;

        // beacon params
        rosmsg.signal_level = gz_signal_level;
        rosmsg.essid = gz_essid;
        rosmsg.frequency = gz_frequency;

        this->pub.publish(rosmsg);
        ros::spinOnce();

        this->sequence_ctr ++;
        //ROS_INFO("Callback Called! WirelessNodes Message Received");
        //std::cout << "rosmsg: " << rosmsg.header.seq << std::endl;
      }
      
      
    }
    //ROS_INFO("Callback Called! No WirelessNodes msg");
 
  }

  // The callback for the WorldStatistics message (sim_time)
  public: void ClockCB(ConstWorldStatisticsPtr &gmsg)
  {
    if(gmsg!=0)
    {
      this->gz_sec = gmsg->sim_time().sec();
      this->gz_nsec = gmsg->sim_time().nsec();
    }
  }

};

GZ_REGISTER_SENSOR_PLUGIN(BeaconRepub)
}
#endif