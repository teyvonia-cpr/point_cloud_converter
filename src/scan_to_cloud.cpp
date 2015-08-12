#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"

using std::string;

class LaserScanToPointCloud
{

  
public:

  ros::NodeHandle n_;
  laser_geometry::LaserProjection projector_;

  tf::TransformListener listener_;
  ros::Subscriber  laser_sub_;
  // message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
  // tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;
  ros::Publisher scan_pub_;
  string world_frame_;


  LaserScanToPointCloud(ros::NodeHandle n) : 
    n_(n),
    listener_(ros::Duration(100))
    // laser_notifier_(laser_sub_, listener_, "map", 10)

  {
    string scan_beam_topic;
    string pcl_topic;
    ros::param::param<string>("~scan_beam_topic", scan_beam_topic,
    "/front/scan");
    ros::param::param<string>("~world_frame", world_frame_,
    "map");

    ROS_WARN_STREAM("Scan beam topic: " << scan_beam_topic);

    laser_sub_ = n_.subscribe<sensor_msgs::LaserScan>(scan_beam_topic, 1,
    &LaserScanToPointCloud::scanCallback, this);

    // tf::MessageFilter<sensor_msgs::LaserScan> temp_laser_notifier = new tf::MessageFilter<sensor_msgs::LaserScan>(laser_sub_, listener_, "map", 10);
    // laser_notifier_(laser_sub_, listener_, "map", 10);


    ros::param::param<string>("~pcl_topic", pcl_topic,
    "lidar_pcl");
    // laser_notifier_.registerCallback(
    //   boost::bind(&LaserScanToPointCloud::scanCallback, this, _1));
    // laser_notifier_.setTolerance(ros::Duration(0.01));
    scan_pub_ = n_.advertise<sensor_msgs::PointCloud>(pcl_topic,1);
  }

  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
  {
    // ROS_INFO("In scan callback..");
    sensor_msgs::PointCloud cloud;
    tf::StampedTransform transform;
    try{
      listener_.waitForTransform(world_frame_, scan_in->header.frame_id, scan_in->header.stamp, ros::Duration(4.0));
      // listener_.lookupTransform(world_frame_, scan_in->header.frame_id,  
      //                          scan_in->header.stamp, transform);

      // scan_in->header.stamp = ros::Time(0);

      /*
      ROS_INFO("\033[93m Time - Now: %.8f", ros::Time::now().toSec());
      ROS_INFO("\033[94m Time - Current: %.8f", ros::Time(0).toSec());
      ROS_INFO("\033[92m Time - Scan Start: %.8f", scan_in->header.stamp.toSec());
      ROS_INFO("\033[90m Scan Duration: %.8f", scan_in->ranges.size () * scan_in->time_increment);
      ros::Time end_time   = scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size () * scan_in->time_increment);
      ROS_INFO("\033[91m Time - Scan End: %.8f", end_time.toSec());
      ROS_INFO("\033[91m Time - Transform: %.8f \n", transform.stamp_.toSec());
      */

      // listener_.lookupTransform(world_frame_, scan_in->header.frame_id,  
      //                          scan_in->header.stamp, transform);
      projector_.transformLaserScanToPointCloud(world_frame_, *scan_in, cloud, listener_);
      ROS_DEBUG("\033[93m Scans transformed from Laser to Map Frame");
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      // ros::Duration(1.0).sleep();
    }


    // try
    // {
    //     projector_.transformLaserScanToPointCloud(
    //       world_frame_, *scan_in, cloud, listener_);
    // }
    // catch (tf::TransformException& e)
    // {
    //     std::cout << e.what();
    //     return;
    // }
    
    // Do something with cloud.

    scan_pub_.publish(cloud);

  }
};

int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "scan_to_cloud");
  ros::NodeHandle n;
  LaserScanToPointCloud scan_to_pcl(n);
  
  ros::spin();
  
  return 0;
}
