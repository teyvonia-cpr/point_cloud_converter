#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"

using std::string;

ros::NodeHandle nh;
std::string global_name, relative_name, default_param;
nh.getParam("/global_name", global_name);

class LaserScanToPointCloud{

protected:
  void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in);
  
public:

  ros::NodeHandle n_;
  laser_geometry::LaserProjection projector_;
  tf::TransformListener listener_;
  message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
  // tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;
  ros::Publisher scan_pub_;



  LaserScanToPointCloud(ros::NodeHandle n) : 
    n_(n)
    // laser_notifier_(laser_sub_, listener_, "map", 10)

  {
    string scan_beam_topic;
    string pcl_topic;
    ros::param::param<string>("~scan_beam_topic", scan_beam_topic,
    "scan_beam_1");
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

  void LaserScanToPointCloud::scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
  {
    sensor_msgs::PointCloud cloud;
    try
    {
        projector_.transformLaserScanToPointCloud(
          "map", *scan_in, cloud, listener_);
    }
    catch (tf::TransformException& e)
    {
        std::cout << e.what();
        return;
    }
    
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
