#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using std::string;
using std::vector;

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

  message_filters::Subscriber<sensor_msgs::LaserScan> laser1_sub;
  message_filters::Subscriber<sensor_msgs::LaserScan> laser2_sub;
  message_filters::Subscriber<sensor_msgs::LaserScan> laser3_sub;
  message_filters::Subscriber<sensor_msgs::LaserScan> laser4_sub;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, 
                                                        sensor_msgs::LaserScan, 
                                                        sensor_msgs::LaserScan, 
                                                        sensor_msgs::LaserScan> LaserScanSyncPolicy;

  LaserScanToPointCloud(ros::NodeHandle n) : 
    n_(n),
    laser1_sub(n_, "scan_beam_1", 10),
    laser2_sub(n_, "scan_beam_2", 10),
    laser3_sub(n_, "scan_beam_3", 10),
    laser4_sub(n_, "scan_beam_4", 10)
  {
    string pcl_topic;
    ros::param::param<string>("~world_frame", world_frame_,
    "map");

    // laser_sub_ = n_.subscribe<sensor_msgs::LaserScan>(scan_beam_topic, 1,
    // &LaserScanToPointCloud::scanCallback, this);

    message_filters::Synchronizer<LaserScanSyncPolicy> sync(LaserScanSyncPolicy(10), laser1_sub, laser2_sub, laser3_sub, laser4_sub);
    sync.registerCallback(boost::bind(&LaserScanToPointCloud::scanCallback, this, _1, _2, _3, _4));



    ros::param::param<string>("~pcl_topic", pcl_topic,
    "lidar_pcl");
    scan_pub_ = n_.advertise<sensor_msgs::PointCloud>(pcl_topic,1);
  }

  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan1, const sensor_msgs::LaserScan::ConstPtr& scan2, 
    const sensor_msgs::LaserScan::ConstPtr& scan3, const sensor_msgs::LaserScan::ConstPtr& scan4)
  {
    sensor_msgs::PointCloud cloud1, cloud2, cloud3, cloud4;
    try
    {
        projector_.transformLaserScanToPointCloud(
          world_frame_, *scan1, cloud1, listener_);
        projector_.transformLaserScanToPointCloud(
          world_frame_, *scan2, cloud2, listener_);
        projector_.transformLaserScanToPointCloud(
          world_frame_, *scan3, cloud3, listener_);
        projector_.transformLaserScanToPointCloud(
          world_frame_, *scan4, cloud4, listener_);
    }
    catch (tf::TransformException& e)
    {
        std::cout << e.what();
        return;
    }
   
    sensor_msgs::PointCloud stiched_cloud;
    stiched_cloud.header = cloud1.header;

    vector<sensor_msgs::PointCloud> clouds(4);
    clouds[0] = cloud1;
    clouds[1] = cloud2;
    clouds[2] = cloud3;
    clouds[3] = cloud4;

    for(int i=0; i<4; i++)
    {
      for(int j=0; j<clouds[i].points.size(); j++)
      {
        stiched_cloud.points.push_back(clouds[i].points[j]);
      }
    }
    // Do something with cloud.

    scan_pub_.publish(stiched_cloud);

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
