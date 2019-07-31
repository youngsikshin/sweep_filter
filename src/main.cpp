#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <iostream>

using namespace std;

class SweepFilter
{
public:
    SweepFilter(ros::NodeHandle nh):
      nh_(nh)
    {
        cout << "SweepFilter" << endl;
        sweep_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/pc2",1,&SweepFilter::sweep_callback, this);
        scan_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/filtered_pc",1);
    }

    ~SweepFilter() {}

    void sweep_callback(const sensor_msgs::PointCloud2::ConstPtr& pc_in)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*pc_in, *pc_ptr);

        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(pc_ptr);

        pcl::PointCloud<pcl::PointXYZ>::Ptr pub_pc(new pcl::PointCloud<pcl::PointXYZ>);

        for(size_t i=0; i<pc_ptr->points.size(); ++i) {
            double radius = 0.5;
            std::vector<int> point_idx_radius_search;
            std::vector<float> point_radius_squared_distance;

            double dist = sqrt(pc_ptr->points[i].x*pc_ptr->points[i].x+pc_ptr->points[i].y*pc_ptr->points[i].y+pc_ptr->points[i].z*pc_ptr->points[i].z);
            double angle = atan2(pc_ptr->points[i].y,pc_ptr->points[i].x) * 180.0 / M_PI;

            if(dist > 1.8) {
                pub_pc->points.push_back(pc_ptr->points[i]);
                continue;
            } else if(fabs(angle) > 135.0) {
                continue;
            }
            else {

                if(kdtree.radiusSearch(pc_ptr->points[i], radius, point_idx_radius_search, point_radius_squared_distance) > 2)
                {
                    pub_pc->points.push_back(pc_ptr->points[i]);
                }

            }
        }

        sensor_msgs::PointCloud2 pc_msg;
        pcl::toROSMsg(*pub_pc, pc_msg);
//        pc_msg.header.frame_id = "/laser_frame";
        pc_msg.header = pc_in->header;
//        pc_msg.header.stamp.fromSec(last_lidar_element_->timestamp);
//        ros::Time sync = ros::Time::now();
//        pc_msg.header.stamp = sync;
        scan_pub_.publish(pc_msg);

    }

private:
    ros::NodeHandle nh_;
    ros::Publisher scan_pub_;
    ros::Subscriber sweep_sub_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sweep_filter");
    ros::NodeHandle nh;
    SweepFilter sweep(nh);

    ros::spin();

    return 0;
}
