#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/segmentation/sac_segmentation.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

void tf_broadcast(const std::string frame_id)
{
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "camera_depth_optical_frame";
    transformStamped.child_frame_id = frame_id;
    transformStamped.transform.translation.x = 2.0;
    transformStamped.transform.translation.y = 0.0;
    transformStamped.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    br.sendTransform(transformStamped);
}


int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "test");
    ros::NodeHandle nh("~");

    // Create a ROS subscriber for the input point cloud
    // ros::Subscriber sub = nh.subscribe("/camera/depth_registered/points", 1, cloud_cb);

    // Create a ROS publisher for the output point cloud
    static ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("output_sample", 1);

    const static std::string EXAMPLE_FRAME_ID = "example_frame";
    ros::Rate rate(100);
    while (ros::ok())
    {
        pcl::PointCloud<pcl::PointXYZRGB> cloud;

        // Fill in the cloud data
        cloud.width = 150;
        cloud.height = 100;
        cloud.points.resize(cloud.width * cloud.height);

        // Generate the data
        size_t i;
        for (i = 0; i < cloud.points.size() / 2; ++i)
        {
            // 平面におく
            cloud.points[i].x = 1024 * rand() / (RAND_MAX + 1.0);
            cloud.points[i].y = 1024 * rand() / (RAND_MAX + 1.0);
            cloud.points[i].z = 0.1 * rand() / (RAND_MAX + 1.0);
            cloud.points[i].r = 255;
            cloud.points[i].g = 255;
            cloud.points[i].b = 255;
        }

        for (; i < cloud.points.size(); ++i)
        {
            // ちらばらせる
            cloud.points[i].x = 1024 * rand() / (RAND_MAX + 1.0);
            cloud.points[i].y = 1024 * rand() / (RAND_MAX + 1.0);
            cloud.points[i].z = 1024 * rand() / (RAND_MAX + 1.0);
            cloud.points[i].r = 255;
            cloud.points[i].g = 255;
            cloud.points[i].b = 255;
        }

        // Set a few outliers
        cloud.points[0].z = 2.0;
        cloud.points[3].z = -2.0;
        cloud.points[6].z = 4.0;

        // Publish the data
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(cloud, output);
        output.header.frame_id = EXAMPLE_FRAME_ID;

        pub.publish(output);

        tf_broadcast(EXAMPLE_FRAME_ID);
        rate.sleep();
    }
    return 0;
}
