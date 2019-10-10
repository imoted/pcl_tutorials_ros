#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/project_inliers.h>


static ros::Publisher PubOutput;
static int ExampleNumber;

void tf_broadcast(const std::string frame_id){
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

void passThrough(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, const std::string frame_id){
    // PassThrough Filtering
    // Ref: http://www.pointclouds.org/documentation/tutorials/passthrough.php#passthrough

    // Container for original & filtered data
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2 cloud_filtered;
    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *cloud);


    // Create the filtering object
    pcl::PassThrough<pcl::PCLPointCloud2> pass;
    pass.setInputCloud (cloudPtr);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 1.0);
    // pass.setFilterLimitsNegative (true);
    pass.filter (cloud_filtered);


    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl_conversions::moveFromPCL(cloud_filtered, output);
    output.header.frame_id = frame_id;
    // Publish the data
    PubOutput.publish(output);
}

void downsampling(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, const std::string frame_id){
    // VoxelGrid filtering
    // Ref: http://www.pointclouds.org/documentation/tutorials/voxel_grid.php#voxelgrid

    // Container for original & filtered data
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2 cloud_filtered;
    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *cloud);


    // Perform the actual filtering
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloudPtr);
    sor.setLeafSize(0.1, 0.1, 0.1);
    sor.filter(cloud_filtered);


    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl_conversions::moveFromPCL(cloud_filtered, output);
    output.header.frame_id = frame_id;
    // Publish the data
    PubOutput.publish(output);
}


void statisticalOutlierRemoval(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, const std::string frame_id){
    // StatisticalOutlierRemoval filtering
    // Ref: http://www.pointclouds.org/documentation/tutorials/statistical_outlier.php#statistical-outlier-removal
    
    // Container for original & filtered data
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2 cloud_filtered;
    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *cloud);


    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloudPtr);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    sor.filter (cloud_filtered);


    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl_conversions::moveFromPCL(cloud_filtered, output);
    output.header.frame_id = frame_id;
    // Publish the data
    PubOutput.publish(output);
}

void projectInliers(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, const std::string frame_id){
    // Projecting
    // Ref: http://www.pointclouds.org/documentation/tutorials/project_inliers.php#project-inliers

    // Container for original & filtered data
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2 cloud_projected;
    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *cloud);


    // Create a set of planar coefficients with X=Y=0,Z=1
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    coefficients->values.resize (4);
    coefficients->values[0] = coefficients->values[1] = 0;
    coefficients->values[2] = 1.0;
    coefficients->values[3] = 0;

    // Create the filtering object
    pcl::ProjectInliers<pcl::PCLPointCloud2> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (cloudPtr);
    proj.setModelCoefficients (coefficients);
    proj.filter (cloud_projected);


    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl_conversions::moveFromPCL(cloud_projected, output);
    output.header.frame_id = frame_id;
    // Publish the data
    PubOutput.publish(output);
}

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    const static std::string EXAMPLE_FRAME_ID = "example_frame";

    switch(ExampleNumber){
    case 0:
        passThrough(cloud_msg, EXAMPLE_FRAME_ID);
        break;
    case 1:
        downsampling(cloud_msg, EXAMPLE_FRAME_ID);
        break;
    case 2:
        statisticalOutlierRemoval(cloud_msg, EXAMPLE_FRAME_ID);
        break;
    case 3:
        projectInliers(cloud_msg, EXAMPLE_FRAME_ID);
        break;
    default:
        break;
    }

    // to shift positions of rendering point clouds
    tf_broadcast(EXAMPLE_FRAME_ID);
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "example_filtering");
    ros::NodeHandle nh("~");

    nh.param<int>("number", ExampleNumber, 0);

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe("/camera/depth_registered/points", 1, cloud_cb);

    // Create a ROS publisher for the output point cloud
    PubOutput = nh.advertise<sensor_msgs::PointCloud2> ("/output", 1);

    ros::spin ();
}