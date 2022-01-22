#include <ros/ros.h>
#include <vector>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

// グローバル変数宣言に static 修飾子を付けると、その識別子がそれを記述したファイルでのみ有効になる
// ローカル変数に static を付けた効果は「スコープを抜けても変数の値がそのまま保持される」
static ros::Publisher PubOutput;
static int ExampleNumber;
static float cluster_tolerance;

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

void plane(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, const std::string frame_id){
    // PassThrough Filtering
    // https://pcl.readthedocs.io/projects/tutorials/en/master/planar_segmentation.html#planar-segmentation

    // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    // Use pcl::PointXYZRGB to visualize segmentation.
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::fromROSMsg(*cloud_msg, cloud);

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);
    
    seg.setInputCloud (cloud.makeShared());  // shared_ptr オブジェクトを構築する。
    seg.segment (*inliers, *coefficients);   // インライアを出力

    if (inliers->indices.size () == 0)
    {
        ROS_ERROR("Could not estimate a planar model for the given dataset.");
        exit(-1);
    }

    for (size_t i = 0; i < inliers->indices.size (); ++i){
        cloud.points[inliers->indices[i]].r = 0;  // cloudのinliersのindex listで示された配列の変数を255に変更
        cloud.points[inliers->indices[i]].g = 255;
        cloud.points[inliers->indices[i]].b = 0;
    }

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud, output);
    output.header.frame_id = frame_id;
    // // Publish the data
    PubOutput.publish(output);
}

void euclideanClusterExtraction(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, const std::string frame_id){
    // Euclidean Cluster Extraction  ユークリッドクラスター抽出
    // クラスタリング手法は、未整理の点群モデルをより小さなパーツに分割し、全体の処理時間を大幅に短縮する必要がある。
    // ユークリッド的な意味での簡単なデータクラスタリングは、固定幅のボックスを用いた空間の3次元グリッド細分化、
    // より一般的にはオクツリーデータ構造を利用することで実現できる。
    // https://ja.wikipedia.org/wiki/%E5%85%AB%E5%88%86%E6%9C%A8
    // この特殊な表現は非常に高速に構築でき、占有空間の容積表現が必要な場合や、結果として得られる各3Dボックス（またはオクトリーリーフ）
    // のデータが別の構造で近似できる場合に有用である。

    // しかし、より一般的な意味では、最近傍を利用し、本質的にフラッドフィル(塗りつぶし)アルゴリズムに類似したクラスタリング手法を実装することができる。

    // 表とその上にオブジェクトが乗っている点群が与えられたとしよう。
    // 我々は、平面上に横たわる個々のオブジェクトの点群を見つけて、セグメント化したい。
    // 近傍探索にKd-tree構造を使用すると仮定すると、そのためのアルゴリズムステップは次のようになります（[RusuDissertation]より）。

    // Ref: http://www.pointclouds.org/documentation/tutorials/cluster_extraction.php#cluster-extraction
    
    enum COLOR_RGB{  // 列挙型の変数 GREEN = 1, BLUE = 2, RED = 3
        RED=0,
        GREEN,
        BLUE,
        COLOR_MAX
    };
    const int CLUSTER_MAX = 10;
    const int CLUSTER_COLOR[CLUSTER_MAX][COLOR_MAX] = {
        {230, 0, 18},{243, 152, 18}, {255, 251, 0},
        {143, 195, 31},{0, 153, 68}, {0, 158, 150},
        {0, 160, 233},{0, 104, 183}, {29, 32, 136},
        {146, 7, 131}
    };

    // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    // Use pcl::PointXYZRGB to visualize segmentation.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>()); 
    pcl::fromROSMsg(*cloud_msg, *cloud);

    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>());
    vg.setInputCloud (cloud);
    vg.setLeafSize (0.01, 0.01, 0.01);
    vg.filter (*cloud_filtered);

    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ());
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.02);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_extracted(new pcl::PointCloud<pcl::PointXYZRGB>()); 
    int i=0, nr_points = (int) cloud_filtered->points.size ();
    while (cloud_filtered->points.size () > 0.3 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud_filtered);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
            ROS_WARN("Could not estimate a planar model for the given dataset.");
            break;
        }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud (cloud_filtered);
        extract.setIndices (inliers);

        // Remove the planar inliers, extract the rest
        extract.setNegative (true);
        extract.filter (*cloud_extracted);
        *cloud_filtered = *cloud_extracted;
    }

    // Creating the KdTree object for the search method of the extraction
    // ここでは、抽出アルゴリズムの検索方法として、KdTreeオブジェクトを作成しています。
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>());
    tree->setInputCloud (cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    // ここでは、実際のインデックス情報をvector<int>で表したPointIndicesのベクトルを作成しています。検出された各クラスタのインデックスがここに保存される。
    // cluster_indices は、検出された各クラスタの PointIndices のインスタンスを 1 つ含むベクトルであるという事実に注意してほしい。
    // つまり、cluster_indices[0]には、点群における最初のクラスタのすべてのインデックスが格納されます。
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance (cluster_tolerance); // 2cm
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_filtered);
    ec.extract (cluster_indices);
    // ここでは、点群がPointXYZ型であるため、PointXYZ型のEuclideanClusterExtractionオブジェクトを作成しています。
    // また、抽出のためのパラメータと変数を設定しています。
    // setClusterTolerance()に正しい値を設定することに注意してください。
    // あまり小さい値をとると、実際のオブジェクトが複数のクラスタとして見えてしまうことがあります。
    // 一方、値を大きくしすぎると、複数のオブジェクトがひとつのクラスタとして見えてしまうことがあります。
    // そのため、どの値が自分のデータセットに合っているかをテストして試してみることをお勧めします。

    // 発見されたクラスタは少なくともsetMinClusterSize()ポイント、最大setMaxClusterSize()ポイントを持たなければならないと課している。

    // ここで、点群からクラスターを抽出し、そのインデックスをcluster_indicesに保存しました。
    // vector<PointIndices>から各クラスタを分離するには、cluster_indicesを繰り返し、
    // エントリごとに新しいPointCloudを作成し、現在のクラスタのすべてのポイントをPointCloudに書き込む必要があります。

    int cluster_i=0;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_output (new pcl::PointCloud<pcl::PointXYZRGB>());
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); 
            it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>());
        for (std::vector<int>::const_iterator pit = it->indices.begin (); 
                pit != it->indices.end (); ++pit){
            cloud_filtered->points[*pit].r = CLUSTER_COLOR[cluster_i][RED];
            cloud_filtered->points[*pit].g = CLUSTER_COLOR[cluster_i][GREEN];
            cloud_filtered->points[*pit].b = CLUSTER_COLOR[cluster_i][BLUE];
            cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
        }
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        
        *cloud_output += *cloud_cluster;

        cluster_i++;
        if(cluster_i >= CLUSTER_MAX){
            break;
        }
    }

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud_output, output);
    // pcl_conversions::moveFromPCL(cloud_extracted, output);
    output.header.frame_id = frame_id;
    // Publish the data
    PubOutput.publish(output);
}


void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    const static std::string EXAMPLE_FRAME_ID = "example_frame";

    switch(ExampleNumber){
    case 0:
        plane(cloud_msg, EXAMPLE_FRAME_ID);
        break;
    case 1:
        break;
    case 2:
        euclideanClusterExtraction(cloud_msg, EXAMPLE_FRAME_ID);
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
    ros::init (argc, argv, "example_segmentation");
    ros::NodeHandle nh("~");

    nh.param<int>("number", ExampleNumber, 0);
    nh.param<float>("cluster_tolerance", cluster_tolerance, 0.02);

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe("/camera/depth_registered/points", 1, cloud_cb);

    // Create a ROS publisher for the output point cloud
    PubOutput = nh.advertise<sensor_msgs::PointCloud2> ("/output", 1);

    ros::spin ();
}
