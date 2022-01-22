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
#include <pcl/filters/extract_indices.h>

#include <pcl/segmentation/sac_segmentation.h>


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

    // 以下どっちのコードにするか？ ::Ptr で定義すると、ポインタ変数も定義されるので、 FIlterのInputがやりやすい。
    // pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
    // pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2);  // どうやらshared_ptrを使うと、メモリリークが起きない。

    // for (int i = 0; i < 100000; i++) {
    //     pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2);   //  for memory leak test  deleteがなくても、これで起きていない。
    // }

    // pcl::PCLPointCloud2* cloud_test = new pcl::PCLPointCloud2[10000]; // for memory leak test  ->やっぱりリークしている

    pcl::PCLPointCloud2 *cloud_filtered = new pcl::PCLPointCloud2;
    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *cloud); // 参照元の*cloud_msgがconstな為、moveToPCLできない

    // Create the filtering object
    pcl::PassThrough<pcl::PCLPointCloud2> pass;
    // pass.setInputCloud (cloudPtr);
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 2.0); // org 1.0
    // pass.setFilterLimitsNegative (true);
    pass.filter(*cloud_filtered);

    // Convert to ROS data type
    // sensor_msgs::PointCloud2 output;
    sensor_msgs::PointCloud2* output = new sensor_msgs::PointCloud2;
    pcl_conversions::moveFromPCL(*cloud_filtered, *output);
    output->header.frame_id = frame_id;
    // header.frame_id = frame_id;
    // Publish the data
    PubOutput.publish(*output);

    delete cloud_filtered;
    delete output;
    // delete [] cloud_test; // これでリークは防止できる
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
    sor.setLeafSize(0.01, 0.01, 0.01);  // org 0.1, 0.1, 0.1
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
    // pcl::StatisticalOutlierRemovalフィルタが作成される．各ポイントについて分析する近傍の数は50に設定され、標準偏差の倍率は1に設定されています。
    // これは、クエリ点との距離が平均距離の1標準偏差より大きい点はすべて外れ値としてマークされ、除去されるということである。
    sor.setMeanK (50); 
    sor.setStddevMulThresh (3.0);
    sor.filter (cloud_filtered);


    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl_conversions::moveFromPCL(cloud_filtered, output);
    output.header.frame_id = frame_id;
    // Publish the data
    PubOutput.publish(output);
}

// 正しい推定結果を導く良いマッチングはinliersと呼ばれ，それ以外のマッチングはoutliers(外れ値)と呼ばれます．
// 平面への投影
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
    // この場合、ax + by + cz + d = 0の平面モデルを使用します。ここで、a = b = d = 0、c = 1、つまりXY平面です。
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    coefficients->values.resize (4);
    // coefficients->values[0] = coefficients->values[1] = 0;
    // coefficients->values[2] = 1.0;
    // coefficients->values[3] = 0;
    coefficients->values[0] = 0;
    coefficients->values[1] = 1.0;
    // coefficients->values[2] = 1.0;
    coefficients->values[2] = 0;
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

// セグメンテーションアルゴリズムによって出力されたインデックスに基づいてポイントクラウドからポイントのサブセットを抽出する方法を学習します。
void extractIndices(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, const std::string frame_id){
    // Extracting indices from a PointCloud
    // Ref: http://www.pointclouds.org/documentation/tutorials/extract_indices.php#extract-indices

    // Container for original & filtered data
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *cloud);

    // ここでのデータダウンサンプリングの背後にある理論的根拠は、物事をスピードアップすることです。
    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::PCLPointCloud2::Ptr cloud_filtered_blob (new pcl::PCLPointCloud2 ());
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloudPtr);
    sor.setLeafSize(0.01, 0.01, 0.01);
    sor.filter(*cloud_filtered_blob);

    // Convert to the templated PointCloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>()); 
    pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);

    // パラメトリックセグメンテーションを扱います。
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    // PCLのアルゴリズムの多くはインデックスを返す。 インデックスは、クラウドのポイントのリストである（すべてのデータを含むポイント自体ではなく、クラウド内のインデックスのみ）。 例えば、シリンダ分割アルゴリズムは、シリンダモデルに合うと考えられた点のリストを出力として提供する。 
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);  // 平面モデルを使用します。
    seg.setMethodType (pcl::SAC_RANSAC);  // ロバスト推定量としてRANSAC法（pcl::SAC_RANSAC）を使用することにします。この決定の理由は，RANSACが単純であるためです（他のロバスト推定量では，これをベースとして，さらに複雑な概念を追加しています）
    // https://en.wikipedia.org/wiki/Random_sample_consensus
    // 基本的な前提は、データは「インライア」、すなわち、ノイズの影響を受けるかもしれないが、あるモデルパラメータのセットによって分布を説明できるデータと、モデルに適合しないデータである「アウトライア」から成るということである。

    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (0.01);

    // Create the filtering object 
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;  // インデックス抽出フィルター
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_extracted(new pcl::PointCloud<pcl::PointXYZRGB>()); 

    int i = 0, nr_points = (int) cloud_filtered->points.size ();
    // While 30% of the original cloud is still there
    // 複数のモデルを処理するために、ループで処理を実行し、各モデルが抽出された後、残りのポイントを得るために戻り、それを繰り返している。
    while (cloud_filtered->points.size () > 0.15 * nr_points)  // 元のsizeの0.15倍より大きい間は、ループを続ける。
    {
        // Segment the largest planar component from the remaining cloud
        // インライアは、以下のようにセグメンテーション処理で得られる。
        seg.setInputCloud (cloud_filtered);   // cloud_filteredをインプットして処理する
        seg.segment (*inliers, *coefficients);  // inliersをアウトプット  = 平面
        if (inliers->indices.size () == 0)
        {
            ROS_INFO("Could not estimate a planar model for the given dataset.");
            break;
        }

        // Extract the inliers
        extract.setInputCloud (cloud_filtered); // インプット
        extract.setIndices (inliers);           // inliersをフィルター上限
        extract.setNegative (true);             // 反転させた条件を適用する
        extract.filter (*cloud_extracted);      // cloud_extractedを出力  平面を除いたcloudとなる。
        cloud_filtered.swap (cloud_extracted);  // cloud_filteredとcloud_extractedを入れ替える。
        i++;
    }
    std::cout << "Number of planes found: " << i << std::endl;

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud_extracted, output);
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
    case 4:
        extractIndices(cloud_msg, EXAMPLE_FRAME_ID);
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
