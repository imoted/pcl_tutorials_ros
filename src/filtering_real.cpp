#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>

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

#include <pcl_ros/transforms.h>


static ros::Publisher PubOutput;
static ros::Publisher test_pub;
static ros::Publisher map_pub;
static int ExampleNumber;
static std::string input_frame_id;
static std::string output_frame_id;

tf2_ros::Buffer tfBuffer;

void tf_broadcast(const std::string frame_id){
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    // transformStamped.header.frame_id = "camera_depth_optical_frame";
    transformStamped.header.frame_id = input_frame_id;
    transformStamped.child_frame_id = frame_id;
    transformStamped.transform.translation.x = 0.0;
    transformStamped.transform.translation.y = 2.0;
    transformStamped.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    br.sendTransform(transformStamped);

    // geometry_msgs::TransformStamped transform_from_base_link;
    // try{
    //     // ros::Time now = ros::Time::now();

    //     // TODO : 以下では frameが読めない。
    //     // transform_from_base_link = tfBuffer.lookupTransform("camera_link", input_frame_id, ros::Time(0));
    //     transform_from_base_link = tfBuffer.lookupTransform("camera_link", "camera_diagonal_link", ros::Time(0));
    //     // transform_from_base_link = tfBuffer.lookupTransform("camera_diagonal_link", "camera_diagonal_color_frame", ros::Time(0));
    // }
    // catch (tf2::TransformException &ex) {
    //   ROS_WARN("%s",ex.what());
    // }
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
    // pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2);
    // pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2 *cloud_filtered = new pcl::PCLPointCloud2;
    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *cloud);

    // Perform the actual filtering
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.1, 0.1, 0.1);  // org 0.1, 0.1, 0.1
    sor.filter(*cloud_filtered);

    // Convert to ROS data type
    // sensor_msgs::PointCloud2 output;
    sensor_msgs::PointCloud2* output = new sensor_msgs::PointCloud2;
    pcl_conversions::moveFromPCL(*cloud_filtered, *output);
    output->header.frame_id = frame_id;
    // Publish the data
    PubOutput.publish(*output);

    delete cloud_filtered;
    delete output;
}


void statisticalOutlierRemoval(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, const std::string frame_id){
    // StatisticalOutlierRemoval filtering
    // Ref: http://www.pointclouds.org/documentation/tutorials/statistical_outlier.php#statistical-outlier-removal
    
    // Container for original & filtered data
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    // pcl::PCLPointCloud2 cloud_filtered;
    pcl::PCLPointCloud2 *cloud_filtered = new pcl::PCLPointCloud2;
    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *cloud);


    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloudPtr);
    // pcl::StatisticalOutlierRemovalフィルタが作成される．各ポイントについて分析する近傍の数は50に設定され、標準偏差の倍率は1に設定されています。
    // これは、クエリ点との距離が平均距離の1標準偏差より大きい点はすべて外れ値としてマークされ、除去されるということである。
    sor.setMeanK (50); 
    sor.setStddevMulThresh (1.0);
    sor.filter (*cloud_filtered);


    // Convert to ROS data type
    // sensor_msgs::PointCloud2 output;
    sensor_msgs::PointCloud2* output = new sensor_msgs::PointCloud2;
    pcl_conversions::moveFromPCL(*cloud_filtered, *output);
    output->header.frame_id = frame_id;
    // Publish the data
    PubOutput.publish(*output);
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

void tf_transform_and_merge(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, const std::string frame_id){
    pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2);
    pcl::PCLPointCloud2 *cloud_filtered = new pcl::PCLPointCloud2;
    pcl_conversions::toPCL(*cloud_msg, *cloud);

    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloud);
    double leaf_size = 0.01;
    sor.setLeafSize(leaf_size, leaf_size, leaf_size);  // org 0.1, 0.1, 0.1
    sor.filter(*cloud_filtered);

    sensor_msgs::PointCloud2* output = new sensor_msgs::PointCloud2;
    pcl_conversions::moveFromPCL(*cloud_filtered, *output);
    output->header.frame_id = frame_id;

    sensor_msgs::PointCloud2 global_frame_cloud;
    // このROS nodeでexample_frameのtf publishをしているが、それでは、example_frameを読むことができずに下記は落ちていた。
    // そのため、static_transform_publisherにて、今は、example_frameを出すことにしている。
    tfBuffer.transform(*output, global_frame_cloud, "camera_link");
    // PubOutput.publish(transformed_cloud);

    //  heap 領域で行うには、後から。　まずはスタック領域でやってみる。
    // sensor_msgs::PointCloud2 *obstacle_cloud = new sensor_msgs::PointCloud2;
    // *obstacle_cloud = global_frame_cloud;

    // 以下完全に同じアドレスとなる ダメ。
    // 0x7ffee1fa53c0   0x7ffee1fa53c0
    // sensor_msgs::PointCloud2& obstacle_cloud = global_frame_cloud;
    // std::cout << &obstacle_cloud << "   " << &global_frame_cloud << std::endl;

    // obstacle_cloud->height = global_frame_cloud.height;
    // obstacle_cloud->width = global_frame_cloud.width;
    // obstacle_cloud->fields = global_frame_cloud.fields;
    // obstacle_cloud->is_bigendian = global_frame_cloud.is_bigendian;
    // obstacle_cloud->point_step = global_frame_cloud.point_step;
    // obstacle_cloud->row_step = global_frame_cloud.row_step;
    // obstacle_cloud->is_dense = global_frame_cloud.is_dense;
    // obstacle_cloud->header.frame_id = global_frame_cloud.header.frame_id;
    // obstacle_cloud->header.stamp = global_frame_cloud.header.stamp;
    // obstacle_cloud->data = global_frame_cloud.data;

    // まずはスタック領域でやってみる。
    sensor_msgs::PointCloud2 obstacle_cloud = global_frame_cloud;
    // 両方共 camera_link
    // std::cout << obstacle_cloud.header.frame_id << "    " << global_frame_cloud.header.frame_id << std::endl;

    // unsigned int cloud_size = global_frame_cloud.height*global_frame_cloud.width;
    sensor_msgs::PointCloud2Modifier modifier(obstacle_cloud);
    // modifier.resize(cloud_size);  // The number of T's to change the size of the original sensor_msgs::PointCloud2 by
    unsigned int point_count = 0;

    // copy over the points that are within our height bounds
    float max_obstacle_height_ = 10;
    float min_obstacle_height_ = 0.005;
    // sensor_msgs::PointCloud2Iterator<float> iter_z(global_frame_cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_x(global_frame_cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_z_obs(obstacle_cloud, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_rgb(obstacle_cloud, "rgb");
    // for (auto i = 0; i < 10; ++i, ++iter_rgb){
    //     std::cout << "iter_rgb_obs address: " << &*iter_rgb << std::endl;
    // }

    nav_msgs::OccupancyGrid obstacle_map;
    obstacle_map.header.frame_id = "map_link";
    obstacle_map.header.stamp = ros::Time::now();
    obstacle_map.info.width = 100;
    obstacle_map.info.height = 100;
    obstacle_map.info.resolution = leaf_size;
    for (uint64_t i = 0; i < obstacle_map.info.width * obstacle_map.info.height; ++i){
        obstacle_map.data.push_back(0);
        // std::cout << obstacle_map.data[i] << std::endl;
    }

    std::vector<unsigned char>::const_iterator iter_global = global_frame_cloud.data.begin(), iter_global_end = global_frame_cloud.data.end();
    // std::vector<unsigned char>::iterator iter_obs = observation_cloud.data.begin();
    std::vector<unsigned char>::iterator iter_obstacle = obstacle_cloud.data.begin();
    for (; iter_global != iter_global_end; ++iter_x, ++iter_z_obs, iter_global += global_frame_cloud.point_step)
    { 
    // std::cout << "iter_z address: " << &*iter_z << std::endl;
    // std::cout << "iter_z_obs address: " << &*iter_z_obs << std::endl;
    // std::cout << "iter_rgb_obs address: " << &*iter_rgb << std::endl;   // このアドレス表示はうまくいかがないが、問題無さそう。
    //   std::cout << "iter_z: " << (float)*iter_z << std::endl;
      if (*(iter_x + 2) <= max_obstacle_height_ && *(iter_x + 2) >= min_obstacle_height_)
      {
        // std::cout << "iter_z: " << (float)*iter_z_obs << std::endl;
        // iter_globalからiter_global + global_frame_cloud.point_step　までを iter_obsからのアドレスにコピー
        std::copy(iter_global, iter_global + global_frame_cloud.point_step, iter_obstacle);

        // iter_z_obs[0] = 0;  // カメラに近い領域のものだけは、ある一定の高さの物は、 zが０に変換された。

        // マイナス方向にはデータは無いようだ。
        // if(iter_x[0] < 0){
        //     std::cout << "iter_x: " << (float)*iter_x << std::endl;
        // }
        // if(iter_x[1] < 0){
        //     std::cout << "iter_y: " << (float)*(iter_x + 1) << std::endl;
        // }

        // std::cout << "axis x : " << iter_x[0] << "   axis y :" << iter_x[1] << std::endl;

        // 以下は上手く言ってない気がする。
        // iter_rgb[0] = 0;
        // iter_rgb[1] = 0;
        // iter_rgb[2] = 0;
        // std::cout << "iter_z: " << (float)*iter_z_obs << std::endl;
        iter_obstacle += global_frame_cloud.point_step;
        ++point_count;
      }
    }

    // for DEBUG
    // float max_obstacle_height_ = 1;
    // float min_obstacle_height_ = 0.1;
    // sensor_msgs::PointCloud2Iterator<float> iter_z(global_frame_cloud, "z");
    // sensor_msgs::PointCloud2Iterator<uint8_t> iter_rgb(global_frame_cloud, "rgb");
    // std::vector<unsigned char>::const_iterator iter_global = global_frame_cloud.data.begin(), iter_global_end = global_frame_cloud.data.end();
    // for (; iter_global != iter_global_end; ++iter_z, ++iter_rgb, iter_global += global_frame_cloud.point_step)
    // {
    // // std::cout << "iter_z address: " << &*iter_z << std::endl;
    // // std::cout << "iter_rgb_obs address: " << &*iter_rgb << std::endl;   // こちらもアドレスはおかしいが、データには下記でアクセスはしっかりできている。。。
    //   if ((*iter_z) <= max_obstacle_height_ && (*iter_z) >= min_obstacle_height_)
    //   {
    //     // 以下 OK
    //     iter_z[0] = 2;

    //     // 以下OK
    //     // iter_rgb[0] = 0;
    //     // iter_rgb[1] = 0;
    //     // iter_rgb[2] = 0;

    //     // iter_obstacle += global_frame_cloud.point_step;
    //     // ++point_count;
    //   }
    // }



    // for DEBUG
    // std::cout << "global_frame_cloud point_step" << global_frame_cloud.point_step << std::endl;
    // std::cout << "global_frame_cloud row_step" << global_frame_cloud.row_step << std::endl;
    // std::cout << "global_frame_cloud height" << global_frame_cloud.height << std::endl;
    // std::cout << "global_frame_cloud width" << global_frame_cloud.width << std::endl;
    // std::cout << "global_frame_cloud data size" << global_frame_cloud.data.size() << std::endl;
    // std::cout << "global_frame_cloud data" << global_frame_cloud.data[0] << std::endl;

    // std::cout << "obstacle point_step" << obstacle_cloud.point_step << std::endl;
    // std::cout << "obstacle row_step" << obstacle_cloud.row_step << std::endl;
    // std::cout << "obstacle height" << obstacle_cloud.height << std::endl;
    // std::cout << "obstacle width" << obstacle_cloud.width << std::endl;
    // std::cout << "obstacle data size" << obstacle_cloud.data.size() << std::endl;
    // std::cout << "obstacle data" << obstacle_cloud.data[0] << std::endl;

    // observation_cloud.header.stamp = cloud.header.stamp;
    // observation_cloud.header.frame_id = global_frame_cloud.header.frame_id;
    // PubOutput.publish(observation_cloud);

    modifier.resize(point_count);
    PubOutput.publish(obstacle_cloud);
    // PubOutput.publish(global_frame_cloud);

    map_pub.publish(obstacle_map);

    delete cloud_filtered;
    delete output;

    // TODO : ld errorが起きる。  
    // ld errorは解消。includeが足りなかった。
    // TODO2 : "moved_link"が見えなくて、落ちる。
    // 原因は単純　tfListner  を、毎回callbackのたびに宣言していては、tfを読めない。

    // TODO 入力の型が違うらしい
    // sensor_msgs::PointCloud2 transformed_cloud;
    // pcl_ros::transformPointCloud(frame_id, output, transformed_cloud, tfListener);
    // PubOutput.publish(transformed_cloud);

    // TODO こちらもエラー transform のエラー
    // geometry_msgs::PointStamped global_origin;
    // geometry_msgs::PointStamped local_origin;
    // local_origin.header.stamp = ros::Time::now();
    // local_origin.header.frame_id = "camera_link";
    // local_origin.point.x = 0;
    // local_origin.point.y = 0;
    // local_origin.point.z = 0;
    // tfBuffer.transform(local_origin, global_origin, frame_id);
    // test_pub.publish(global_origin);

}


void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    const static std::string EXAMPLE_FRAME_ID = "example_frame";

    switch(ExampleNumber){
    case 0:
        passThrough(cloud_msg, output_frame_id);
        break;
    case 1:
        downsampling(cloud_msg, output_frame_id);
        break;
    case 2:
        statisticalOutlierRemoval(cloud_msg, output_frame_id);
        break;
    case 3:
        projectInliers(cloud_msg, output_frame_id);
        break;
    case 4:
        extractIndices(cloud_msg, output_frame_id);
        break;
    case 5:
        tf_transform_and_merge(cloud_msg, output_frame_id);
        break;
    default:
        break;
    }

    // to shift positions of rendering point clouds
    // tf_broadcast(output_frame_id);
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "example_filtering");
    ros::NodeHandle nh("~");

    nh.param<int>("number", ExampleNumber, 0);
    nh.param<std::string>("input_frame", input_frame_id, "camera_diagonal_depth_optical_frame");
    nh.param<std::string>("output_frame", output_frame_id, "example_frame");

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe("/camera/depth_registered/points", 1, cloud_cb);

    // Create a ROS publisher for the output point cloud
    PubOutput = nh.advertise<sensor_msgs::PointCloud2> ("/output", 1);
    test_pub = nh.advertise<geometry_msgs::PointStamped> ("/test", 1);
    map_pub = nh.advertise<nav_msgs::OccupancyGrid> ("/obstacle_map", 1);
    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::spin ();
}