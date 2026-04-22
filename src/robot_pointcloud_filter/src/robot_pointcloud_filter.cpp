#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/filter.h>

#include <urdf/model.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shapes.h>

#include <tf/transform_listener.h>
#include <Eigen/Geometry>
#include <vector>
#include <random>
#include <string>
#include <memory>

struct LinkCloud
{
    std::string link_name;
    std::vector<Eigen::Vector3d> points;
};

class RobotPointCloudFilter
{
public:
    RobotPointCloudFilter(ros::NodeHandle& nh) : nh_(nh)
    {
        // 先初始化发布和订阅
        cloud_sub_ = nh_.subscribe("/l515/depth/color/points", 1, 
                                  &RobotPointCloudFilter::pointcloudCallback, this);
        filtered_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("filtered_pointcloud", 1);
        // robot_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("robot_pointcloud", 1);

        // 参数设置 - 增加性能相关参数
        nh_.param("filter_radius", filter_radius_, 0.03);//过滤半径
        nh_.param("robot_cloud_density", robot_cloud_density_, 0.05);//机器人点云密度
        nh_.param("publish_robot_cloud", publish_robot_cloud_, false);
        nh_.param("voxel_leaf_size", voxel_leaf_size_, 0.01);
        nh_.param("max_points_per_link", max_points_per_link_, 5000);
        nh_.param("use_voxel_filter", use_voxel_filter_, true);

        // 直接获取robot_description参数
        std::string robot_desc;
        if (nh_.getParam("/robot_description", robot_desc))
        {
            if (urdf_model_.initString(robot_desc))
            {
                ROS_INFO("Successfully parsed URDF model");
                tf_listener_ = std::make_shared<tf::TransformListener>();
                cacheRobotMeshPoints();
                urdf_loaded_ = true;
            }
            else
            {
                ROS_ERROR("Failed to parse robot_description");
            }
        }
        else
        {
            ROS_ERROR("Failed to get /robot_description parameter");
            ROS_WARN("Will run without robot filtering");
        }

        ROS_INFO("Pointcloud filter node started - Performance Optimized");
        ROS_INFO("Filter radius: %f", filter_radius_);
        ROS_INFO("Robot cloud density: %f", robot_cloud_density_);
        ROS_INFO("Voxel leaf size: %f", voxel_leaf_size_);
        ROS_INFO("Max points per link: %d", max_points_per_link_);
        ROS_INFO("URDF loaded: %s", urdf_loaded_ ? "true" : "false");
    }

private:
    ros::NodeHandle& nh_;
    ros::Subscriber cloud_sub_;
    ros::Publisher filtered_pub_;
    // ros::Publisher robot_cloud_pub_;
    
    urdf::Model urdf_model_;
    std::shared_ptr<tf::TransformListener> tf_listener_;
    
    double filter_radius_;
    double robot_cloud_density_;
    double voxel_leaf_size_;
    int max_points_per_link_;
    bool publish_robot_cloud_;
    bool use_voxel_filter_;
    bool urdf_loaded_ = false;
    
    std::vector<LinkCloud> cached_robot_points_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_robot_cloud_;
    ros::Time last_tf_update_time_;
    double tf_update_interval_ = 0.1;

    void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        // 转换输入点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *input_cloud);
        
        if (input_cloud->empty())
        {
            ROS_WARN_THROTTLE(10.0, "Received empty pointcloud");
            return;
        }

        // 移除NaN点
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*input_cloud, *input_cloud, indices);
        
        if (input_cloud->empty())
        {
            ROS_WARN_THROTTLE(10.0, "Pointcloud is empty after removing NaN points");
            return;
        }

        // 对输入点云进行体素滤波
        if (use_voxel_filter_ && input_cloud->size() > 10000)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr voxeled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
            voxel_filter.setInputCloud(input_cloud);
            voxel_filter.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
            voxel_filter.filter(*voxeled_cloud);
            input_cloud = voxeled_cloud;
            ROS_DEBUG_THROTTLE(5.0, "Voxel filtered to %lu points", input_cloud->size());
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud;

        if (urdf_loaded_)
        {
            ros::Time now = ros::Time::now();
            if ((now - last_tf_update_time_).toSec() > tf_update_interval_)
            {
                if (updateRobotCloud())
                {
                    last_tf_update_time_ = now;
                }
            }

            if (current_robot_cloud_ && !current_robot_cloud_->empty())
            {
                filtered_cloud = filterPointCloud(input_cloud);
            }
            else
            {
                filtered_cloud = input_cloud;
            }
        }
        else
        {
            filtered_cloud = input_cloud;
        }
        
        // 发布结果
        sensor_msgs::PointCloud2 filtered_msg;
        pcl::toROSMsg(*filtered_cloud, filtered_msg);
        filtered_msg.header = msg->header;
        filtered_pub_.publish(filtered_msg);
        
        // 降低机器人点云发布频率
        if (publish_robot_cloud_ && current_robot_cloud_ && !current_robot_cloud_->empty())
        {
            static int publish_count = 0;
            if (publish_count++ % 1 == 0)
            {
                sensor_msgs::PointCloud2 robot_cloud_msg;
                pcl::toROSMsg(*current_robot_cloud_, robot_cloud_msg);
                robot_cloud_msg.header = msg->header;
                // robot_cloud_pub_.publish(robot_cloud_msg);
            }
        }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr filterPointCloud(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr result_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        
        if (!current_robot_cloud_ || current_robot_cloud_->empty())
        {
            *result_cloud = *input_cloud;
            return result_cloud;
        }

        // 检查机器人点云是否有NaN值
        std::vector<int> robot_indices;
        pcl::PointCloud<pcl::PointXYZ>::Ptr clean_robot_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::removeNaNFromPointCloud(*current_robot_cloud_, *clean_robot_cloud, robot_indices);
        
        if (clean_robot_cloud->empty())
        {
            *result_cloud = *input_cloud;
            return result_cloud;
        }

        // 使用KDTree进行最近邻搜索
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(clean_robot_cloud);
        
        std::vector<int> point_indices;
        std::vector<float> point_distances;
        
        // 预分配内存（优化性能）
        result_cloud->points.reserve(input_cloud->size());
        
        // 使用平方距离比较，避免开方运算
        double squared_radius = filter_radius_ * filter_radius_;
        
        for (size_t i = 0; i < input_cloud->size(); ++i)
        {
            const auto& point = input_cloud->points[i];
            
            // 检查点是否有效
            if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
            {
                continue;
            }
            
            // 搜索最近的机器人点
            if (kdtree.nearestKSearch(point, 1, point_indices, point_distances) > 0)
            {
                // 使用平方距离比较
                if (point_distances[0] > squared_radius)
                {
                    result_cloud->push_back(point);
                }
            }
            else
            {
                result_cloud->push_back(point);
            }
        }
        
        // 移除shrink_to_fit调用，PointCloud不需要这个操作
        
        ROS_DEBUG_THROTTLE(5.0, "Filtered %lu points to %lu points", 
                          input_cloud->size(), result_cloud->size());
        
        return result_cloud;
    }

    bool updateRobotCloud()
    {
        if (!tf_listener_ || cached_robot_points_.empty())
        {
            return false;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr robot_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        
        for (const auto& link_cloud : cached_robot_points_)
        {
            tf::StampedTransform transform;
            try
            {
                tf_listener_->waitForTransform("l515_depth_optical_frame", link_cloud.link_name, 
                                              ros::Time(0), ros::Duration(0.01));
                tf_listener_->lookupTransform("l515_depth_optical_frame", link_cloud.link_name, 
                                             ros::Time(0), transform);
            }
            catch (tf::TransformException &ex)
            {
                try
                {
                    tf_listener_->waitForTransform("l515_color_optical_frame", link_cloud.link_name,
                                                  ros::Time(0), ros::Duration(0.01));
                    tf_listener_->lookupTransform("l515_color_optical_frame", link_cloud.link_name,
                                                 ros::Time(0), transform);
                }
                catch (tf::TransformException &ex2)
                {
                    continue;
                }
            }

            // 变换点云到相机坐标系
            for (const auto& p : link_cloud.points)
            {
                tf::Vector3 pt(p.x(), p.y(), p.z());
                tf::Vector3 pt_trans = transform * pt;
                
                if (std::isfinite(pt_trans.x()) && std::isfinite(pt_trans.y()) && std::isfinite(pt_trans.z()))
                {
                    robot_cloud->push_back(pcl::PointXYZ(pt_trans.x(), pt_trans.y(), pt_trans.z()));
                }
            }
        }
        
        if (!robot_cloud->empty())
        {
            // 对机器人点云也进行体素滤波
            if (use_voxel_filter_ && robot_cloud->size() > 5000)
            {
                pcl::PointCloud<pcl::PointXYZ>::Ptr voxeled_robot_cloud(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
                voxel_filter.setInputCloud(robot_cloud);
                voxel_filter.setLeafSize(voxel_leaf_size_ * 2, voxel_leaf_size_ * 2, voxel_leaf_size_ * 2);
                voxel_filter.filter(*voxeled_robot_cloud);
                current_robot_cloud_ = voxeled_robot_cloud;
            }
            else
            {
                current_robot_cloud_ = robot_cloud;
            }
            return true;
        }
        
        return false;
    }

    void cacheRobotMeshPoints()
    {
        ROS_INFO("Caching robot mesh points with performance optimization...");
        int total_points = 0;
        
        for (const auto& link_pair : urdf_model_.links_)
        {
            urdf::LinkConstSharedPtr link = link_pair.second;
            
            // 跳过相机相关的link
            if (link->name.find("l515") != std::string::npos ||
                link->name.find("camera") != std::string::npos ||
                link->name.find("optical") != std::string::npos ||
                link->name.find("L515") != std::string::npos)
            {
                continue;
            }
            
            if (!link->visual || !link->visual->geometry)
                continue;

            if (link->visual->geometry->type != urdf::Geometry::MESH)
                continue;

            urdf::Mesh* mesh = dynamic_cast<urdf::Mesh*>(link->visual->geometry.get());
            if (!mesh || mesh->filename.empty())
                continue;

            shapes::Mesh* shape = shapes::createMeshFromResource(mesh->filename);
            if (!shape) 
            {
                continue;
            }

            std::vector<Eigen::Vector3d> sampled_points;
            sampleMesh(shape, robot_cloud_density_, sampled_points);
            delete shape;

            if (!sampled_points.empty())
            {
                // 限制每个link的最大点数
                if (sampled_points.size() > max_points_per_link_)
                {
                    sampled_points.resize(max_points_per_link_);
                }
                cached_robot_points_.push_back({link->name, sampled_points});
                total_points += sampled_points.size();
            }
        }
        ROS_INFO("Total cached %d points from %lu robot links", total_points, cached_robot_points_.size());
    }

    void sampleMesh(shapes::Mesh* mesh, double density, std::vector<Eigen::Vector3d>& points)
    {
        if (!mesh) return;
        
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<double> dist(0.0, 1.0);

        // 预分配内存
        points.reserve(mesh->triangle_count * 10);

        for (unsigned int i = 0; i < mesh->triangle_count; ++i)
        {
            int idx0 = mesh->triangles[i*3+0];
            int idx1 = mesh->triangles[i*3+1];
            int idx2 = mesh->triangles[i*3+2];

            Eigen::Vector3d v0(mesh->vertices[idx0 * 3+0], mesh->vertices[idx0 * 3+1], mesh->vertices[idx0 * 3+2]);
            Eigen::Vector3d v1(mesh->vertices[idx1 * 3+0], mesh->vertices[idx1 * 3+1], mesh->vertices[idx1 * 3+2]);
            Eigen::Vector3d v2(mesh->vertices[idx2 * 3+0], mesh->vertices[idx2 * 3+1], mesh->vertices[idx2 * 3+2]);

            // 计算三角形面积
            double area = ((v1 - v0).cross(v2 - v0)).norm() * 0.5;
            int points_per_triangle = std::max(1, static_cast<int>(area / (density * density)));
            
            // 限制每个三角形的最大点数
            points_per_triangle = std::min(points_per_triangle, 50);

            for (int n = 0; n < points_per_triangle; ++n)
            {
                double r1 = dist(gen);
                double r2 = dist(gen);
                if (r1 + r2 > 1.0) 
                {
                    r1 = 1.0 - r1; 
                    r2 = 1.0 - r2; 
                }
                points.push_back(v0 + r1*(v1-v0) + r2*(v2-v0));
            }
        }
        
        // 移除shrink_to_fit调用
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_pointcloud_filter");
    ros::NodeHandle nh("~");
    
    RobotPointCloudFilter filter(nh);
    
    ros::spin();
    return 0;
}