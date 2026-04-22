#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <urdf/model.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shapes.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Geometry>
#include <vector>
#include <random>
#include <map>

#include <sensor_msgs/JointState.h>

class RobotVisualPointCloud
{
public:
    RobotVisualPointCloud(ros::NodeHandle& nh)
    {
        cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("robot_visual_pointcloud", 1);

        // 读取 robot_description
        std::string robot_desc;
        nh.getParam("robot_description", robot_desc);
        if (!urdf_model_.initString(robot_desc))
        {
            ROS_ERROR("Failed to parse robot_description");
            ros::shutdown();
        }

        // 初始化 TF listener
        tf_listener_ = std::make_shared<tf::TransformListener>();

        // 订阅 joint_states
        joint_sub_ = nh.subscribe("/joint_states", 10, &RobotVisualPointCloud::jointStateCallback, this);

        // 默认采样密度
        nh.param("density", density_, 0.08);
    }

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        joint_positions_.clear();
        for (size_t i = 0; i < msg->name.size(); ++i)
        {
            joint_positions_[msg->name[i]] = msg->position[i];
        }
    }

    void publishPointCloud()
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

        std::lock_guard<std::mutex> lock(mutex_);
        for (const auto& link_pair : urdf_model_.links_)
        {
            urdf::LinkConstSharedPtr link = link_pair.second;
            if (!link->visual || !link->visual->geometry)
                continue;

            if (link->visual->geometry->type != urdf::Geometry::MESH)
                continue;

            urdf::Mesh* mesh = dynamic_cast<urdf::Mesh*>(link->visual->geometry.get());
            if (!mesh || mesh->filename.empty())
                continue;

            // 加载 mesh
            shapes::Mesh* shape = shapes::createMeshFromResource(mesh->filename);
            if (!shape) continue;

            // 遍历三角形均匀采样
            std::vector<Eigen::Vector3d> sampled_points;
            sampleMesh(shape, density_, sampled_points);

            // link 原点和旋转
            tf::Transform tf_link;
            tf::Vector3 origin(link->visual->origin.position.x,
                               link->visual->origin.position.y,
                               link->visual->origin.position.z);
            tf::Quaternion rot(link->visual->origin.rotation.x,
                               link->visual->origin.rotation.y,
                               link->visual->origin.rotation.z,
                               link->visual->origin.rotation.w);
            tf_link.setOrigin(origin);
            tf_link.setRotation(rot);

            for (const auto& p : sampled_points)
            {
                tf::Vector3 pt(p.x(), p.y(), p.z());
                tf::Vector3 pt_trans = tf_link * pt;
                cloud->push_back(pcl::PointXYZ(pt_trans.x(), pt_trans.y(), pt_trans.z()));
            }

            delete shape;
        }

        cloud->header.frame_id = "base_link";
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud, cloud_msg);
        cloud_pub_.publish(cloud_msg);
    }

private:
    urdf::Model urdf_model_;
    std::shared_ptr<tf::TransformListener> tf_listener_;
    ros::Publisher cloud_pub_;
    ros::Subscriber joint_sub_;
    double density_;
    std::mutex mutex_;
    std::map<std::string, double> joint_positions_;

    // 简单三角形均匀采样函数
    void sampleMesh(shapes::Mesh* mesh, double density, std::vector<Eigen::Vector3d>& points)
    {
        if (!mesh) return;

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<double> dist(0.0, 1.0);

        for (unsigned int i = 0; i < mesh->triangle_count; ++i)
        {
            int idx0 = mesh->triangles[i*3+0];
            int idx1 = mesh->triangles[i*3+1];
            int idx2 = mesh->triangles[i*3+2];

            Eigen::Vector3d v0(mesh->vertices[idx0*3+0], mesh->vertices[idx0*3+1], mesh->vertices[idx0*3+2]);
            Eigen::Vector3d v1(mesh->vertices[idx1*3+0], mesh->vertices[idx1*3+1], mesh->vertices[idx1*3+2]);
            Eigen::Vector3d v2(mesh->vertices[idx2*3+0], mesh->vertices[idx2*3+1], mesh->vertices[idx2*3+2]);

            int num_points = std::max(1, static_cast<int>((v0 - v1).norm() / density));

            for (int n = 0; n < num_points; ++n)
            {
                double r1 = dist(gen);
                double r2 = dist(gen);
                if (r1 + r2 > 1.0) { r1 = 1.0 - r1; r2 = 1.0 - r2; }
                Eigen::Vector3d p = v0 + r1*(v1-v0) + r2*(v2-v0);
                points.push_back(p);
            }
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "visual_pointcloud_node");
    ros::NodeHandle nh;

    RobotVisualPointCloud visual_pc(nh);

    ros::Rate rate(5);
    while (ros::ok())
    {
        visual_pc.publishPointCloud();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

