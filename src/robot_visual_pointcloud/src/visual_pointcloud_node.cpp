#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

#include <urdf/model.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shapes.h>

#include <tf/transform_listener.h>
#include <Eigen/Geometry>
#include <vector>
#include <random>
#include <string>

struct LinkCloud
{
    std::string link_name;
    std::vector<Eigen::Vector3d> points;
};

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

        nh.param("density", density_, 0.1);
        nh.param("use_voxel_filter", use_voxel_, true);
        nh.param("voxel_leaf", voxel_leaf_, 0.005);

        // 缓存所有 mesh 点云
        cacheMeshPoints();
    }

    void publishPointCloud()
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

        for (const auto& link_cloud : cached_mesh_points_)
        {
            tf::StampedTransform transform;
            try
            {
                tf_listener_->lookupTransform("base_link", link_cloud.link_name, ros::Time(0), transform);
            }
            catch (tf::TransformException &ex)
            {
                ROS_WARN_THROTTLE(1.0, "TF lookup failed for link %s: %s", link_cloud.link_name.c_str(), ex.what());
                continue;
            }

            for (const auto& p : link_cloud.points)
            {
                tf::Vector3 pt(p.x(), p.y(), p.z());
                tf::Vector3 pt_trans = transform * pt;
                cloud->push_back(pcl::PointXYZ(pt_trans.x(), pt_trans.y(), pt_trans.z()));
            }
        }

        if (use_voxel_)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::VoxelGrid<pcl::PointXYZ> vg;
            vg.setInputCloud(cloud);
            vg.setLeafSize(voxel_leaf_, voxel_leaf_, voxel_leaf_);
            vg.filter(*cloud_filtered);
            cloud = cloud_filtered;
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
    double density_;
    bool use_voxel_;
    double voxel_leaf_;

    std::vector<LinkCloud> cached_mesh_points_;

    void cacheMeshPoints()
    {
        ROS_INFO("Caching mesh points for all links...");
        for (const auto& link_pair : urdf_model_.links_)
        {
            urdf::LinkConstSharedPtr link = link_pair.second;
            // 排除 l515_link
            if (link->name == "l515_link")
                continue;
            if (link->name == "l515")
                continue;
            if (link->name == "l515_base")
                continue;
            
                if (!link->visual || !link->visual->geometry)
                continue;

            if (link->visual->geometry->type != urdf::Geometry::MESH)
                continue;

            urdf::Mesh* mesh = dynamic_cast<urdf::Mesh*>(link->visual->geometry.get());
            if (!mesh || mesh->filename.empty())
                continue;

            shapes::Mesh* shape = shapes::createMeshFromResource(mesh->filename);
            if (!shape) continue;

            std::vector<Eigen::Vector3d> sampled_points;
            sampleMesh(shape, density_, sampled_points);
            delete shape;

            if (!sampled_points.empty())
                cached_mesh_points_.push_back({link->name, sampled_points});
        }
        ROS_INFO("Cached %lu link meshes.", cached_mesh_points_.size());
    }

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

    ros::Rate rate(30); // 提高刷新率
    while (ros::ok())
    {
        visual_pc.publishPointCloud();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
