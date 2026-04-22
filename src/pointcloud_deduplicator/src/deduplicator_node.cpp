#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <unordered_set>
#include <cmath>
#include <tuple>

struct VoxelHash
{
    double leaf_size;

    VoxelHash(double ls) : leaf_size(ls) {}

    std::tuple<int,int,int> voxelKey(const pcl::PointXYZ& pt) const
    {
        return std::make_tuple(
            static_cast<int>(std::floor(pt.x / leaf_size)),
            static_cast<int>(std::floor(pt.y / leaf_size)),
            static_cast<int>(std::floor(pt.z / leaf_size))
        );
    }
};

class PointCloudDeduplicator
{
public:
    PointCloudDeduplicator()
    {
        ros::NodeHandle private_nh("~");
        private_nh.param<double>("voxel_leaf_size", voxel_leaf_size_, 0.2); // 默认 5mm

        pub_ = nh_.advertise<sensor_msgs::PointCloud2>("dedup_points", 1);
        sub_source_ = nh_.subscribe("/l515/depth/color/points", 1, &PointCloudDeduplicator::sourceCallback, this);
        sub_existing_ = nh_.subscribe("/robot_visual_pointcloud", 1, &PointCloudDeduplicator::existingCallback, this);

        existing_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_source_, sub_existing_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr existing_cloud_;
    double voxel_leaf_size_;

    std::unordered_set<std::tuple<int,int,int>, boost::hash<std::tuple<int,int,int>>> occupied_voxels_;

    void existingCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        pcl::fromROSMsg(*msg, *existing_cloud_);
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*existing_cloud_, *existing_cloud_, indices);

        // 构建占用 voxel 集合
        occupied_voxels_.clear();
        VoxelHash vh(voxel_leaf_size_);
        for (const auto& pt : existing_cloud_->points)
        {
            if (!pcl::isFinite(pt)) continue;
            occupied_voxels_.insert(vh.voxelKey(pt));
        }
    }

    void sourceCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *source_cloud);
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*source_cloud, *source_cloud, indices);

        pcl::PointCloud<pcl::PointXYZ>::Ptr dedup_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        VoxelHash vh(voxel_leaf_size_);
        for (const auto& pt : source_cloud->points)
        {
            if (!pcl::isFinite(pt)) continue;
            auto key = vh.voxelKey(pt);
            if (occupied_voxels_.find(key) == occupied_voxels_.end())
            {
                dedup_cloud->points.push_back(pt);
            }
        }

        dedup_cloud->width = dedup_cloud->points.size();
        dedup_cloud->height = 1;
        dedup_cloud->is_dense = true;

        sensor_msgs::PointCloud2 output_msg;
        pcl::toROSMsg(*dedup_cloud, output_msg);
        output_msg.header = msg->header;
        pub_.publish(output_msg);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pointcloud_deduplicator");
    PointCloudDeduplicator dedup;
    ros::spin();
    return 0;
}

