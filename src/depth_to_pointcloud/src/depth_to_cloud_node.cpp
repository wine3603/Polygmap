#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <cv_bridge/cv_bridge.h>

class DepthToCloud {
public:
    DepthToCloud(ros::NodeHandle& nh) {
        image_sub_ = nh.subscribe("/l515/depth/image_rect_raw", 1, &DepthToCloud::imageCallback, this);
        info_sub_  = nh.subscribe("/l515/depth/camera_info", 1, &DepthToCloud::infoCallback, this);
        cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/depth_to_points", 1);
        got_camera_info_ = false;
    }

private:
    ros::Subscriber image_sub_, info_sub_;
    ros::Publisher cloud_pub_;
    bool got_camera_info_;
    float fx_, fy_, cx_, cy_;

    void infoCallback(const sensor_msgs::CameraInfoConstPtr& msg) {
        fx_ = msg->K[0];
        fy_ = msg->K[4];
        cx_ = msg->K[2];
        cy_ = msg->K[5];
        got_camera_info_ = true;
        ROS_INFO_ONCE("Camera info received.");
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        if (!got_camera_info_) return;

        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_16UC1);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        int width = msg->width;
        int height = msg->height;
        int step = 8;  // 采样步长

        std::vector<float> points;  // 用于缓存点
        for (int v = 0; v < height; v += step) {
            for (int u = 0; u < width; u += step) {
                uint16_t depth = cv_ptr->image.at<uint16_t>(v, u);
                if (depth == 0) continue;

                float z = depth * 0.001f;
                float x = (u - cx_) * z / fx_;
                float y = (v - cy_) * z / fy_;

                points.push_back(x);
                points.push_back(y);
                points.push_back(z);
            }
        }

        // 构造 PointCloud2 消息
        sensor_msgs::PointCloud2 cloud_msg;
        cloud_msg.header = msg->header;
        cloud_msg.height = 1;
        cloud_msg.width = points.size() / 3;
        cloud_msg.is_dense = false;

        sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(cloud_msg.width);

        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

        for (size_t i = 0; i < points.size(); i += 3, ++iter_x, ++iter_y, ++iter_z) {
            *iter_x = points[i];
            *iter_y = points[i + 1];
            *iter_z = points[i + 2];
        }

        cloud_pub_.publish(cloud_msg);

        // ✅ 打印生成的点数量
        ROS_INFO("Published point cloud with %lu points", points.size() / 3);
    }


};

int main(int argc, char** argv) {
    ros::init(argc, argv, "depth_to_cloud_node");
    ros::NodeHandle nh;
    DepthToCloud node(nh);
    ros::spin();
    return 0;
}

