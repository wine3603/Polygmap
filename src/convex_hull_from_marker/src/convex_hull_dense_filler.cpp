#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/surface/convex_hull.h>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/functional/hash.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <unordered_map>
#include <unordered_set>
#include <cmath>
#include <limits>
#include <vector>
#include <string>

#include <nav_msgs/Odometry.h>

using Key = std::pair<int, int>;

// 全局变量和参数
ros::Publisher pub_cloud;      // 发布完整点云
ros::Publisher pub_filtered;   // 发布范围筛选后点云
ros::Publisher pub_max_z;      // 发布最大Z点并腐蚀后的点云
ros::Publisher pub_rectangles;
ros::Publisher pub_rectangles2;
ros::Publisher pub_foot_layer;  // 脚底附近层腐蚀后点云
ros::Publisher pub_nearest_odom;

tf2_ros::Buffer tf_buffer;

float g_resolution;       // 栅格分辨率
float g_range_xy;         // XY范围筛选阈值
int g_erosion_rounds;     // 脚底附近层腐蚀轮数
int g_erosion_rounds_above_below;  // 脚底附近层上下层腐蚀轮数（统一）
float g_z_threshold;      // Z阈值，用于脚底高度附近过滤
float g_layer_height;     // 高度分层步长，0.05米
float foot_layer_delta;   // 脚底附近层高范围阈值，单位米


// void publishRectangles(const geometry_msgs::Point& center_point, ros::Publisher& pub, const std_msgs::Header& header)
// {
//     float length = 0.25f;  // 25cm 长边（x方向）
//     float width = 0.10f;   // 10cm 宽边（y方向）
//     float offset_y = 0.1f; // 左右偏移0.1米

//     for (int i = 0; i < 2; ++i) {
//         visualization_msgs::Marker marker;
//         marker.header = header;
//         marker.ns = "nearest_rectangles";
//         marker.id = i;
//         marker.type = visualization_msgs::Marker::LINE_STRIP;
//         marker.action = visualization_msgs::Marker::ADD;
//         marker.pose.orientation.w = 1.0;
//         marker.scale.x = 0.02;  // 线宽2cm
//         marker.color.a = 1.0;

//         if (i == 0) {  // 左边红色
//             marker.color.r = 1.0;
//             marker.color.g = 0.0;
//             marker.color.b = 0.0;
//         } else {       // 右边蓝色
//             marker.color.r = 0.0;
//             marker.color.g = 0.0;
//             marker.color.b = 1.0;
//         }

//         float cx = center_point.x;
//         float cy = center_point.y + (i == 0 ? -offset_y : offset_y); // 左右偏移
//         float cz = center_point.z;  // 使用实际高度

//         std::vector<geometry_msgs::Point> pts(5);
//         // 矩形4个顶点顺序，闭合最后一点=第一个
//         pts[0].x = cx - length / 2; pts[0].y = cy - width / 2; pts[0].z = cz;
//         pts[1].x = cx + length / 2; pts[1].y = cy - width / 2; pts[1].z = cz;
//         pts[2].x = cx + length / 2; pts[2].y = cy + width / 2; pts[2].z = cz;
//         pts[3].x = cx - length / 2; pts[3].y = cy + width / 2; pts[3].z = cz;
//         pts[4] = pts[0]; // 闭合

//         marker.points = pts;
//         pub.publish(marker);
//     }
// }


void publishRectangles(const geometry_msgs::Point& center_point,
                       ros::Publisher& pub,
                       const std_msgs::Header& header,
                       double direction_in_odom)
{
    float length = 0.25f;  // 25cm 长边（x方向）
    float width = 0.10f;   // 10cm 宽边（y方向）
    float offset_y = 0.1f; // 左右偏移0.1米

    float cos_theta = std::cos(direction_in_odom);
    float sin_theta = std::sin(direction_in_odom);

    for (int i = 0; i < 2; ++i) {
        visualization_msgs::Marker marker;
        marker.header = header;
        marker.ns = "nearest_rectangles";
        marker.id = i;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.02;  // 线宽2cm
        marker.color.a = 1.0;

        if (i == 0) {
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
        } else {
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
        }

        float offset = (i == 0 ? -offset_y : offset_y);

        // 定义矩形顶点相对中心的局部坐标（左下开始，逆时针）
        std::vector<std::pair<float, float>> local_pts = {
            {-length / 2, -width / 2 + offset},
            { length / 2, -width / 2 + offset},
            { length / 2,  width / 2 + offset},
            {-length / 2,  width / 2 + offset},
            {-length / 2, -width / 2 + offset}  // 闭合
        };

        for (const auto& pt : local_pts) {
            float local_x = pt.first;
            float local_y = pt.second;

            float rotated_x = cos_theta * local_x - sin_theta * local_y;
            float rotated_y = sin_theta * local_x + cos_theta * local_y;

            geometry_msgs::Point p;
            p.x = center_point.x + rotated_x;
            p.y = center_point.y + rotated_y;
            p.z = center_point.z;
            marker.points.push_back(p);
        }

        pub.publish(marker);
    }
}


// 执行一次 XY 平面的腐蚀（去除最外层一圈）
pcl::PointCloud<pcl::PointXYZ>::Ptr erodePointCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
    float resolution)
{
    std::unordered_map<Key, pcl::PointXYZ, boost::hash<Key>> grid_map;
    std::unordered_set<Key, boost::hash<Key>> occupied;

    // 将点云映射到二维栅格，并记录占用格子
    for (const auto& pt : input_cloud->points) {
        int xi = static_cast<int>(std::round(pt.x / resolution));
        int yi = static_cast<int>(std::round(pt.y / resolution));
        Key key = {xi, yi};
        grid_map[key] = pt;
        occupied.insert(key);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // 检查每个格子邻居，若有缺失则视为边界，过滤掉
    for (const auto& kv : grid_map) {
        const Key& key = kv.first;
        bool is_inner = true;

        for (int dx = -1; dx <= 1 && is_inner; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                if (dx == 0 && dy == 0) continue;
                Key neighbor = {key.first + dx, key.second + dy};
                if (!occupied.count(neighbor)) {
                    is_inner = false;
                    break;
                }
            }
        }

        if (is_inner) {
            output_cloud->points.push_back(kv.second);
        }
    }

    return output_cloud;
}

void markerArrayCallback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr total_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 遍历每个 Marker（多边形）
    for (const auto& marker : msg->markers)
    {
        if (marker.points.size() < 3) continue;

        // 仅使用 x,y，z置0，构建输入点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& pt : marker.points)
            input_cloud->points.emplace_back(pt.x, pt.y, 0.0f);

        // 计算二维凸包
        pcl::PointCloud<pcl::PointXYZ>::Ptr hull(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ConvexHull<pcl::PointXYZ> chull;
        chull.setInputCloud(input_cloud);
        chull.setDimension(2);
        chull.reconstruct(*hull);

        if (hull->points.size() < 3) continue;

        // 计算凸包包围盒，用于填充
        float min_x = std::numeric_limits<float>::max();
        float max_x = -std::numeric_limits<float>::max();
        float min_y = std::numeric_limits<float>::max();
        float max_y = -std::numeric_limits<float>::max();
        for (const auto& pt : hull->points) {
            min_x = std::min(min_x, pt.x);
            max_x = std::max(max_x, pt.x);
            min_y = std::min(min_y, pt.y);
            max_y = std::max(max_y, pt.y);
        }

        // Boost.Geometry多边形用于判点内外
        using boost_point = boost::geometry::model::d2::point_xy<float>;
        boost::geometry::model::polygon<boost_point> polygon;
        for (const auto& pt : hull->points)
            boost::geometry::append(polygon, boost_point(pt.x, pt.y));
        boost::geometry::correct(polygon);

        // 计算原始点云的平均高度，填充点云时用
        float sum_z = 0.0;
        for (const auto& pt : marker.points)
            sum_z += pt.z;
        float avg_z = sum_z / marker.points.size();

        // 在包围盒内以g_resolution为步长填充点
        for (float x = min_x; x <= max_x; x += g_resolution) {
            for (float y = min_y; y <= max_y; y += g_resolution) {
                if (boost::geometry::within(boost_point(x, y), polygon)) {
                    total_cloud->points.emplace_back(x, y, avg_z);
                }
            }
        }
    }

    if (total_cloud->empty()) return;

    // 发布所有多边形填充的点云
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*total_cloud, cloud_msg);
    cloud_msg.header = msg->markers.front().header;
    cloud_msg.header.frame_id = "odom";
    pub_cloud.publish(cloud_msg);

    // 坐标转换，将点云从odom转换到base_link
    geometry_msgs::TransformStamped tf;
    try {
        tf = tf_buffer.lookupTransform("base_link", "odom", ros::Time(0), ros::Duration(0.1));
    } catch (tf2::TransformException& ex) {
        ROS_WARN("TF lookup failed: %s", ex.what());
        return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& pt : total_cloud->points) {
        geometry_msgs::PointStamped pt_in, pt_out;
        pt_in.header.frame_id = "odom";
        pt_in.point.x = pt.x;
        pt_in.point.y = pt.y;
        pt_in.point.z = pt.z;

        try {
            tf2::doTransform(pt_in, pt_out, tf);
            if (std::abs(pt_out.point.x) <= g_range_xy && std::abs(pt_out.point.y) <= g_range_xy) {
                // 保留在范围内点，保持原坐标系坐标
                filtered_cloud->points.emplace_back(pt.x, pt.y, pt.z);
            }
        } catch (tf2::TransformException& ex) {
            ROS_WARN("Transform point failed: %s", ex.what());
            continue;
        }
    }

    if (!filtered_cloud->empty()) {
        sensor_msgs::PointCloud2 filtered_msg;
        pcl::toROSMsg(*filtered_cloud, filtered_msg);
        filtered_msg.header = cloud_msg.header;
        pub_filtered.publish(filtered_msg);
    }

    // 计算每个栅格内最高点，构造max_z_cloud
    std::unordered_map<Key, pcl::PointXYZ, boost::hash<Key>> max_z_map;
    for (const auto& pt : filtered_cloud->points) {
        int xi = static_cast<int>(std::round(pt.x / g_resolution));
        int yi = static_cast<int>(std::round(pt.y / g_resolution));
        Key key = {xi, yi};
        auto it = max_z_map.find(key);
        if (it == max_z_map.end() || pt.z > it->second.z) {
            max_z_map[key] = pt;
        }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr max_z_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& kv : max_z_map) {
        max_z_cloud->points.push_back(kv.second);
    }

    // 读取机器人脚底各点高度，找最小值
    std::vector<std::string> foot_frames = {
        "ll_foot_toe", "rr_foot_toe", "ll_foot_heel", "rr_foot_heel"
    };

    float min_foot_z = std::numeric_limits<float>::max();
    for (const auto& frame : foot_frames) {
        try {
            geometry_msgs::TransformStamped tf_foot = tf_buffer.lookupTransform("odom", frame, ros::Time(0), ros::Duration(0.1));
            float z = tf_foot.transform.translation.z;
            min_foot_z = std::min(min_foot_z, z);
        } catch (tf2::TransformException& ex) {
            ROS_WARN("TF lookup for %s failed: %s", frame.c_str(), ex.what());
        }
    }

    // 过滤最大z点，剔除过高点（离脚底高度超过阈值）
    pcl::PointCloud<pcl::PointXYZ>::Ptr max_z_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& pt : max_z_cloud->points) {
        if (pt.z <= min_foot_z + g_z_threshold) {
            max_z_filtered->points.push_back(pt);
        }
    }

    // 按层划分最大z点云并分层腐蚀
    std::unordered_map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr> layer_map;
    for (const auto& pt : max_z_filtered->points) {
        int layer_idx = static_cast<int>(std::floor(pt.z / g_layer_height));
        if (layer_map.find(layer_idx) == layer_map.end()) {
            layer_map[layer_idx] = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        }
        layer_map[layer_idx]->points.push_back(pt);
    }

    // 对所有层统一腐蚀次数，并合并结果（不再判断是否靠近脚底，不提前停止）
    pcl::PointCloud<pcl::PointXYZ>::Ptr eroded_all(new pcl::PointCloud<pcl::PointXYZ>);
    for (auto& kv : layer_map) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_layer = kv.second;

        pcl::PointCloud<pcl::PointXYZ>::Ptr current_layer(new pcl::PointCloud<pcl::PointXYZ>(*cloud_layer));

        for (int i = 0; i < g_erosion_rounds; ++i) {
            current_layer = erodePointCloud(current_layer, g_resolution);
        }

        *eroded_all += *current_layer;
    }



    if (!eroded_all->empty()) {
        sensor_msgs::PointCloud2 eroded_msg;
        pcl::toROSMsg(*eroded_all, eroded_msg);
        eroded_msg.header = cloud_msg.header;
        pub_max_z.publish(eroded_msg);

        geometry_msgs::TransformStamped tf_base;
        try {
            tf_base = tf_buffer.lookupTransform("odom", "base_link", ros::Time(0), ros::Duration(0.1));
        } catch (tf2::TransformException& ex) {
            ROS_WARN("TF lookup failed: %s", ex.what());
            return;
        }

        double base_x = tf_base.transform.translation.x;
        double base_y = tf_base.transform.translation.y;

        float min_dist_sq = std::numeric_limits<float>::max();
        geometry_msgs::Point nearest_relative_point;
        geometry_msgs::Point nearest_point_odom;
        bool found = false;
        float odom_x_pt,odom_y_pt;
        for (const auto& pt : eroded_all->points) {
            float z = pt.z;
            if (z > min_foot_z + foot_layer_delta && z <= min_foot_z + g_z_threshold) {
                float dx = pt.x - base_x;
                float dy = pt.y - base_y;
                float dist_sq = dx * dx + dy * dy;

                if (dist_sq < min_dist_sq && dist_sq<=0.25) {//0.5*0.5 ！！！
                    // 在 base_link 附近且满足高度条件，尝试转换坐标
                    geometry_msgs::PointStamped pt_in_odom, pt_in_base;
                    pt_in_odom.header.frame_id = "odom";
                    pt_in_odom.header.stamp = ros::Time(0);
                    pt_in_odom.point.x = pt.x;
                    pt_in_odom.point.y = pt.y;
                    pt_in_odom.point.z = pt.z;

                    try {
                        tf_buffer.transform(pt_in_odom, pt_in_base, "base_link", ros::Duration(0.05));
                        nearest_relative_point = pt_in_base.point;
                        nearest_relative_point.z = pt.z;

                        nearest_point_odom.x = pt.x;
                        nearest_point_odom.y = pt.y;
                        nearest_point_odom.z = pt.z;

                        min_dist_sq = dist_sq;
                        found = true;
                    } catch (tf2::TransformException& ex) {
                        ROS_WARN("TF transform failed: %s", ex.what());
                        continue;
                    }
                }
                odom_x_pt=pt.x;
                odom_y_pt=pt.y;
            }
        }

        // ====== 新增逻辑：找到第二个点 higher_nearest_point_odom ======
        geometry_msgs::Point higher_nearest_point_odom;
        geometry_msgs::Point higher_nearest_relative_point;
        bool found_second = false;
        float min_dist_sq_high2 = std::numeric_limits<float>::max();
        float odom_x_pt2,odom_y_pt2;
        if (found) {
            for (const auto& pt : eroded_all->points) {
                // 必须比第一个点层级更高
                if (pt.z > nearest_point_odom.z + g_layer_height * 0.5 &&
                    pt.z <= nearest_point_odom.z + g_z_threshold) 
                {
                    float dx = pt.x - nearest_point_odom.x;
                    float dy = pt.y - nearest_point_odom.y;
                    float dist_sq = dx * dx + dy * dy;

                    // if (dist_sq < min_dist_sq_high2 && dist_sq<=0.25) 
                    if (dist_sq < min_dist_sq_high2) 
                    {
                        // 在 base_link 附近且满足高度条件，尝试转换坐标
                        geometry_msgs::PointStamped pt_in_odom, pt_in_base;
                        pt_in_odom.header.frame_id = "odom";
                        pt_in_odom.header.stamp = ros::Time(0);
                        pt_in_odom.point.x = pt.x;
                        pt_in_odom.point.y = pt.y;
                        pt_in_odom.point.z = pt.z;

                        try {
                            tf_buffer.transform(pt_in_odom, pt_in_base, "base_link", ros::Duration(0.05));
                            higher_nearest_relative_point = pt_in_base.point;
                            higher_nearest_relative_point.z = pt.z;

                            higher_nearest_point_odom.x = pt.x;
                            higher_nearest_point_odom.y = pt.y;
                            higher_nearest_point_odom.z = pt.z;

                            min_dist_sq_high2 = dist_sq;
                            found_second = true;
                        } catch (tf2::TransformException& ex) {
                            ROS_WARN("TF transform failed: %s", ex.what());
                            continue;
                        }
                    }
                    odom_x_pt2=pt.x;
                    odom_y_pt2=pt.y;
                }
            }
        }

        tf2::Quaternion q(
            tf_base.transform.rotation.x,
            tf_base.transform.rotation.y,
            tf_base.transform.rotation.z,
            tf_base.transform.rotation.w
        );
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        double direction_in_odom = std::atan2(nearest_point_odom.y, nearest_point_odom.x);

        double relative_angle = direction_in_odom - yaw;
        relative_angle = std::atan2(std::sin(relative_angle), std::cos(relative_angle));

        tf2::Quaternion q_rot;
        q_rot.setRPY(0, 0, relative_angle);  // 只绕 Z 轴
        geometry_msgs::Quaternion q_msg = tf2::toMsg(q_rot);


        double roll2, pitch2, yaw2;
        tf2::Matrix3x3(q_rot).getRPY(roll2, pitch2, yaw2);
        double direction_in_odom2 = std::atan2(higher_nearest_point_odom.y, higher_nearest_point_odom.x);
        double relative_angle2 = direction_in_odom2 - yaw2;
        relative_angle2 = std::atan2(std::sin(relative_angle2), std::cos(relative_angle2));
        tf2::Quaternion q_rot2;
        q_rot2.setRPY(0, 0, relative_angle2);  // 只绕 Z 轴
        geometry_msgs::Quaternion q_msg2 = tf2::toMsg(q_rot2);


        ROS_INFO("Yaw (odom->base_link): %.3f rad", yaw);
        ROS_INFO("base_x:%.3f,base_y:%.3f", base_x,base_y);
        ROS_INFO("base_x:%.3f,base_y:%.3f", odom_x_pt,odom_y_pt);
        ROS_INFO("Direction (odom): %.3f rad", direction_in_odom);
        ROS_INFO("Relative angle (base_link->point): %.3f du", relative_angle * 180.0 / M_PI);

        // =================== 脚底附近层单独腐蚀并发布 ===================
        int min_layer_idx = static_cast<int>(std::floor((min_foot_z - foot_layer_delta) / g_layer_height));
        int max_layer_idx = static_cast<int>(std::floor((min_foot_z + foot_layer_delta) / g_layer_height));

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_foot_layer_combined(new pcl::PointCloud<pcl::PointXYZ>);

        for (int layer_idx = min_layer_idx; layer_idx <= max_layer_idx; ++layer_idx) {
            auto it = layer_map.find(layer_idx);
            if (it != layer_map.end()) {
                *cloud_foot_layer_combined += *(it->second);  // 合并多个层
            }
        }

        if (!cloud_foot_layer_combined->empty()) {
            for (int i = 0; i < g_erosion_rounds_above_below; ++i) {
                cloud_foot_layer_combined = erodePointCloud(cloud_foot_layer_combined, g_resolution);
            }

            sensor_msgs::PointCloud2 foot_layer_msg;
            pcl::toROSMsg(*cloud_foot_layer_combined, foot_layer_msg);
            foot_layer_msg.header = cloud_msg.header;
            pub_foot_layer.publish(foot_layer_msg);
        }


        if (found) {
            ROS_INFO("Nearest point relative to base_link (odom frame): x=%.3f, y=%.3f, z=%.3f",
                    nearest_relative_point.x,
                    nearest_relative_point.y,
                    nearest_relative_point.z);
            std::cout << "Min foot z: " << min_foot_z << std::endl;
            // 发布两个矩形 Marker
            // publishRectangles(nearest_point_odom, pub_rectangles, msg->markers.front().header);
            publishRectangles(nearest_point_odom, pub_rectangles, msg->markers.front().header, direction_in_odom);
            if(found_second)
            {
                    ROS_INFO("Nearest point relative to base_link (odom frame): x=%.3f, y=%.3f, z=%.3f",
                    higher_nearest_relative_point.x,
                    higher_nearest_relative_point.y,
                    higher_nearest_relative_point.z);
            std::cout << "found_second: " << found_second << std::endl;
                publishRectangles(higher_nearest_point_odom, pub_rectangles2, msg->markers.front().header, direction_in_odom2);
            }

            // 发布nav_msgs/Odometry消息，最近点相对base_link
            nav_msgs::Odometry odom_msg;
            odom_msg.header = msg->markers.front().header;
            odom_msg.header.frame_id = "odom";
            odom_msg.child_frame_id = "nearest_point";

            odom_msg.pose.pose.position.x = nearest_relative_point.x;
            odom_msg.pose.pose.position.y = nearest_relative_point.y;
            odom_msg.pose.pose.position.z = nearest_relative_point.z;
            odom_msg.twist.twist.linear.z = min_foot_z;            
            odom_msg.pose.pose.orientation.w = 1.0;  // 无旋转

            // 方向（四元数）
            odom_msg.pose.pose.orientation = q_msg;

            pub_nearest_odom.publish(odom_msg);
        }
        else{
            // 发布nav_msgs/Odometry消息，最近点相对base_link
            nav_msgs::Odometry odom_msg;
            odom_msg.header = msg->markers.front().header;
            odom_msg.header.frame_id = "odom";
            odom_msg.child_frame_id = "nearest_point";

            odom_msg.pose.pose.position.x = 0;
            odom_msg.pose.pose.position.y = 0;
            odom_msg.pose.pose.position.z = 0;
            odom_msg.twist.twist.linear.z = min_foot_z;   
                     
            odom_msg.pose.pose.orientation.x = 0.0;
            odom_msg.pose.pose.orientation.y = 0.0;
            odom_msg.pose.pose.orientation.z = 0.0;
            odom_msg.pose.pose.orientation.w = 1.0;

            pub_nearest_odom.publish(odom_msg);
        }


    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "convex_hull_from_marker");
    ros::NodeHandle nh("~");

    nh.param("resolution", g_resolution, 0.01f);
    nh.param("range_xy", g_range_xy, 3.0f);
    nh.param("erosion_rounds", g_erosion_rounds, 3);
    nh.param("erosion_rounds_above_below", g_erosion_rounds_above_below, 1);
    nh.param("z_threshold", g_z_threshold, 0.3f);
    nh.param("layer_height", g_layer_height, 0.05f);
    nh.param("foot_layer_delta", foot_layer_delta, 0.1f);

    // ros::Subscriber sub = nh.subscribe("/polygon_markers", 10, markerArrayCallback);
    ros::Subscriber sub = nh.subscribe("/polygon_new_markers", 10, markerArrayCallback);

    pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("/convex_hull_filled", 10);
    pub_filtered = nh.advertise<sensor_msgs::PointCloud2>("/convex_hull_filtered", 10);
    pub_max_z = nh.advertise<sensor_msgs::PointCloud2>("/convex_hull_max_z_eroded", 10);
    pub_rectangles = nh.advertise<visualization_msgs::Marker>("nearest_rectangles", 10);
    pub_rectangles2 = nh.advertise<visualization_msgs::Marker>("nearest_rectangles2", 10);
    pub_foot_layer = nh.advertise<sensor_msgs::PointCloud2>("/convex_hull_foot_layer_eroded", 10);

    pub_nearest_odom = nh.advertise<nav_msgs::Odometry>("/nearest_point", 10);

    tf2_ros::TransformListener tf_listener(tf_buffer);

    ros::spin();

    return 0;
}
