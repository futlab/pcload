#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
//#include <pcl_conversions/

int main(int argc, char *argv[])
{
    using namespace std;
    using namespace rosbag;
    using namespace pcl;
    using namespace Eigen;
    if (!argc) {
        printf(".bag file needed");
        return 1;
    }
    Bag bag;
    bag.open(argv[1], bagmode::Read);
    vector<string> topics = {
        "/scan",
        //"/mavros/imu/data",
        "/mavros/local_position/pose"
    };

    View view(bag, TopicQuery(topics));
    PointCloud<PointXYZ> cloud;
    cloud.width    = 0;
    cloud.height   = 1080;
    cloud.is_dense = false;

    Matrix3d orientation = Matrix3d::Identity();
    Vector3d position = Vector3d::Zero();


    for (MessageInstance const m : view) {
        sensor_msgs::LaserScan::ConstPtr ls = m.instantiate<sensor_msgs::LaserScan>();
        if (ls != nullptr) {
            Vector3d ray = AngleAxisd(ls->angle_min, Vector3d::UnitY()).toRotationMatrix() * Vector3d::UnitZ();
            Matrix3d rotationStep = AngleAxisd(ls->angle_increment, Vector3d::UnitY()).toRotationMatrix();
            for (float range : ls->ranges) {
                auto point = position + orientation * ray * range;
                cloud.points.emplace_back(point[0], point[1], point[2]);
                ray = rotationStep * ray;
            }
            cloud.height = ls->ranges.size();
            cloud.width++;
        }
        geometry_msgs::PoseStamped::ConstPtr ps = m.instantiate<geometry_msgs::PoseStamped>();
        if (ps != nullptr) {
            const auto &o = ps->pose.orientation;
            orientation = Quaterniond(o.w, o.x, o.y, o.z).toRotationMatrix();
        }
    }
    bag.close();
    io::savePCDFile("result.pcd", cloud);
}
