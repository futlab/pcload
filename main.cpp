#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
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
    if (argc <= 1) {
        printf(".bag file needed");
        return 1;
    }
    Bag bag;
    bag.open(argv[1], bagmode::Read);
    const vector<string> topics = {
        "/scan",
        //"/mavros/imu/data",
        "/mavros/local_position/pose",
        "/encoder"
    };
    const double encoderStep = 0.1;

    View view(bag, TopicQuery(topics));
    PointCloud<PointXYZ> cloud;
    cloud.width    = 0;
    cloud.height   = 1080;
    cloud.is_dense = false;

    Matrix3d orientation = Matrix3d::Identity();
    Vector3d position = Vector3d::Zero();
    double xPrev = 0.0;

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
        geometry_msgs::Twist::ConstPtr twist = m.instantiate<geometry_msgs::Twist>();
        if (ps != nullptr) {
            double x = twist->linear.x, dx = x - xPrev;
            xPrev = x;
            position += (dx * encoderStep) * orientation.block<3, 1>(0, 0);
        }
    }
    bag.close();
    io::savePCDFile("result.pcd", cloud);
}
