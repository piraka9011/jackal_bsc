#include <ros/ros.h>
#include <tf/transform_broadcaster.h>


int main(int argc, char** argv) {
    ros::init(argc, argv, "kinect_tf_publisher");

    // Defs
    static tf::TransformBroadcaster  br;
    std::vector<tf::StampedTransform> tf_vector;
    tf::StampedTransform tf_front_mount_to_frame, tf_frame_to_ir, tf_frame_to_rgb;
    tf::Quaternion q;
    std::string front_mount = "front_mount";
    std::string kinect_link = "kinect2_link";
    std::string rgb_link = "kinect2_rgb_optical_frame";
    std::string ir_link = "kinect2_ir_optical_frame";

    // Default rotation
    q.setRPY(0, 0, 0);

    // Translations
    tf_front_mount_to_frame.setOrigin(tf::Vector3(0.11715, 0, 0.125));
    tf_front_mount_to_frame.setRotation(q);
    tf_front_mount_to_frame.frame_id_ = front_mount;
    tf_front_mount_to_frame.child_frame_id_ = kinect_link;

    tf_frame_to_rgb.setOrigin(tf::Vector3(0, -0.1, 0));
    tf_frame_to_rgb.setRotation(q);
    tf_frame_to_rgb.frame_id_= kinect_link;
    tf_frame_to_rgb.child_frame_id_ = rgb_link;

    tf_frame_to_ir.setOrigin(tf::Vector3(0, -0.06, 0));
    tf_frame_to_ir.setRotation(q);
    tf_frame_to_ir.frame_id_ = kinect_link;
    tf_frame_to_ir.child_frame_id_ = ir_link;

    ROS_INFO("[Kinect TF]: Broadcasting...");
    ros::Rate r(5);
    while (ros::ok()) {
        tf_vector.clear();
        tf_front_mount_to_frame.stamp_ = ros::Time::now();
        tf_frame_to_rgb.stamp_ = ros::Time::now();
        tf_frame_to_ir.stamp_ = ros::Time::now();
        tf_vector.push_back(tf_front_mount_to_frame);
        tf_vector.push_back(tf_frame_to_ir);
        tf_vector.push_back(tf_frame_to_rgb);
        br.sendTransform(tf_vector);
        r.sleep();
    }

    return 0;
}
