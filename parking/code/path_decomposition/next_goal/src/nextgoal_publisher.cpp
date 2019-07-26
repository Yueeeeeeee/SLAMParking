#include <ros/ros.h>
#include <cmath>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

class NextGoalPublisher {
    ros::NodeHandle n;
    ros::Subscriber path_sub;
    ros::Publisher nextgoal_pub;

    geometry_msgs::PoseStamped output_msg;
    geometry_msgs::PoseStamped temp;

    void pathCallback(const nav_msgs::Path::ConstPtr& msg) {
        int i = 0;
        temp = msg->poses.front();

        while(std::abs(temp.pose.position.x - output_msg.pose.position.x) < 0.2 && std::abs(temp.pose.position.y - output_msg.pose.position.y) < 0.2) {
            temp = msg->poses[i];
            i++;
        }

        //TODO: what should happen if local path destination are too close
        output_msg = temp;
        nextgoal_pub.publish(output_msg);
    }

    public:
    NextGoalPublisher() {
        path_sub = n.subscribe<nav_msgs::Path>("move_base/TrajectoryPlannerROS/global_plan", 5, &NextGoalPublisher::pathCallback, this);
        nextgoal_pub = n.advertise<geometry_msgs::PoseStamped>("nextgoal", 5);
    }

    ~NextGoalPublisher() {
    }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "nextgoal_publisher");
    NextGoalPublisher next_goal;
    
    ros::spin();
    ros::shutdown();
    return 0;
}
