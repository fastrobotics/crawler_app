#include <actionlib/client/simple_action_client.h>
#include <eros/system_commandAction.h>
#include <gtest/gtest.h>
#include <ros/ros.h>

#include "../TankDriveNode.h"

using namespace crawler_app;

std::string robot_namespace = "/test/";
std::string unittest_nodename = "tankdrive_node";
typedef actionlib::SimpleActionClient<eros::system_commandAction> CommandActionClient;
uint64_t heartbeat_count = 0;
eros::heartbeat latest_heartbeat;
void heartbeat_Callback(const eros::heartbeat& msg) {
    latest_heartbeat = msg;
    heartbeat_count++;
}
uint64_t left_channel_rx_count = 0;
void leftchannel_Callback(const std_msgs::UInt16& /* msg */) {
    left_channel_rx_count++;
}
uint64_t right_channel_rx_count = 0;
void rightchannel_Callback(const std_msgs::UInt16& /* msg */) {
    right_channel_rx_count++;
}
TEST(TankDriveNode, TestBasics) {
    ros::NodeHandle nh("~");
    eros::Logger* logger = new eros::Logger("DEBUG", "test_TankDriveNode");
    logger->enable_ROS_logger();
    std::string heartbeat_topic = robot_namespace + unittest_nodename + "/heartbeat";
    ros::Subscriber heartbeat_sub = nh.subscribe(heartbeat_topic, 100, &heartbeat_Callback);
    ros::Publisher cmd_vel_pub =
        nh.advertise<geometry_msgs::Twist>(robot_namespace + "cmd_vel_perc", 20);

    std::string leftchannel_topic = robot_namespace + "/left_drive";
    ros::Subscriber leftdrive_sub = nh.subscribe(leftchannel_topic, 100, &leftchannel_Callback);
    std::string rightchannel_topic = robot_namespace + "/right_drive";
    ros::Subscriber rightdrive_sub = nh.subscribe(rightchannel_topic, 100, &rightchannel_Callback);
    sleep(5.0);
    EXPECT_NE(ros::topic::waitForMessage<eros::heartbeat>(heartbeat_topic, ros::Duration(10)),
              nullptr);
    EXPECT_EQ(1, heartbeat_sub.getNumPublishers());
    EXPECT_EQ(1, cmd_vel_pub.getNumSubscribers());
    // EXPECT_NE(ros::topic::waitForMessage<std_msgs::UInt16>(leftchannel_topic, ros::Duration(10)),
    //          nullptr);
    EXPECT_EQ(1, leftdrive_sub.getNumPublishers());
    // EXPECT_NE(ros::topic::waitForMessage<std_msgs::UInt16>(rightchannel_topic,
    // ros::Duration(10)),
    //           nullptr);
    EXPECT_EQ(1, rightdrive_sub.getNumPublishers());

    EXPECT_TRUE(heartbeat_count > 0);
    EXPECT_TRUE(left_channel_rx_count > 0);
    EXPECT_TRUE(right_channel_rx_count > 0);

    logger->log_warn("Testing Unsupported Commands...");
    {  // System Command not currently supported by Node.
        CommandActionClient client(robot_namespace + "SystemCommandAction", true);
        EXPECT_TRUE(client.waitForServer());
        eros::system_commandGoal cmd;
        client.sendGoal(cmd);
        client.waitForResult(ros::Duration(1.0));
        EXPECT_EQ(client.getState(), actionlib::SimpleClientGoalState::ABORTED);
    }
    {
        ros::Publisher command_pub =
            nh.advertise<eros::command>(robot_namespace + "SystemCommand", 20);
        sleep(1);
        EXPECT_EQ(1, command_pub.getNumSubscribers());
        eros::command snapshot_command;
        command_pub.publish(snapshot_command);
        sleep(1.0);
    }
    logger->log_notice("Basic Test of Node State Change...");
    {
        ros::ServiceClient client = nh.serviceClient<eros::srv_change_nodestate>(
            robot_namespace + unittest_nodename + "/srv_nodestate_change");
        eros::srv_change_nodestate req;
        req.request.RequestedNodeState = "PAUSED";
        EXPECT_TRUE(client.call(req));
        sleep(1);
        EXPECT_EQ((uint8_t)eros::Node::State::PAUSED, latest_heartbeat.NodeState);
        req.request.RequestedNodeState = "RUNNING";
        EXPECT_TRUE(client.call(req));
        sleep(1);
        EXPECT_EQ((uint8_t)eros::Node::State::RUNNING, latest_heartbeat.NodeState);

        req.request.RequestedNodeState = "RESET";
        EXPECT_TRUE(client.call(req));
        sleep(5);
        EXPECT_EQ((uint8_t)eros::Node::State::RUNNING,
                  latest_heartbeat.NodeState);  // Node should automatically change state to Running
    }

    delete logger;
}
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_TankDriveNode");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    int ret = RUN_ALL_TESTS();
    spinner.stop();
    ros::shutdown();
    return ret;
}
