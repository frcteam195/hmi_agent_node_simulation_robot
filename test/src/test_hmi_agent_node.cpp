#include "test_hmi_agent_node.hpp"
#include "hmi_agent_node.hpp"
#include "ros/ros.h"

#include <gtest/gtest.h>

TEST(SampleTest, Test_Test)
{
    ASSERT_TRUE(1);
    ASSERT_FALSE(0);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_hmi_agent_node");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}