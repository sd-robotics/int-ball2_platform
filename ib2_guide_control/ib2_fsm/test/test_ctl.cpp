
#include "ib2_ctl_common/Log.h"

#include <gtest/gtest.h>
// #include <rclcpp/rclcpp.hpp>

#include <cmath>


TEST(CtlTest, rotRoll)
{

}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    using namespace ib2_mss;
    Log::configure("log/test_ctl.log", "DEBUG");
    testing::InitGoogleTest(&argc, argv);
    // rclcpp::init(argc, argv);
    return RUN_ALL_TESTS();
}

// End Of File -----------------------------------------------------------------
