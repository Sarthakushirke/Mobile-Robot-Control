#include <gtest/gtest.h>
#include <cmath>

#include "./tools.h"

TEST(ToolTest, wrap2PiTest)
{
    EXPECT_NEAR(wrapToPi(0 * M_PI), 0 * M_PI, 1e-6);
    EXPECT_NEAR(wrapToPi(0.5 * M_PI), 0.5 * M_PI, 1e-6);
    EXPECT_NEAR(wrapToPi(-0.5 * M_PI), -0.5 * M_PI, 1e-6);

    EXPECT_NEAR(wrapToPi(2.99 * M_PI), 0.99 * M_PI, 1e-6);
    EXPECT_NEAR(wrapToPi(500 * M_PI), 0 * M_PI, 1e-6);

    EXPECT_NEAR(wrapToPi(-2.99 * M_PI), -0.99 * M_PI, 1e-6);
    EXPECT_NEAR(wrapToPi(-500 * M_PI), 0 * M_PI, 1e-6);
}

TEST(ToolTest, constructNoFail)
{
    int argc = 1;
    char arg0[] = "./main.cpp";
    char *argv[] = {&arg0[0]};

    auto programConfig = load_config_file(argc, argv);
    Planner planner = constructPlanner(programConfig);
}