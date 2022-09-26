#include <gtest/gtest.h>

#include "mock_rover/mock_rover.hpp"

static float TICK_RATE_HZ = 10.0;

TEST(UnicycleModelTest, ConstructModel)
{
    UnicycleModel um(TICK_RATE_HZ);

    // We didn't puke, so call it good.
    ASSERT_TRUE(true);
}

TEST(UnicycleModelTest, ZeroToOne) {
    UnicycleModel um(TICK_RATE_HZ);

    // Should be stopped, and going nowhere.
    ASSERT_TRUE(um.get_vehicle_state().x_dot == 0.0);
    ASSERT_TRUE(um.get_vehicle_state().x == 0.0);

    ////////
    // Set a non-zero linear velocity, and step the model forward.
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 1.0; // 1 m/s forward

    // Should take this long to reach 1.0 m/s, if t = v_final / a_max
    double t_total = cmd_vel.linear.x / MAX_LINEAR_ACC;
    int n_ticks = static_cast<int>(t_total * um.get_tick_rate()); // ...which is this many steps.
    um.update_cmd_vel(cmd_vel);
    for(int i = 0; i < n_ticks; ++i)
    {
        um.update_model();
        std::cout << "x_dot: "<< um.get_vehicle_state().x_dot << " "
                  << "x: " << um.get_vehicle_state().x << "\n";
    }
    double vx = um.get_vehicle_state().x_dot;
    ASSERT_TRUE(vx == 1.0);

    // Now, set a command velocity of zero and stop the rover.
    cmd_vel.linear.x = 0.0; // stop
    um.update_cmd_vel(cmd_vel);
    for(int i = 0; i < n_ticks; ++i)
    {
        um.update_model();
        std::cout << "x_dot: "<< um.get_vehicle_state().x_dot << " "
                  << "x: " << um.get_vehicle_state().x << "\n";
    }

    // Should be stopped now.
    vx = um.get_vehicle_state().x_dot;
    ASSERT_TRUE(vx == 0.0);
}

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
