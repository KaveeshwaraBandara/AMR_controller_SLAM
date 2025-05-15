#include <gz/transport/Node.hh>
#include <gz/msgs/twist.pb.h>
#include <thread>
#include <chrono>
#include <iostream>

int main()
{
    // Create Gazebo Transport node
    gz::transport::Node node;

    // Advertise the velocity topic
    auto velPub = node.Advertise<gz::msgs::Twist>("/model/tugbot/cmd_vel");

    // Give Gazebo some time to connect
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    std::cout << "Publishing velocity commands to /model/tugbot/cmd_vel..." << std::endl;

    // Create a Twist message to move forward
    gz::msgs::Twist moveCmd;
    moveCmd.mutable_linear()->set_x(0.5);  // Forward
    moveCmd.mutable_angular()->set_z(0.0); // No rotation

    // Send the move command for 3 seconds
    for (int i = 0; i < 30; ++i)
    {
        velPub.Publish(moveCmd);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Stop the robot
    gz::msgs::Twist stopCmd;
    stopCmd.mutable_linear()->set_x(0.0);
    stopCmd.mutable_angular()->set_z(0.0);
    velPub.Publish(stopCmd);

    std::cout << "Robot stopped." << std::endl;

    return 0;
}

