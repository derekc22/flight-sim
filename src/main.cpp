#include "core/messages/messages.hpp"
#include <iostream>

int main() {
    messages::Vector3D position(1.0, 2.0, 3.0);
    messages::Quaternion attitude;
    messages::Vector3D velocity;
    messages::Vector3D angular_velocity;
    
    messages::DroneState drone(position, velocity, attitude, angular_velocity);
    
    std::cout << "FPV Simulator initialized!" << std::endl;
    
    return 0;
}
