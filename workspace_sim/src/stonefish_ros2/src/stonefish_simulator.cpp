/*    
    This file is a part of stonefish_ros2.

    stonefish_ros is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    stonefish_ros is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

//
//  stonefish_simulator.cpp
//  stonefish_ros2
//
//  Created by Patryk Cieslak on 02/10/23.
//  Copyright (c) 2023-2024 Patryk Cieslak. All rights reserved.
//

#include "rclcpp/rclcpp.hpp"
#include <algorithm>
#include <cmath>
#include "std_msgs/msg/float64_multi_array.hpp"
#include <Stonefish/utils/SystemUtil.hpp>
#include "stonefish_ros2/ROS2SimulationManager.h"
#include "stonefish_ros2/ROS2GraphicalSimulationApp.h"

using namespace std::chrono_literals;

class StonefishNode : public rclcpp::Node
{
public:
    StonefishNode(const std::string& scenarioPath,
                           const std::string& dataPath, 
                           const sf::RenderSettings& s, 
                           const sf::HelperSettings& h,
                           sf::Scalar rate) 
                           : Node("stonefish_simulator")
    {   
        sf::ROS2SimulationManager* manager = new sf::ROS2SimulationManager(rate, scenarioPath, std::shared_ptr<rclcpp::Node>(this));
        app_ = std::shared_ptr<sf::ROS2GraphicalSimulationApp>(new sf::ROS2GraphicalSimulationApp("Stonefish Simulator", 
                                                                                                 dataPath, s, h, manager));
        app_->Startup();
        const double requestedFps = this->declare_parameter<double>("render_fps", 30.0);
        const double renderFps = std::isfinite(requestedFps)
            ? std::clamp(requestedFps, 5.0, 120.0) : 30.0;
        tickTimer_ = this->create_wall_timer(
            std::chrono::microseconds(static_cast<int64_t>(1e6 / renderFps)),
            std::bind(&sf::ROS2GraphicalSimulationApp::Tick, app_));
        RCLCPP_INFO(get_logger(), "Render limit %.1f FPS; physics %.1f Hz", renderFps, double(rate));
        performancePub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/sim/performance", 1);
        performanceLastWall_ = std::chrono::steady_clock::now();
        performanceTimer_ = create_wall_timer(5s, [this, manager]() {
            const auto now = std::chrono::steady_clock::now();
            const double simTime = manager->getSimulationTime();
            const double wallSeconds = std::chrono::duration<double>(
                now - performanceLastWall_).count();
            const double realTimeFactor = (simTime - performanceLastSim_) /
                std::max(1e-6, wallSeconds);
            std_msgs::msg::Float64MultiArray msg;
            msg.data = {simTime, realTimeFactor};
            performancePub_->publish(msg);
            RCLCPP_INFO(get_logger(), "Simulation time=%.2fs, real_time_factor=%.3f", simTime, realTimeFactor);
            performanceLastWall_ = now;
            performanceLastSim_ = simTime;
        });
    };

private:
    std::shared_ptr<sf::ROS2GraphicalSimulationApp> app_;
    rclcpp::TimerBase::SharedPtr tickTimer_;
    rclcpp::TimerBase::SharedPtr performanceTimer_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr performancePub_;
    std::chrono::steady_clock::time_point performanceLastWall_;
    double performanceLastSim_ = 0.0;
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    
    //Check number of command line arguments
	if(argc < 7)
	{
		std::cout << "Not enough command-line arguments provided!" << std::endl;
		exit(-1);
	}

    //Parse the arguments
    std::string dataPath = std::string(argv[1]) + "/";
    std::string scenarioPath(argv[2]);
    sf::Scalar rate = atof(argv[3]);

	sf::RenderSettings s;
    s.windowW = atoi(argv[4]);
    s.windowH = atoi(argv[5]);

    std::string quality(argv[6]);
    if(quality == "low")
    {
        s.shadows = sf::RenderQuality::LOW;
        s.ao = sf::RenderQuality::DISABLED;
        s.atmosphere = sf::RenderQuality::LOW;
        s.ocean = sf::RenderQuality::LOW;
        s.aa = sf::RenderQuality::LOW;
        s.ssr = sf::RenderQuality::DISABLED;
    }
    else if(quality == "high")
    {
        s.shadows = sf::RenderQuality::HIGH;
        s.ao = sf::RenderQuality::HIGH;
        s.atmosphere = sf::RenderQuality::HIGH;
        s.ocean = sf::RenderQuality::HIGH;
        s.aa = sf::RenderQuality::HIGH;
        s.ssr = sf::RenderQuality::HIGH;
    }
    else // "medium"
    {
        s.shadows = sf::RenderQuality::MEDIUM;
        s.ao = sf::RenderQuality::MEDIUM;
        s.atmosphere = sf::RenderQuality::MEDIUM;
        s.ocean = sf::RenderQuality::MEDIUM;
        s.aa = sf::RenderQuality::MEDIUM;
        s.ssr = sf::RenderQuality::MEDIUM;
    }

    // Initialize GUI settings
    sf::HelperSettings h;
    h.showFluidDynamics = false;
    h.showCoordSys = false;
    h.showBulletDebugInfo = false;
    h.showSensors = false;
    h.showActuators = false;
    h.showForces = false;
	
    // Start simulation
    std::shared_ptr<StonefishNode> node(new StonefishNode(scenarioPath, dataPath, s, h, rate));
    rclcpp::spin(node);
    return 0;
}
