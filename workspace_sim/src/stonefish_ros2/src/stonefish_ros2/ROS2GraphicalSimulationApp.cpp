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
//  ROS2GraphicalSimulationApp.cpp
//  stonefish_ros2
//
//  Created by Patryk Cieslak on 02/10/23.
//  Copyright (c) 2023-2025 Patryk Cieslak. All rights reserved.
//

#include "stonefish_ros2/ROS2SimulationManager.h"
#include "stonefish_ros2/ROS2GraphicalSimulationApp.h"

#include <Stonefish/graphics/OpenGLTrackball.h>
#include <glm/gtx/quaternion.hpp>

namespace sf
{

ROS2GraphicalSimulationApp::ROS2GraphicalSimulationApp(std::string title, std::string dataPath, RenderSettings s, HelperSettings h, ROS2SimulationManager* sim)
    : GraphicalSimulationApp(title, dataPath, s, h, sim), sim_(sim), configuredTrackball_(nullptr)
{
}

void ROS2GraphicalSimulationApp::ConfigureInitialPoolView(OpenGLTrackball* trackball)
{
    // The Stonefish default trackball is created around (0, 0, -1) with a
    // 5 m orbit radius.  The fixed Guoshui 2026 pool occupies
    // x=[0, 9.5], y=[-2.25, 2.25], z=[0, 1.3].
    constexpr glm::vec3 defaultCenter(0.0f, 0.0f, -1.0f);
    constexpr glm::vec3 poolCenter(4.75f, 0.0f, 0.65f);

    // Move the orbit target to the geometric centre of the pool.
    trackball->MoveCenter(poolCenter - defaultCenter);

    // OpenGLTrackball looks along its local -Y axis.  Construct the
    // relative rotation which makes that axis point along +Z (down in NED),
    // while retaining a stable +Y image-up direction.
    const glm::quat alignUp = glm::rotation(
        glm::vec3(0.0f, 0.0f, -1.0f), glm::vec3(0.0f, 0.0f, 1.0f));
    const glm::quat topDown = glm::rotation(
        glm::vec3(0.0f, 0.0f, 1.0f), glm::vec3(0.0f, -1.0f, 0.0f));
    trackball->Rotate(glm::inverse(alignUp) * topDown);

    // Increase the default 5 m orbit to 6 m so the 9.5 m pool and its walls
    // have a visible margin in the default 16:9 window.
    trackball->MouseScroll(3.0f);
    trackball->UpdateTransform();
}

void ROS2GraphicalSimulationApp::Startup()
{
    Init();
    StartSimulation();
}

void ROS2GraphicalSimulationApp::Tick()
{
    // StartSimulation() builds the scenario on Stonefish's simulation
    // thread, so the trackball may not exist when Startup() returns.
    // Configure it on the first GUI tick after it has been created.
    if(sim_ != nullptr)
    {
        OpenGLTrackball* trackball = sim_->getTrackball();
        if(trackball != nullptr && trackball != configuredTrackball_)
        {
            ConfigureInitialPoolView(trackball);
            configuredTrackball_ = trackball;
        }
    }

    LoopInternal();
    if(state_ == SimulationState::FINISHED)
    {
        CleanUp();
        rclcpp::shutdown();
    }
}

}
