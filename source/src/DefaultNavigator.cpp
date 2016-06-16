//! \addtogroup	0033 Navigator
//! \brief By introducing the navigator module to R2D2 links between different
//! parts of r2d2 which are responsible for moving the robot will be possible.
//! The navigator module is resposible for ploting courses from pilots position
//! to destination goal.
//!
//!
//! \file   DefaultNavigator.cpp
//! \author Anas Shehata 1651951
//! \author Timon van den Brink 1664554
//! \date   Created: 06-06-2016
//! \date   Last Modified: 06-06-2016
//! \brief  In this file the Implementation of the DefaultNavigator
//! can be found. The Implementation of Navigator is an active one.
//!
//!
//! \copyright Copyright � 2016, HU University of Applied Sciences Utrecht.
//! All rights reserved.
//!
//! License: newBSD
//!
//! Redistribution and use in source and binary forms,
//! with or without modification, are permitted provided that
//! the following conditions are met:
//! - Redistributions of source code must retain the above copyright notice,
//!   this list of conditions and the following disclaimer.
//! - Redistributions in binary form must reproduce the above copyright notice,
//!   this list of conditions and the following disclaimer in the documentation
//!   and/or other materials provided with the distribution.
//! - Neither the name of the HU University of Applied Sciences Utrecht
//!   nor the names of its contributors may be used to endorse or promote
//!   products derived from this software without specific prior written
//!   permission.
//!
//! THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//! "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING,
//! BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
//! AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
//! IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED SCIENCES UTRECHT
//! BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
//! CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
//! PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
//! OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
//! WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
//! OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
//! EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// ~< HEADER_VERSION 2016 04 12 >~

#include "../include/DefaultNavigator.hpp"

namespace r2d2{

    DefaultNavigator::DefaultNavigator(Pilot & pilot,
        PathFinder & path_finder,
        CoordinateAttitude & robot_coordinate_attitude,
        CoordinateAttitude waypoint_precision_margin):
            Navigator(pilot, path_finder, robot_coordinate_attitude),
            waypoint_precision_margin(waypoint_precision_margin){}

    void DefaultNavigator::update(){
        // Check if the pilot is in range of the waypoint
        if(has_reached_waypoint()){
            std::vector<Coordinate> & path_vector = LockingSharedObject<std
                ::vector<Coordinate>>::Accessor(path).access();
            Pilot & p = SharedObject<Pilot>::Accessor(pilot).access();
            p.stop();
            // check if the path is not empty, so the next waypoint can be
            // shifted to the pilot.
            if (!path_vector.empty()){
                CoordinateAttitude & coordinate_attitude_data =
                    SharedObject<CoordinateAttitude>
                    ::Accessor(robot_coordinate_attitude).access();
                Angle angle;
                // check if the path vector contains more than only the last
                // waypoint. If this is true calculate the angle between the
                // next waypoint and the waypoint after the next waypoint.
                if (path_vector.size() > 1){
                    angle = atan2((path_vector[1].get_y() -
                        current_waypoint.coordinate.get_y()) / Length
                        ::METER, (path_vector[1].get_x() -
                        current_waypoint.coordinate.get_x()) / Length
                        ::METER) * Angle::rad;
                }
                // if the path contains only 1 element, assign the goal angle
                // to the new waypoint angle.
                else {
                    angle = SharedObject<CoordinateAttitude>::Accessor(
                        get_goal()).access().attitude.angle_z.get_angle()
                        * Angle::rad;
                }

                // combine the Coordinate from the path and the obtained angle
                // to an CoordinateAttitude and assign this to the
                // current_waypoint.
                current_waypoint = {path_vector[0], Attitude(0 * Angle
                        ::rad, 0 * Angle::rad, angle)};
                path_vector.erase(path_vector.begin());

                // Then shift it to the pilot.
                p.go_to_position(current_waypoint);
            }
        }
    }

    void DefaultNavigator::run(){
        int period_ms = 100;
        while(true){
            std::this_thread::sleep_for(std::chrono::milliseconds(period_ms));
            update();
        }
    }

    bool DefaultNavigator::has_reached_waypoint(){
        CoordinateAttitude & coordinate_attitude_data =
            SharedObject<CoordinateAttitude>
            ::Accessor(robot_coordinate_attitude).access();

        // check if the x coordinate is in range of the precision_margin
        if(in_range(
                coordinate_attitude_data.coordinate.get_x(),
                current_waypoint.coordinate.get_x(),
                waypoint_precision_margin.coordinate.get_x()
            )){
            // check if the y coordinate is in range of the precision_margin
            if(in_range(
                    coordinate_attitude_data.coordinate.get_y(),
                    current_waypoint.coordinate.get_y(),
                    waypoint_precision_margin.coordinate.get_y()
                )){
                // check if the attitude yaw is in range of the percision_margin

                // TODO remove attitude.angle_z (stub from Stubs.hpp in Pilot)
                // to attitude.get_yaw()
                if(in_range(
                        coordinate_attitude_data.attitude.angle_z,
                        current_waypoint.attitude.angle_z,
                        waypoint_precision_margin.attitude.angle_z
                    )){

                    return true;
                }
            }
        }
        return false;
    }
    //function which checks if length_1 is between length_2 + precision_margin and length_2 - precision_margin
    bool DefaultNavigator::in_range(Length length_1, Length length_2, Length precision_margin){
            return ((length_2 - precision_margin) < length_1) &&
                (length_1 < (length_2 + precision_margin));
    }

    //function which checks if angle_1 is between angle_2 + precision_margin and angle_2 - precision_margin
    bool DefaultNavigator::in_range(Angle angle_1, Angle angle_2, Angle precision_margin){
            return ((angle_2 - precision_margin) < angle_1) &&
                (angle_1 < (angle_2 + precision_margin));
    }
}
