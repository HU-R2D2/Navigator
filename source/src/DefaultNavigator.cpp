//! \addtogroup	0033 Navigator
//! \brief
//!
//!
//! \file   DefaultNavigator.cpp
//! \author Anas Shehata 1651951
//! \author Timon van den Brink 1664554
//! \date   Created: 06-06-2016
//! \date   Last Modified: 06-06-2016
//! \brief  the interface Navigator is an interface for Navigators to plot the
//!          course of a given path.
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

    DefaultNavigator::DefaultNavigator(r2d2::Pilot & pilot,
        r2d2::AStarPathFinder & path_finder,
        CoordinateAttitude & robot_coordinate_attitude,
        CoordinateAttitude & goal,
        CoordinateAttitude waypoint_precision_margin):
            r2d2::Navigator(pilot, path_finder, robot_coordinate_attitude, goal),
            waypoint_precision_margin(waypoint_precision_margin){}

    void DefaultNavigator::update(){
        if(has_reached_waypoint()){
            std::vector<Coordinate> & path_vector = LockingSharedObject<std
                ::vector<Coordinate>>::Accessor(path).access();
            Pilot & p = SharedObject<Pilot>::Accessor(pilot).access();
            p.stop();
            if (!path_vector.empty()){
                CoordinateAttitude & coordinate_attitude_data =
                    SharedObject<CoordinateAttitude>
                    ::Accessor(robot_coordinate_attitude).access();
                r2d2::Angle angle;
                if (path_vector.size() > 1){
                    angle = atan2((path_vector[1].get_y() -
                        current_waypoint.coordinate.get_y()) / r2d2::Length
                        ::METER, (path_vector[1].get_x() -
                        current_waypoint.coordinate.get_x()) / r2d2::Length
                        ::METER) * r2d2::Angle::rad;
                }
                else {
                    angle = atan2(current_waypoint.coordinate.get_y()/r2d2
                        ::Length::METER, current_waypoint.coordinate.get_x() /
                        r2d2::Length::METER) * r2d2::Angle::rad;
                }

                current_waypoint = {path_vector[0], Attitude(0 * r2d2::Angle
                        ::rad, 0 * r2d2::Angle::rad, angle)};
                path_vector.erase(path_vector.begin());

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


    CoordinateAttitude DefaultNavigator::get_current_waypoint(){
        return current_waypoint;
    }

    bool DefaultNavigator::has_reached_waypoint(){
        CoordinateAttitude & coordinate_attitude_data =
            SharedObject<CoordinateAttitude>
            ::Accessor(robot_coordinate_attitude).access();

        if (coordinate_attitude_data.coordinate.get_x() <
            (current_waypoint.coordinate + r2d2::Translation{
            waypoint_precision_margin.coordinate.get_x(), 0 *
            r2d2::Length::METER, 0 * r2d2::Length::METER}).get_x() &&
            coordinate_attitude_data.coordinate.get_x() >
            (current_waypoint.coordinate -
            r2d2::Translation{waypoint_precision_margin.coordinate.get_x(),
            0 * r2d2::Length::METER, 0 * r2d2::Length::METER}).get_x()){

            if (coordinate_attitude_data.coordinate.get_y() <
                (current_waypoint.coordinate + r2d2::Translation{0 *
                r2d2::Length::METER,
                waypoint_precision_margin.coordinate.get_y(),0 *
                r2d2::Length::METER}).get_y() &&
                coordinate_attitude_data.coordinate.get_y() >
                (current_waypoint.coordinate - r2d2::Translation{0 *
                r2d2::Length::METER,
                waypoint_precision_margin.coordinate.get_y(), 0 *
                r2d2::Length::METER}).get_y()){

                if (coordinate_attitude_data.attitude.angle_z.get_angle() <
                    current_waypoint.attitude.angle_z.get_angle() +
                    waypoint_precision_margin.attitude.angle_z.get_angle() &&
                    coordinate_attitude_data.attitude.angle_z.get_angle() >
                    current_waypoint.attitude.angle_z.get_angle() -
                    waypoint_precision_margin.attitude.angle_z.get_angle()){
                        return true;
                }
            }
        }
        return false;
    }
}
