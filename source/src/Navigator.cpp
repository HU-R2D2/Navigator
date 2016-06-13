//! \addtogroup	0033 Navigator
//! \brief
//!
//!
//! \file   Navigator.cpp
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

#include "../include/Navigator.hpp"

namespace r2d2{
    Navigator::Navigator(Pilot & pilot,
        PathFinder & path_finder,
        CoordinateAttitude & robot_coordinate_attitude):
            pilot(pilot),
            path_finder(path_finder),
            robot_coordinate_attitude(robot_coordinate_attitude),
            goal(goal_object),
            path(p){
    }

    bool Navigator::set_goal(CoordinateAttitude new_goal){
        LockingSharedObject<CoordinateAttitude>::Accessor goal_accessor(goal);
        CoordinateAttitude & goal_data  = goal_accessor.access();
        goal_data = new_goal;

        LockingSharedObject<CoordinateAttitude>::Accessor
            robot_orientation_accessor(robot_coordinate_attitude);
            CoordinateAttitude & robot_orientation_data =
            robot_orientation_accessor.access();

            std::vector<Coordinate> & path_vector =
            LockingSharedObject<std::vector<Coordinate>>::Accessor(path)
            .access();

        if(path_finder.get_path_to_coordinate(robot_orientation_data.coordinate,
            new_goal.coordinate, path_vector)){
                return true;
        }
        return false;
    }

    LockingSharedObject<CoordinateAttitude> & Navigator::get_goal(){
        return goal;
    }

    CoordinateAttitude Navigator::get_current_waypoint(){
        return current_waypoint;
    }
}
