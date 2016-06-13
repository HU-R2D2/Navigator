//! \addtogroup	0033 Navigator
//! \brief
//!
//!
//! \file   Navigator.hpp
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

#ifndef R2D2_NAVIGATOR_HPP
#define R2D2_NAVIGATOR_HPP

#include <vector>
#include <iostream>

#include <Pilot.hpp>
#include <AStarPathFinder.hpp>
#include <LockingSharedObject.hpp>
#include <Stubs.hpp>

namespace r2d2{

    //!
    //! @author     Anas Shehata 1651951
    //! @author     Timon van den Brink 1664554
    //! @date       06-06-16
    //! @version    1.0
    //! @brief      The Navigator class is an interface class for Navigators
    //!             responsible for plotting the course of a given path.
    //!             It contains 2 CoordinateAttitude attributes,
    //!             1 pathfinder attribute, 1 vector attribute.
    //!             and 1 pilot attribute.
    //!
    class Navigator{
    public:

        //!
        //!@brief     Constructor for Navigator
        //!@param     pilot    Pilot object that is to be navigated
        //!@param     path_finder     PathFinder object which calculates a path
        //!           between two points
        //!@param     robot_coordinate_attitude    CoordinateAttitude object
        //!           which contains Pilot currents coordinates
        //!@param     waypoint_precision_margin    CoordinateAttitude object
        //!           which is the precision_margin of every waypoint
        //!
        //!
        Navigator(Pilot & pilot,
            PathFinder & path_finder,
            CoordinateAttitude & robot_coordinate_attitude);

        //!
        //! @brief     responsible for the navigation of the pilot object to the
        //!            destined goal.
        //!
        virtual void update() = 0;


        //!
        //! @brief     sets new Coordinates for destination goal
        //! @param     goal     CoordinatesAttitude of the destination goal
        //!
        bool set_goal(CoordinateAttitude goal);


        //!
        //! @brief  Getter for goal CoordinateAttitude.
        //! @return LockingSharedObject<CoordinateAttitude> goal
        //!
        LockingSharedObject<CoordinateAttitude> & get_goal();


        //!
        //! @brief     getter for the current_waypoint in the path of Navigator.
        //!            returns a copy of the current_waypoint
        //!
        //! @return    CoordinateAttitude current_waypoint
        //!
        CoordinateAttitude get_current_waypoint();
    protected:
        LockingSharedObject<Pilot> pilot;
        PathFinder & path_finder;
        LockingSharedObject<CoordinateAttitude> robot_coordinate_attitude;
        LockingSharedObject<CoordinateAttitude> goal;
        LockingSharedObject<std::vector<Coordinate>> path;
        CoordinateAttitude current_waypoint{Coordinate(), Attitude()};
    private:
        std::vector<Coordinate> p;
        CoordinateAttitude goal_object = {Coordinate::origin, Attitude()};
    };
}

#endif
