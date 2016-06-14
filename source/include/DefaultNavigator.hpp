//! \addtogroup	0033 Navigator
//! \brief By introducing the navigator module to R2D2 links between different
//! parts of r2d2 which are responsible for moving the robot will be possible.
//! The navigator module is resposible for ploting courses from pilots position
//! to destination goal.
//!
//!
//! \file   DefaultNavigator.hpp
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

#ifndef R2D2_DEFAULTNAVIGATOR_HPP
#define R2D2_DEFAULTNAVIGATOR_HPP

#include <chrono>
#include <thread>

#include "Navigator.hpp"

namespace r2d2{

    //!
    //! @author     Anas Shehata 1651951
    //! @author     Timon van den Brink 1664554
	//! @date       06-06-16
	//! @version    1.0
	//! @brief      The DefaultNavigator class is an active impementation
    //!             of the class Navigator. DefaultNavigator is responsible
    //!             for active coordination of the Pilot.
	//!             It contains 2 CoordinateAttitude attributes,
    //!             1 pathfinder attribute, 1 vector attribute.
    //!             and 1 pilot attribute.
	//!
    class DefaultNavigator : public Navigator{
    public:

        //!
        //!@brief     Constructor for DefaultNavigator
        //!@param     pilot    Pilot object that is to be navigated
        //!@param     path_finder     PathFinder object which calculates a path
        //!           between two points
        //!@param     robot_coordinate_attitude    CoordinateAttitude object
        //!           which contains Pilot currents coordinates
        //!@param     waypoint_precision_margin    CoordinateAttitude object
        //!           which is the precision_margin of every waypoint
        //!
        DefaultNavigator(Pilot & pilot,
            PathFinder & path_finder,
            CoordinateAttitude & robot_coordinate_attitude,
			CoordinateAttitude waypoint_precision_margin);

        //!
    	//! @brief     responsible for the navigation of the pilot object to the
        //!            destined goal.
        //!
        void update() override;

        //!
        //! @brief     Function that periodically updates the DefaultNavigator
        //!
        void run();

	private:
	    bool has_reached_waypoint();
		CoordinateAttitude waypoint_precision_margin;
    };
}

#endif
