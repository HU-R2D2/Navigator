#ifndef R2D2_DEFAULTNAVIGATOR_HPP
#define R2D2_DEFAULTNAVIGATOR_HPP

#include <chrono>
#include <thread>
#include <cmath>

#include "Navigator.hpp"

namespace r2d2{

    class DefaultNavigator : public Navigator{
    public:
        DefaultNavigator(r2d2::Pilot & pilot,
            r2d2::AStarPathFinder & path_finder,
            CoordinateAttitude & robot_coordinate_attitude,
            CoordinateAttitude & goal,
			CoordinateAttitude waypoint_precision_margin);

        void update() override;
		void run();
		CoordinateAttitude get_current_waypoint();
	private:
	    bool has_reached_waypoint();
		CoordinateAttitude waypoint_precision_margin;
    };
}

#endif
