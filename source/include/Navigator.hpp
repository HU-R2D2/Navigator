#ifndef R2D2_NAVIGATOR_HPP
#define R2D2_NAVIGATOR_HPP

#include <vector>
#include <iostream>

#include <Pilot.hpp>
#include <AStarPathFinder.hpp>
#include <LockingSharedObject.hpp>
#include <stubs.hpp>

namespace r2d2{

	class Navigator{
	public:
		Navigator(r2d2::Pilot & pilot,
			r2d2::AStarPathFinder & path_finder,
			CoordinateAttitude & robot_coordinate_attitude,
            CoordinateAttitude & goal);

		virtual void update() = 0;

		void set_goal(CoordinateAttitude goal);

		LockingSharedObject<CoordinateAttitude> & get_goal();
	protected:
		LockingSharedObject<r2d2::Pilot> pilot;
		r2d2::AStarPathFinder & path_finder;
		LockingSharedObject<CoordinateAttitude> robot_coordinate_attitude;
		LockingSharedObject<CoordinateAttitude> goal;
		LockingSharedObject<std::vector<r2d2::Coordinate>> path;
		CoordinateAttitude current_waypoint{r2d2::Coordinate(), Attitude()};
	private:
		std::vector<r2d2::Coordinate> p;
	};
}

#endif
