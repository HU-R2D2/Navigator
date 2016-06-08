#include "../include/Navigator.hpp"

r2d2::Navigator::Navigator(r2d2::Pilot & pilot,
    r2d2::AStarPathFinder & path_finder,
    CoordinateAttitude & robot_coordinate_attitude,
    CoordinateAttitude & goal):
        pilot(pilot),
        path_finder(path_finder),
        robot_coordinate_attitude(robot_coordinate_attitude),
        goal(goal),
        path(p){
}

void r2d2::Navigator::set_goal(CoordinateAttitude new_goal){
    LockingSharedObject<CoordinateAttitude>::Accessor goal_accessor(goal);
    CoordinateAttitude & goal_data  = goal_accessor.access();
    goal_data = new_goal;

    LockingSharedObject<CoordinateAttitude>::Accessor
        robot_orientation_accessor(robot_coordinate_attitude);
    CoordinateAttitude & robot_orientation_data = robot_orientation_accessor
        .access();

    std::vector<Coordinate> & path_vector =
        LockingSharedObject<std::vector<Coordinate>>::Accessor(path).access();

    if(path_finder.get_path_to_coordinate(robot_orientation_data.coordinate,
        new_goal.coordinate, path_vector)){

        for (int i = 0; i < path_vector.size(); ++i){
            std::cout << path_vector[i] << std::endl;
        }
    }
}

LockingSharedObject<CoordinateAttitude> & r2d2::Navigator::get_goal(){
    return goal;
}
