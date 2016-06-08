
#include <iostream>

#include <LockingSharedObject.hpp>
#include <AStarPathFinder.hpp>
#include <PilotSimulation.hpp>

#include "../source/include/DefaultNavigator.hpp"

int main(int argc,char *argv[]) {
    //pilot initialisation
    CoordinateAttitude pilot_position = {r2d2::Coordinate(1 * r2d2::Length
        ::METER, 1 * r2d2::Length::METER, 0 * r2d2::Length::METER), Attitude()};

    r2d2::Speed s = 0 * r2d2::Length::METER/r2d2::Duration::SECOND;

    r2d2::RobotStatus robot_status(pilot_position, s);

    r2d2::Speed max_speed = 1.00 * r2d2::Length::METER/r2d2::Duration::SECOND;
    Rotation rotation_speed(0.1);

    CoordinateAttitude waypoint = {r2d2::Coordinate(), Attitude()};

    r2d2::PilotSimulation pilot(robot_status, max_speed, rotation_speed,
        waypoint);

    //map initialisation
    std::vector<std::vector<int>> cornerSqueezeMap;
    for (int x = 0; x < 50; x++) {
        std::vector<int> current;
        for (int y = 50; y > 0; y--) {
            if (x == y) {
                if (x >= 4 && x <= 6){
                    current.push_back(0);
                }
                else{
                    current.push_back(1);
                }
            } else {
                current.push_back(0);
            }
        }
        cornerSqueezeMap.push_back(current);
    }
    r2d2::Map map(cornerSqueezeMap);
    map.print_map();

    //pathfinder initialisation
    r2d2::Translation robotBox{.5 * r2d2::Length::METER,
                               .5 * r2d2::Length::METER,
                               0 * r2d2::Length::METER};
    LockingSharedObject<r2d2::Map> sharedMap{map};
    r2d2::AStarPathFinder path_finder(sharedMap, {{}, robotBox});

    //navigator initialisation
    CoordinateAttitude init_goal{r2d2::Coordinate::origin, Attitude()};

    r2d2::DefaultNavigator navigator(pilot, path_finder, pilot_position,
        init_goal, {r2d2::Coordinate(3.0 * r2d2::Length::METER, 3.0 *
        r2d2::Length::METER, 0 * r2d2::Length::METER), Attitude(0 *
        r2d2::Angle::rad, 0 * r2d2::Angle::rad, 0.087 * r2d2::Angle::rad)});

    CoordinateAttitude ca = {r2d2::Coordinate(49 * r2d2::Length::METER,
        49 * r2d2::Length::METER, 0 * r2d2::Length::METER), Attitude()};
    navigator.set_goal(ca);

    //start both threads
    std::thread pilot_simulation_thread(&r2d2::PilotSimulation::run, &pilot);
    std::thread navigator_thread(&r2d2::DefaultNavigator::run, &navigator);

    // log the current_waypoint of the robot and his actual position to the console
	while(true){
        LockingSharedObject<CoordinateAttitude> & temp_robot_status =
            robot_status.get_current_coordinate_attitude();

        CoordinateAttitude & coordinate_attitude =
            SharedObject<CoordinateAttitude>::Accessor(temp_robot_status)
            .access();
        Attitude & my_attitude = coordinate_attitude.attitude;
        r2d2::Coordinate & my_coordinate = coordinate_attitude.coordinate;

		std:cout << "waypoint:    " << navigator.get_current_waypoint()
            .coordinate << " "  << navigator.get_current_waypoint()
            .attitude.angle_z << std::endl;

		std::cout << "RobotStatus: " << coordinate_attitude.coordinate << " "
            << coordinate_attitude.attitude.angle_z << std::endl;


        std::this_thread::sleep_for(std::chrono::microseconds(500000));
	}

    pilot_simulation_thread.join();
    navigator_thread.join();
    return 0;
}
