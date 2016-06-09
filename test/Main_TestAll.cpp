#include <limits.h>
#include "gtest/gtest.h"

#include <AStarPathFinder.hpp>
#include <PilotSimulation.hpp>
//#include <Translation.hpp>

#include "../source/include/DefaultNavigator.hpp"

bool angle_range(r2d2::Angle angle1, r2d2::Angle angle2,
    double offset = 0.0001) {

    return ((angle1 - (offset * r2d2::Angle::rad)) < angle2) &&
        (angle2 < (angle1 + (offset * r2d2::Angle::rad)));
}

bool attitude_equal(Attitude att1, Attitude att2){
    return angle_range(att1.angle_x, att2.angle_x)
        && angle_range(att1.angle_y, att2.angle_y)
        && angle_range(att1.angle_z, att2.angle_z);
}

bool coord_equal(r2d2::Coordinate coord1, r2d2::Coordinate coord2) {
    return coord1.get_x() < (coord2.get_x() + 0.1 * r2d2::Length::METER)
        && coord1.get_y() < (coord2.get_y() + 0.1 * r2d2::Length::METER)
        && coord1.get_z() < (coord2.get_z() + 0.1 * r2d2::Length::METER)
        && coord2.get_x() < (coord1.get_x() + 0.1 * r2d2::Length::METER)
        && coord2.get_y() < (coord1.get_y() + 0.1 * r2d2::Length::METER)
        && coord2.get_z() < (coord1.get_z() + 0.1 * r2d2::Length::METER);
}

TEST(Navigator, set_goal){
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
    r2d2::Map map(50, 50, 0);

    //pathfinder initialisation
    r2d2::Translation robotBox{.5 * r2d2::Length::METER,
                               .5 * r2d2::Length::METER,
                               0 * r2d2::Length::METER};
    LockingSharedObject<r2d2::Map> sharedMap{map};
    r2d2::AStarPathFinder path_finder(sharedMap, {{}, robotBox});

    //navigator initialisation
    CoordinateAttitude init_goal{r2d2::Coordinate::origin, Attitude()};

    r2d2::DefaultNavigator navigator(pilot, path_finder, pilot_position,
        {r2d2::Coordinate(3.0 * r2d2::Length::METER, 3.0 *
        r2d2::Length::METER, 0 * r2d2::Length::METER), Attitude(0 *
        r2d2::Angle::rad, 0 * r2d2::Angle::rad, 0.087 * r2d2::Angle::rad)});

    CoordinateAttitude new_goal = {r2d2::Coordinate(49 * r2d2::Length::METER,
        49 * r2d2::Length::METER, 0 * r2d2::Length::METER), Attitude()};
    navigator.set_goal(new_goal);

    CoordinateAttitude goal_navigator =  SharedObject<CoordinateAttitude>::Accessor(navigator.get_goal()).access();

    ASSERT_TRUE(coord_equal(new_goal.coordinate, goal_navigator.coordinate) && attitude_equal(new_goal.attitude, goal_navigator.attitude));
}

TEST(Navigator, set_goal2){
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

    //map initialisationstd::vector<std::vector<int>> cornerSqueezeMap;
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

    //pathfinder initialisation
    r2d2::Translation robotBox{.5 * r2d2::Length::METER,
                               .5 * r2d2::Length::METER,
                               0 * r2d2::Length::METER};
    LockingSharedObject<r2d2::Map> sharedMap{map};
    r2d2::AStarPathFinder path_finder(sharedMap, {{}, robotBox});

    //navigator initialisation
    CoordinateAttitude init_goal{r2d2::Coordinate::origin, Attitude()};

    r2d2::DefaultNavigator navigator(pilot, path_finder, pilot_position,
        {r2d2::Coordinate(3.0 * r2d2::Length::METER, 3.0 *
        r2d2::Length::METER, 0 * r2d2::Length::METER), Attitude(0 *
        r2d2::Angle::rad, 0 * r2d2::Angle::rad, 0.087 * r2d2::Angle::rad)});

    CoordinateAttitude new_goal = {r2d2::Coordinate(49 * r2d2::Length::METER,
        49 * r2d2::Length::METER, 0 * r2d2::Length::METER), Attitude()};
    ASSERT_TRUE(navigator.set_goal(new_goal));
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
