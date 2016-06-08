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

                SharedObject<Pilot>::Accessor(pilot).access()
                    .go_to_position(current_waypoint);
            }
        }
    }

    void DefaultNavigator::run(){
        while(true){
            std::this_thread::sleep_for(std::chrono::microseconds(500000));
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
