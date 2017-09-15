//
// Created by sun on 17-9-15.
//

#include <ros/ros.h>
#include <ecnu_pick_place/ECNUPickPlace.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_pick_place");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(4);
    spinner.start();

    string planner = "LBKPIECEkConfigDefault";
    if(argc > 1) {
        planner = argv[1];
    }

    ecnu_pick_place::ECNUPickPlace t(nh, planner);
    t.start();

    return EXIT_SUCCESS;
}