/*
 * Filename: QTControllerNode.cpp
 * Path: QT_docking\src
 * Created Date: Tuesday, September 15th 2020, 3:42:02 pm
 * Author: zhao wang
 * 
 * Copyright (c) 2020 Your Company
 */
#include "stdio.h"
#include "qt_controller/QTControllerROS.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "qt_controller_node");

    int ctrl_flag = 0; // 0 - slide model controller 1 - PID controller

    if(argc == 2 && strcmp(argv[1], "pid") == 0){
        printf("select controller: %s\n", argv[1]);
        ctrl_flag = 1;
    }
    else{
        printf("select controller: slide model\n");
    }

    std::string auv_ns = "uvsm";
    qt_controller::QTControllerROS QT_controller(auv_ns, ctrl_flag);

    ros::spin();

    return 0;  
};
