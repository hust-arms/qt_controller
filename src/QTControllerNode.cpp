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
    std::string ns = "O_29qt";

    if(argc == 1){
        printf("select controller: slide model, default namespace: O_29qt\n");
    }

    
    if(argc == 2 && strcmp(argv[1], "pid") == 0){
        printf("select controller: %s\n", argv[1]);
        ctrl_flag = 1;
    }
    else if(argc == 2 && strcmp(argv[1], "uvms") == 0){
        printf("use namespace with uvms");
    }
    else{
        printf("select controller: slide model, default namespace: O_29qt\n");
    }

    if(argc == 3 && strcmp(argv[2], "uvms") == 0){
        printf("use namespace with uvms");
        ns = "uvms";
    }

    printf("Create controller\n");

    qt_controller::QTControllerROS QT_controller(ns, ctrl_flag);

    ros::spin();

    return 0;  
};
