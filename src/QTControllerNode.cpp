/*
 * Filename: QTControllerNode.cpp
 * Path: QT_docking\src
 * Created Date: Tuesday, September 15th 2020, 3:42:02 pm
 * Author: zhao wang
 * 
 * Copyright (c) 2020 Your Company
 */

#include "qt_controller/QTControllerROS.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "qt_controller_node");

    std::string ns = "O_29qt";
    
    if(argc == 2){
        if(strcmp(argv[1], "uvms") == 0){
            ns = "uvms";
        }
    }

    qt_controller::QTControllerROS QT_controller(ns);

    ros::spin();

    return 0;  
};
