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

    std::string auv_ns = "uvsm";
    qt_controller::QTControllerROS QT_controller(auv_ns);

    ros::spin();

    return 0;  
};
