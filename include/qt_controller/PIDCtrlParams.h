/*
 * Filename: SMCtrlParams.h
 * Path: auv_controller
 * Created Date: Tuesday, September 8th 2020, 10:32:15 pm
 * Author: zhao wang
 * 
 * Copyright (c) 2020 Your Company
 */

#ifndef PID_CTRL_PARAMS_H_
#define PID_CTRL_PARAMS_H_

#include <stdio.h>

namespace qt_controller{
    /**
     * @brief PID control parameters
     */
    struct PIDCtrlParams{
        double p_;
        double i_;
        double d_;

        /**
         * @brief Default constructor
         */ 
        PIDCtrlParams():p_(0.0), i_(0.0), d_(0.0){}
        
        /**
         * @brief Print PID control parameters
         * @param p Scaling factor 
         * @param i Inertial factor
         * @param d Differential factor
         */ 
        PIDCtrlParams(double p, double i, double d):p_(p), i_(i), d_(d){}

        /**
         * @brief Print slide control parameters
         */ 
        void printPIDCtrlParameters()const{
            printf("PIDCtrlParam:{p:%f i:%f d:%f}\n", p_, i_, d_);
        }
    }; // PIDCtrlParams
}; // ns

#endif
