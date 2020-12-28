/*
 * Filename: Dynamic.h
 * Path: qt_controller
 * Created Date: Monday, September 7th 2020, 8:51:07 pm
 * Author: zhao wang
 * 
 * Copyright (c) 2020 Your Company
 */
#ifndef DYNAMIC_H_
#define DYNAMIC_H_

namespace qt_controller{
    /**
     * @brief qt dynamic parameters
     */
    struct QTDynamic{
        // Associated mass
        double x_dotu_, x_dotv_, x_dotw_, x_dotp_, x_dotq_, x_dotr_; 
        double y_dotu_, y_dotv_, y_dotw_, y_dotp_, y_dotq_, y_dotr_;
        double z_dotu_, z_dotv_, z_dotw_, z_dotp_, z_dotq_, z_dotr_;
        double k_dotu_, k_dotv_, k_dotw_, k_dotp_, k_dotq_, k_dotr_;
        double m_dotu_, m_dotv_, m_dotw_, m_dotp_, m_dotq_, m_dotr_;
        double n_dotu_, n_dotv_, n_dotw_, n_dotp_, n_dotq_, n_dotr_;
        
        // Damping
        double x_uu_, x_uv_, x_uw_, x_up_, x_uq_, x_ur_; 
        double y_uu_, y_uv_, y_uw_, y_up_, y_uq_, y_ur_; 
        double z_uu_, z_uv_, z_uw_, z_up_, z_uq_, z_ur_; 
        double k_uu_, k_uv_, k_uw_, k_up_, k_uq_, k_ur_; 
        double m_uu_, m_uv_, m_uw_, m_up_, m_uq_, m_ur_; 
        double n_uu_, n_uv_, n_uw_, n_up_, n_uq_, n_ur_; 

        /**
         * @brief Constructor
         */
        QTDynamic(){
            x_dotu_ = 0.0; x_dotv_ = 0.0; x_dotw_ = 0.0; x_dotp_ = 0.0; x_dotq_ = 0.0; x_dotr_ = 0.0;
            y_dotu_ = 0.0; y_dotv_ = 0.0; y_dotw_ = 0.0; y_dotp_ = 0.0; y_dotq_ = 0.0; y_dotr_ = 0.0;
            z_dotu_ = 0.0; z_dotv_ = 0.0; z_dotw_ = 0.0; z_dotp_ = 0.0; z_dotq_ = 0.0; z_dotr_ = 0.0;
            k_dotu_ = 0.0; k_dotv_ = 0.0; k_dotw_ = 0.0; k_dotp_ = 0.0; k_dotq_ = 0.0; k_dotr_ = 0.0;
            m_dotu_ = 0.0; m_dotv_ = 0.0; m_dotw_ = 0.0; m_dotp_ = 0.0; m_dotq_ = 0.0; m_dotr_ = 0.0;
            n_dotu_ = 0.0; n_dotv_ = 0.0; n_dotw_ = 0.0; n_dotp_ = 0.0; n_dotq_ = 0.0; n_dotr_ = 0.0;
        
            // Damping
            x_uu_ = 0.0; x_uv_ = 0.0; x_uw_ = 0.0; x_up_ = 0.0; x_uq_ = 0.0; x_ur_ = 0.0;
            y_uu_ = 0.0; y_uv_ = 0.0; y_uw_ = 0.0; y_up_ = 0.0; y_uq_ = 0.0; y_ur_ = 0.0;
            z_uu_ = 0.0; z_uv_ = 0.0; z_uw_ = 0.0; z_up_ = 0.0; z_uq_ = 0.0; z_ur_ = 0.0;
            k_uu_ = 0.0; k_uv_ = 0.0; k_uw_ = 0.0; k_up_ = 0.0; k_uq_ = 0.0; k_ur_ = 0.0;
            m_uu_ = 0.0; m_uv_ = 0.0; m_uw_ = 0.0; m_up_ = 0.0; m_uq_ = 0.0; m_ur_ = 0.0;
            n_uu_ = 0.0; n_uv_ = 0.0; n_uw_ = 0.0; n_up_ = 0.0; n_uq_ = 0.0; n_ur_ = 0.0;
        }

        /**
         * @brief QT dynamic parameters setting
         */
        void setParameters(double d0, double d1, double d2, double d3, double d4, double d5)
        {
            double s = 0.1;
            // Associated mass
            x_dotu_ = -0.000949 * d3 * s; x_dotv_ = 0.0; x_dotw_ = 0.0; x_dotp_ = 0.0; x_dotq_ = 0.0; x_dotr_ = 0.0;
            y_dotu_ = 0.0; y_dotv_ = -0.0408 * d3 * s; y_dotw_ = 0.0; y_dotp_ = 0.000547 * d4 * s; y_dotq_ = 0.0; y_dotr_ = 0.00139 * d4 * s;
            z_dotu_ = 0.0; z_dotv_ = 0.0; z_dotw_ = -0.0226 * d3 * s; z_dotp_ = 0.0; z_dotq_ = -0.0000510 * d4 * s; z_dotr_ = 0.0;
            k_dotu_ = 0.0; k_dotv_ = 0.000547 * d4 * s; k_dotw_ = 0.0; k_dotp_ = -0.000105 * d5 * s; k_dotq_ = 0.0; k_dotr_ = -0.00000370 * d5 * s;
            m_dotu_ = 0.0; m_dotv_ = 0.0; m_dotw_ = -0.0000510 * d4 * s; m_dotp_ = 0.0; m_dotq_ = -0.00175 * d5 * s; m_dotr_ = 0.0;
            n_dotu_ = 0.0; n_dotv_ = 0.00139 * d4 * s; n_dotw_ = 0.0; n_dotp_ = -0.00000370 * d5 * s; n_dotq_ = 0.0; n_dotr_ = -0.00183 * d5 * s;
        
            // Damping
            x_uu_ = -0.00315 * d2; x_uv_ = 0.0; x_uw_ = 0.0; x_up_ = 0.0; x_uq_ = 0.0; x_ur_ = 0.0;
            y_uu_ = 0.0; y_uv_ = -0.099 * d2; y_uw_ = 0.0; y_up_ = 0.00677 * d3; y_uq_ = 0.0; y_ur_ = 0.0250 * d3;
            z_uu_ = 0.0; z_uv_ = 0.0; z_uw_ = -0.067 * d2; z_up_ = 0.0; z_uq_ = -0.00658 * d3; z_ur_ = 0.0;
            k_uu_ = 0.0; k_uv_ = 0.00406 * d3; k_uw_ = 0.0; k_up_ = -0.00145 * d4; k_uq_ = 0.0; k_ur_ = -0.000578 * d4;
            m_uu_ = 0.0; m_uv_ = 0.0; m_uw_ = 0.00120 * d3; m_up_ = 0.0; m_uq_ = -0.00687 * d4; m_ur_ = 0.0;
            n_uu_ = 0.0; n_uv_ = -0.005 * d3; n_uw_ = 0.0; n_up_ = 0.000086 * d4; n_uq_ = 0.0; n_ur_ = -0.00747 * d4;
        }
    }; // QTDynamic
}; // ns

#endif
