/*
 * Filename: CtrlForceParams.h
 * Path: qt_controller
 * Created Date: Monday, September 7th 2020, 9:11:29 pm
 * Author: zhao wang
 * 
 * Copyright (c) 2020 Your Company
 */
#ifndef FORCE_PARAMS_H_
#define FORCE_PARAMS_H_

#include <vector>

using std::vector;

namespace qt_controller{
    struct ForceParams{
        double xdndn_; 
        double ydr_;
        double zdfp_, zdfs_, zdap_, zdas_;
        double kdfp_, kdfs_, kdap_, kdas_, kdr_;
        double mdfp_, mdfs_, mdap_, mdas_;
        double ndr_;

        /**
         * @brief Set parameters
         */
        void setParameters(double rho, double d2, double d04,
                           double xfp, double xfs, double xap, double xas, double xr,
                           double yfp, double yfs, double yap, double yas, double yr,
                           double zfp ,double zfs, double zap, double zas, double zr){
            xdndn_ = 0.1351 * rho * d04;
            ydr_ = 0.00917 * d2; 
            zdfp_ = -0.010 * d2; zdfs_ = -0.010 * d2; zdap_ = -0.010 * d2; zdas_ = -0.010 * d2;

            kdfp_ = yfp * zdfp_; kdfs_ = yfs * zdfs_; kdap_ = yap * zdap_; kdas_ = yas * zdas_; kdr_ = -zr * ydr_;
            mdfp_ = -xfp * zdfp_; mdfs_ = -xfs * zdfs_; mdap_ = -xap * zdap_; mdas_ = -xas * zdas_; 
            ndr_ = xr * ydr_;
        }

        /**
         * @brief Set parameters
         */
        void setParameters(double xdndn, double ydr, double zdfp, double zdfs, double zdap, double zdas, 
                           double kdfp, double kdfs, double kdap, double kdas, double kdr,
                           double mdfp, double mdfs, double mdap, double mdas, double ndr){
            xdndn_ = xdndn; ydr_ = ydr; 
            zdfp_ = zdfp; zdfs_ = zdfs; zdap_ = zdap; zdas_ = zdas;
            kdfp_ = kdfp; kdfs_ = kdfs; kdap_ = kdap; kdas_ = kdas; kdr_ = kdr;
            mdfp_ = mdfp; mdfs_ = mdfs; mdap_ = mdap; mdas_ = mdas;
            ndr_ = ndr;
        }
    }; // CtrlForceParams
}; // ns

#endif
