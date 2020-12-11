/*
 * Filename: QTController.h
 * Path: QT_controller
 * Created Date: Monday, September 7th 2020, 9:40:39 pm
 * Author: zhao wang
 * 
 * Copyright (c) 2020 Your Company
 */
#ifndef QT_CONTROLLER_H_
#define QT_CONTROLLER_H_

#include <vector>
#include <string>
#include <sstream>
#include "BodyParams.h"
#include "ForceParams.h"
#include "CtrlParams.h"
#include "Dynamic.h"
#include "Common.h"

namespace qt_controller{

    // Pre determination
    class QTController;

    const double PI = 3.141592654;

    /**
     * @brief QT controller input parameters
     */ 
    class QTControllerInput{
        friend QTController;
    public:
        /**
         * @brief Constructor
         */ 
        QTControllerInput(double depthd, double pitchd, double yawd, double xd, double yd) : 
            depth_d_(depthd), pitch_d_(pitchd), yaw_d_(yawd), x_d_(xd), y_d_(yd){};

    private:
        double depth_d_; // desired depth
        double pitch_d_; // desired pitch 
        double yaw_d_; // desired yaw
        double x_d_; // waypoint on the desired line
        double y_d_; // waypoint on the desired line
    }; //  QTControllerInput

    /**
     * @brief QT controller output parameters
     */
    struct QTControllerOutput{
        double rouder_;
        double fwd_fin_;
        double aft_fin_;
    }; // QTControllerOutput

    struct QTRouderPostion{
        double xfp_, xfs_, xap_, xas_, xr_;
        double yfp_, yfs_, yap_, yas_, yr_;
        double zfp_, zfs_, zap_, zas_, zr_;

        /**
         * @brief Set rouder position
         */
        void setParameters(double xfp, double xfs, double xap, double xas, double xr,
                           double yfp, double yfs, double yap, double yas, double yr,
                           double zfp ,double zfs, double zap, double zas, double zr)
        {
            xfp_ = xfp; xfs_ = xfs; xap_ = xap; xas_ = xas; xr_ = xr; 
            yfp_ = yfp; yfs_ = yfs; yap_ = yap; yas_ = yas; yr_ = yr; 
            zfp_ = zfp; zfs_ = zfs; zap_ = zap; zas_ = zas; zr_ = zr; 
        }
    }; // QTRouderPostion

    /**
     * @brief Arms QT controller
     */ 
    class QTController{
    public:
        /**
         * @brief Default class initialization
         */
        QTController();

        /**
         * @brief Deconstructor
         */
        ~QTController(){};

        /**
         * @brief Set QT body parameters whicih includes mass, length, gravity, buoyancy et al.
         */
        void setQTBodyParams(double m, double l, double w, double b, 
            double xb, double yb, double zb, double xg, double yg, double zg, 
            double ixx, double iyy, double izz)
        {
            body_.setParameters(m, l, w, b, xb, yb, zb, xg, yg, zg, ixx, iyy, izz);
        };

        /**
         * @brief Set QT body parameters whicih includes mass, length, gravity, buoyancy et al.
         */
        bool setQTBodyParams(const std::vector<double>& body);

        /**
         * @brief Set QT hydrodyanamic parameters 
         */ 
        void setQTDynamic(double d0, double d1, double d2, double d3, double d4, double d5)
        {
            dynamic_.setParameters(d0, d1, d2, d3, d4, d5);
        };

        /**
         * @brief Set QT hydrodyanamic parameters 
         */ 
        bool setQTDynamic(const std::vector<double>& d);

        /**
         * @brief Set control parameters
         */ 
        void setCtrlParams(double cz, double kz, double alphaz,
            double ctheta, double ktheta, double alphatheta,
            double cpsi, double kpsi, double alphapsi, double bt);

        /**
         * @brief Set control parameters
         */ 
        bool setCtrlParams(const std::vector<double>& ctrl);

        /**
         * @brief Set control parameters
         */
        void setCtrlParams(double c, double k, double alpha, unsigned int flag);

        /**
         * @brief Set boundary thick
         */
        void setBoundaryThick(double bt){
            ctrl_.bondary_thick_ = bt;
        }

        /**
         * @brief Set force parameters
         */ 
        void setForceParams(double rho, double d2, double d04, QTRouderPostion& rouder){
            force_.setParameters(rho, d2, d04,
                                 rouder.xfp_, rouder.xfs_, rouder.xap_, rouder.xas_, rouder.xr_,
                                 rouder.yfp_, rouder.yfs_, rouder.yap_, rouder.yas_, rouder.yr_,
                                 rouder.zfp_, rouder.zfs_, rouder.zap_, rouder.zas_, rouder.zr_);
        };

        /**
         * @brief Set force parameters
         */ 
        void setForceParams(double xdndn, double ydr, double zdfp, double zdfs, double zdap, double zdas, 
                             double kdfp, double kdfs, double kdap, double kdas, double kdr,
                             double mdfp, double mdfs, double mdap, double mdas, double ndr)
        {
            force_.setParameters(xdndn,  ydr,  zdfp,  zdfs,  zdap,  zdas, 
                              kdfp,  kdfs,  kdap,  kdas,  kdr,
                              mdfp,  mdfs,  mdap,  mdas,  ndr);
        };

        /**
         * @brief Set rouder position
         */ 
        void setRouderPosition(double xfp, double xfs, double xap, double xas, double xr,
                               double yfp, double yfs, double yap, double yas, double yr,
                               double zfp ,double zfs, double zap, double zas, double zr)
        {
            rouder_.setParameters(xfp, xfs, xap, xas, xr,
                                  yfp, yfs, yap, yas, yr,
                                  zfp, zfs, zap, zas, zr);
        }

        /**
         * @brief Control solution
         */
        void controllerRun(const QTKineticSensor& sensor, const QTControllerInput& input, QTControllerOutput& output, const double dt);

    public:
        /**
         * @brief Serialize QT body parameters
         */ 
        void serializeQTBodyParams(std::stringstream& str);
        
        /**
         * @brief Serialize QT dynamic parameters
         */ 
        void serializeQTDynamicParams(std::stringstream& str);

        /**
         * @brief Serialize QT force parameters
         */ 
        void serializeQTForceParams(std::stringstream& str);

        /**
         * @brief Serialize QT control parameters
         */ 
        void serializeQTControlParams(std::stringstream& str);

    private:
        /**
         * @brief Initialize with default parameters
         */ 
        void defaultInit();

    private:
        /**
         * @brief QT depth surface status
         */ 
        struct QTDepthSFStatus{
            double a_zw_, a_zq_, a_zs_, a_zb_, f_z_;
            double a_tw_, a_tq_, a_ts_, a_tb_, f_t_;
            double b_z_, b_zb_, b_zs_, b_t_, b_tb_, b_ts_;
            double g_z_, g_zb_, g_zs_, g_t_, g_tb_, g_ts_;
            double dot_z_, dot_theta_;

            void init(){
                a_zw_=0.0; a_zq_=0.0; a_zs_=0.0; a_zb_=0.0; f_z_=0.0;
                a_tw_=0.0; a_tq_=0.0; a_ts_=0.0; a_tb_=0.0; f_t_=0.0;
                b_z_=0.0; b_zb_=0.0; b_zs_=0.0; b_t_=0.0; b_tb_=0.0; b_ts_=0.0;
                g_z_=0.0; g_zb_=0.0; g_zs_=0.0; g_t_=0.0; g_tb_=0.0; g_ts_=0.0;
                dot_z_=0.0; dot_theta_=0.0;
            }
        }; // QTDepthSFStatus

        /**
         * @brief QT horizontal surface status
         */ 
        struct QTHorizonSFStatus{
            double a_yv_, a_yr_, a_ydr_, f_y_;
            double a_pv_, a_pr_, a_pdr_, f_p_;
            double b_y_, b_ydr_, b_p_, b_pdr_;
            double dot_psi_;

            void init(){
                a_yv_=0.0; a_yr_=0.0; a_ydr_=0.0; f_y_=0.0;
                a_pv_=0.0; a_pr_=0.0; a_pdr_=0.0; f_p_=0.0;
                b_y_=0.0; b_ydr_=0.0; b_p_=0.0; b_pdr_=0.0;
                dot_psi_=0.0;
            }
        }; // QTHorizonSFStatus

        /**
         * @brief Slide model parameters
         */
        struct SlideModelParams{
            SMCtrlParams z_; // depth
            SMCtrlParams theta_; // picth
            SMCtrlParams psi_; // yaw
        }; // SlideModelParams

        /**
         * @brief Filter function
         */
        int sign(double input){
            if(input > 0){
                return 1;
            }
            else if (input < 0){
                return -1;
            }
            else
            {
                return 0;
            }
        };

        double sat(double input, double thick){
            return fabs(input) >= thick ? sign(input) : (input / thick);
        };

    private:
        QTBodyParams body_; 
        QTDynamic dynamic_;
        ForceParams force_;
        CtrlParams ctrl_;
        QTRouderPostion rouder_;

        QTKinetic kinetic_;
        CtrlMissionParams mission_;
        SlideModelParams slide_model_;

        QTDepthSFStatus depth_sf_;
        QTHorizonSFStatus horizon_sf_;

        // Commands
        double deltab_, deltas_, deltar_;

        double rho_;
        double d00_;
        const double a_ = PI * 0.5 * 0.5;

        // Const value
        const unsigned int body_num_ = 13;
        const unsigned int dfactors_num_ = 6;
        const unsigned int ctrl_num_ = 10;
        const unsigned int force_num_ = 6;
    }; // QTController
}; // ns

#endif
