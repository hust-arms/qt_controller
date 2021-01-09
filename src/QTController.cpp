/*
 * Filename: QTController.cpp
 * Path: QT_controller
 * Created Date: Tuesday, September 8th 2020, 2:53:16 pm
 * Author: zhao wang
 * 
 * Copyright (c) 2020 Your Company
 */

#include <math.h>
#include "qt_controller/QTController.h"

namespace qt_controller{
    QTController::QTController(int controller_type){
        controller_type_ = controller_type;
        defaultInit();
    }

    /**
     * @brief Set QT body parameters whicih includes mass, length, gravity, buoyancy et al.
     */
    bool QTController::setQTBodyParams(const std::vector<double>& body){
        if(body.size() == body_num_){
            setQTBodyParams(body[0], body[1], body[2], body[3], body[4], body[5], body[6],
                body[7], body[8], body[9], body[10], body[11], body[12]);
            return true;
        }
        else{
            return false;
        }
    };

    /**
     * @brief Set QT hydrodyanamic parameters 
     */ 
    bool QTController::setQTDynamic(const std::vector<double>& dynamic){
        if(dynamic.size() == dfactors_num_){
            setQTDynamic(dynamic[0], dynamic[1], dynamic[2], dynamic[3], dynamic[4], dynamic[5]);
            return true;
        }
        else{
            return false;
        }
    };

    /**
     * @brief Set control parameters
     */ 
    bool QTController::setCtrlParams(const std::vector<double>& ctrl){
        if(ctrl.size() == ctrl_num_){
            setCtrlParams(ctrl[0], ctrl[1], ctrl[2], ctrl[3], ctrl[4], ctrl[5], ctrl[6], ctrl[7], ctrl[8], ctrl[9]);
            return true;
        }
        else{
            return false;
        }
    }

    /**
     * @brief Initialize with default paramters
     */ 
    void QTController::defaultInit(){
        // double mass = 4390.0;
        double mass = 815650.0;
        // double bouy = 4690 * 9.81;
        // double bouy = 4390 * 9.81;
        double bouy = 815650 * 9.81;
        double weight = mass * 9.81;
        // double len = 8.534;
        double len = 40.97;
        double bx = 0.0; double by = 0.0; double bz = -0.137;
        // double bx = 0.0; double by = 0.0; double bz = 0.0;
        // double ixx = 1315.0; double iyy = 5900.0; double izz = 5057.0;
        double ixx = 1436900.0; double iyy = 25510000.0; double izz = 24921000.0;
        setQTBodyParams(mass, len, weight, bouy, bx, by, bz, 0.0, 0.0, 0.0, ixx, iyy, izz);
        rho_ = 1025; 
        d00_ = 0.7159;
        double d04 = pow(d00_, 4);
        // double len = body_.getLength();
        double d0 = 0.5 * rho_; double d1 = d0 * len; double d2 = d1 * len; double d3 = d2 * len;
        double d4 = d3 * len; double d5 = d4 * len;
        setQTDynamic(d0, d1, d2, d3, d4, d5);
        setRouderPosition(1.865, 1.865, -3.369, -3.369, -3.369,
                          -0.699, 0.699, -0.621, 0.621, 0.0,
                          0.013, 0.013, -0.0844, -0.0844, 0.0);
        setForceParams(rho_, d2, d04, rouder_);
        // setCtrlParams(0.05, 0.05, 0.5, 0.15, 0.15, 0.4, 0.0, 0.0, 0.0, 0.01);
        // setCtrlParams(1.0, 1.0, 1.0, 1.5, 1.5, 0.4, 0.0, 0.0, 0.0, 0.1);
        setCtrlParams(0.5, 0.5, 0.5, 0.15, 0.15, 0.4, 0.0, 0.0, 0.0, 0.01);
        setPIDCtrlParams(10.0, 1.8, 0.0);

        depth_sf_.init();
        horizon_sf_.init();

        factor_ = 4.0;

        /* Initialize mission control variable */
        mission_.z_.ref_ = 0.0;
        mission_.z_.ref_dot_ = 0.0;
        mission_.z_.ref_dot2_ = 0.0;
        mission_.z_.pre_ref_ = 0.0;
        mission_.z_.pre_ref_dot_ = 0.0;
        mission_.z_.det_ = 0.0;
        mission_.z_.det_pre_ = 0.0;
        mission_.z_.det_pre2_ = 0.0;

        mission_.theta_.ref_ = 0.0;
        mission_.theta_.ref_dot_ = 0.0;
        mission_.theta_.ref_dot2_ = 0.0;
        mission_.theta_.pre_ref_ = 0.0;
        mission_.theta_.pre_ref_dot_ = 0.0;
        mission_.theta_.det_ = 0.0;
        mission_.theta_.det_pre_ = 0.0;
        mission_.theta_.det_pre2_ = 0.0;

        mission_.psi_.ref_ = 0.0;
        mission_.psi_.ref_dot_ = 0.0;
        mission_.psi_.ref_dot2_ = 0.0;
        mission_.psi_.pre_ref_ = 0.0;
        mission_.psi_.pre_ref_dot_ = 0.0;
        mission_.psi_.det_ = 0.0;
        mission_.psi_.det_pre_ = 0.0;
        mission_.psi_.det_pre2_ = 0.0;
        
        deltab_ = 0.0; deltas_ = 0.0; deltar_ = 0.0;
    }

    /**
     * @brief Set control parameters
     */
    void QTController::setCtrlParams(double cz, double kz, double alphaz,
            double ctheta, double ktheta, double alphatheta,
            double cpsi, double kpsi, double alphapsi, double bt)
    {  
        ctrl_.z_.setParameters(cz, kz, alphaz); // depth 
        ctrl_.theta_.setParameters(ctheta, ktheta, alphatheta); // pitch
        ctrl_.psi_.setParameters(cpsi, kpsi, alphapsi); // yaw
        ctrl_.bondary_thick_ = bt;
    };

    /**
     * @brief Set control parameters
     */
    void QTController::setCtrlParams(double c, double k, double alpha, unsigned int flag)
    {
        switch (flag)
        {
        case 0:
            ctrl_.z_.setParameters(c, k, alpha);
            break;
        case 1:
            ctrl_.theta_.setParameters(c, k, alpha);
            break;
        case 2:
            ctrl_.psi_.setParameters(c, k, alpha);
            break;
        default:
            break;
        }
    }

    void QTController::setPIDCtrlParams(double p, double i, double d){
        pid_.p_ = p;
        pid_.i_ = i;
        pid_.d_ = d;
    }

    /**
     * @brief Serialize QT body parameters
     */ 
    void QTController::serializeQTBodyParams(std::stringstream& str){
        str << "m: " << body_.m_ << " l: " << body_.l_ << " w: " << body_.w_ << " b: " << body_.b_;
        str << " xb: " << body_.x_b_ << " yb: " << body_.y_b_ << " zb: " << body_.z_b_;
        str << " xg: " << body_.x_g_ << " yg: " << body_.y_g_ << " zg: " << body_.z_g_;
        str << " ixx: " << body_.i_xx_ << " iyy: " << body_.i_yy_ << " izz: " << body_.i_zz_;
    }

    /**
     * @brief Serialize QT dynamic parameters
     */ 
    void QTController::serializeQTDynamicParams(std::stringstream& str){
        str << "xdotu: " << dynamic_.x_dotu_ << " ydotv: " << dynamic_.y_dotv_ <<" ydotr: " << dynamic_.y_dotr_ << " zdotw: " << dynamic_.z_dotw_ << " zdotq: " << dynamic_.z_dotq_;
        str << " kdotp: " << dynamic_.k_dotp_ << " mdotw: " << dynamic_.m_dotw_ << " mdotq: " << dynamic_.m_dotq_ << " ndotv: " << dynamic_.n_dotv_ << " ndotr: " << dynamic_.n_dotr_;
    } 

    /**
     * @brief Serialize QT force parameters
     */ 
    void QTController::serializeQTForceParams(std::stringstream& str){
        str << "xdndn: " << force_.xdndn_ << " ydr: " << force_.ydr_ << " zdfp: " << force_.zdfp_ 
            << " zdfs: " << force_.zdfs_ << " zdap: " << force_.zdap_ << " zdas: " << force_.zdas_
            << " kdfp: " << force_.kdfp_ << " kdfs: " << force_.kdfs_ << " kdap: " << force_.kdap_
            << " kdas: " << force_.kdas_ << " kdr: " << force_.kdr_ << " mdfp: " << force_.mdfp_
            << " mdfs: " << force_.mdfs_ << " mdap: " << force_.mdap_ << " mdas: " << force_.mdas_ << " ndr: " << force_.ndr_;
    }

    /**
     * @brief Serialize QT control parameters
     */ 
    void QTController::serializeQTControlParams(std::stringstream& str){
        str << "cz: " << ctrl_.z_.c_ << " kz: " << ctrl_.z_.k_ << " alphaz: " << ctrl_.z_.alpha_;
        str << " ctheta: " << ctrl_.theta_.c_ << " ktheta: " << ctrl_.theta_.k_ << " alphatheta: " << ctrl_.theta_.alpha_;
        str << " cpsi: " << ctrl_.psi_.c_ << " kpsi: " << ctrl_.psi_.k_ << " alphapsi: " << ctrl_.psi_.alpha_;
        str << " boundary thick: " << ctrl_.bondary_thick_;
    }


    void QTController::controllerRun(const QTKineticSensor& sensor, const QTControllerInput& input, QTControllerOutput& output, const double dt){
        if(controller_type_ == 0){
            smcControllerRun(sensor, input, output, dt);
            return;
        }
        pidControllerRun(sensor, input, output, dt);
    }

    void QTController::smcControllerRun(const QTKineticSensor& sensor, const QTControllerInput& input, QTControllerOutput& output, const double dt)
    {
        // Update kinetic parameters
        kinetic_.setPosition(sensor.x_, sensor.y_, sensor.z_, sensor.roll_, sensor.pitch_, sensor.yaw_);
        // kinetic_.setVelocity(sensor.x_dot_, sensor.y_dot_, sensor.z_dot_, sensor.roll_dot_, sensor.pitch_dot_, sensor.yaw_dot_);
        kinetic_.setVelocity(sensor.x_dot_, 0.0, sensor.z_dot_, sensor.roll_dot_, sensor.pitch_dot_, sensor.yaw_dot_);

        // Draw and compute control mission parameters
        mission_.z_.ref_ = input.depth_d_;
        mission_.z_.ref_dot_ = 0;
        mission_.z_.ref_dot2_ = 0;
        mission_.z_.Update(); // update previous depth and depth differential
        // printf("z:{ref:%f ref_dot:%f ref_dot2:%f pre_ref:%f pre_ref_dot:%f}\n", mission_.z_.ref_, mission_.z_.ref_dot_, mission_.z_.ref_dot2_, 
        //        mission_.z_.pre_ref_, mission_.z_.pre_ref_dot_);

        // mission_.theta_.ref_ = input.pitch_d_ + 0.1 * atan((kinetic_.z_ - mission_.z_.ref_) / (4 * body_.l_));
        // mission_.theta_.ref_ = -0.33333 * atan(-(kinetic_.z_ - mission_.z_.ref_) / (1.5 * body_.l_));
        mission_.theta_.ref_ = -0.33333 * atan((kinetic_.z_ - mission_.z_.ref_) / (1.5 * body_.l_));
        mission_.theta_.ref_dot_ = (mission_.theta_.ref_ - mission_.theta_.pre_ref_) / dt;
        mission_.theta_.ref_dot2_ = (mission_.theta_.ref_dot_ - mission_.theta_.pre_ref_dot_) / dt;
        mission_.theta_.Update();
        // printf("theta:{ref:%f ref_dot:%f ref_dot2:%f pre_ref:%f pre_ref_dot:%f}\n", mission_.theta_.ref_, mission_.theta_.ref_dot_, mission_.theta_.ref_dot2_, 
        //        mission_.theta_.pre_ref_, mission_.theta_.pre_ref_dot_);

        mission_.lateral_dist_ = (kinetic_.x_ - input.x_d_) * sin(input.yaw_d_) - (kinetic_.y_ - input.y_d_) * cos(input.yaw_d_);
        printf("LateralDist:%f\n", mission_.lateral_dist_);

        mission_.psi_.ref_ = input.yaw_d_ + atan(mission_.lateral_dist_ / 10);
        mission_.psi_.ref_dot_ = (mission_.psi_.ref_ - mission_.psi_.pre_ref_) / dt;
        mission_.psi_.ref_dot2_ = (mission_.psi_.ref_dot_ - mission_.psi_.pre_ref_dot_) / dt;
        mission_.psi_.Update();
        // printf("psi:{ref:%f ref_dot:%f ref_dot2:%f pre_ref:%f pre_ref_dot:%f}\n", mission_.psi_.ref_, mission_.psi_.ref_dot_, mission_.psi_.ref_dot2_, 
        //        mission_.psi_.pre_ref_, mission_.psi_.pre_ref_dot_);

        // Lateral variable substitution
        double zds = force_.zdap_ + force_.zdas_;
        double zdb = force_.zdfp_ + force_.zdfs_;
        double mds = force_.mdap_ + force_.mdas_;
        double mdb = force_.mdfp_ + force_.mdfs_;
        depth_sf_.a_zw_ = body_.m_ - dynamic_.z_dotw_;
        depth_sf_.a_zq_ = -(body_.m_ * body_.x_g_ + dynamic_.z_dotq_);
        depth_sf_.a_zs_ = zds * kinetic_.u_ * kinetic_.u_;
        depth_sf_.a_zb_ = zdb * kinetic_.u_ * kinetic_.u_;
        depth_sf_.f_z_ = body_.m_ * body_.z_g_ * kinetic_.q_ * kinetic_.q_ + body_.m_ * kinetic_.u_ * kinetic_.q_ + dynamic_.z_uw_ * kinetic_.u_ * kinetic_.w_ +
            dynamic_.z_uq_ * kinetic_.u_ * kinetic_.q_ - dynamic_.x_dotu_ * kinetic_.u_ * kinetic_.q_ + (body_.w_ - body_.b_) * cos(kinetic_.theta_);

        // printf("Temp{a_zw:%f a_zq:%f a_zs:%f a_zb:%f f_z:%f}\n", depth_sf_.a_zw_, depth_sf_.a_zq_, depth_sf_.a_zs_, depth_sf_.a_zb_, depth_sf_.f_z_);
        
        // Pitch variable substitution
        depth_sf_.a_tw_ = -(body_.m_ * body_.x_g_ + dynamic_.m_dotw_);
        depth_sf_.a_tq_ = body_.i_yy_ - dynamic_.m_dotq_;
        depth_sf_.a_ts_ = mds * kinetic_.u_ * kinetic_.u_;
        depth_sf_.a_tb_ = mdb * kinetic_.u_ * kinetic_.u_;
        depth_sf_.f_t_ = dynamic_.x_dotu_ * kinetic_.u_ * kinetic_.w_ + dynamic_.m_uw_ * kinetic_.u_ * kinetic_.w_ + dynamic_.m_uq_ * kinetic_.u_ * kinetic_.q_ -
            dynamic_.z_dotw_ * kinetic_.u_ * kinetic_.w_ - dynamic_.z_dotq_ * kinetic_.u_ * kinetic_.q_ - body_.m_ * body_.z_g_ * kinetic_.w_ * kinetic_.q_ - body_.m_ * body_.x_g_ * kinetic_.u_ * kinetic_.q_ -
            (body_.z_g_ * body_.w_ - body_.z_b_ * body_.b_) * sin(kinetic_.theta_) - (body_.x_g_ * body_.w_ - body_.x_b_ * body_.b_) * cos(kinetic_.theta_);

        // printf("Temp{a_tw:%f a_tq:%f a_ts:%f a_tb:%f f_t:%f}\n", depth_sf_.a_tw_, depth_sf_.a_tq_, depth_sf_.a_ts_, depth_sf_.a_tb_, depth_sf_.f_t_);
        
        // dot_w & dot_q
        double common1 = depth_sf_.a_zw_ * depth_sf_.a_tq_ - depth_sf_.a_zq_ * depth_sf_.a_tw_;
        depth_sf_.b_z_ = (depth_sf_.a_tq_ * depth_sf_.f_z_ - depth_sf_.a_zq_ * depth_sf_.f_t_) / common1;
        depth_sf_.b_zb_ = (depth_sf_.a_tq_ * depth_sf_.a_zb_ - depth_sf_.a_zq_ * depth_sf_.a_tb_) / common1;
        depth_sf_.b_zs_ = (depth_sf_.a_tq_ * depth_sf_.a_zs_ - depth_sf_.a_zq_ * depth_sf_.a_ts_) / common1;
        depth_sf_.b_t_ = (depth_sf_.a_zw_ * depth_sf_.f_t_ - depth_sf_.a_tw_ * depth_sf_.f_z_) / common1;
        depth_sf_.b_tb_ = (depth_sf_.a_zw_ * depth_sf_.a_tb_ - depth_sf_.a_tw_ * depth_sf_.a_zb_) / common1;
        depth_sf_.b_ts_ = (depth_sf_.a_zw_ * depth_sf_.a_ts_ - depth_sf_.a_tw_ * depth_sf_.a_zs_) / common1;
        
        // printf("Temp{b_z:%f b_zb:%f b_zs:%f b_t:%f b_tb:%f b_ts:%f}\n", depth_sf_.b_z_, depth_sf_.b_zb_, depth_sf_.b_zs_, depth_sf_.b_t_, depth_sf_.b_tb_, depth_sf_.b_ts_);
        // printf("common1:%f\n", common1);

        // dot2_z & dot2_theta
        depth_sf_.g_z_ = depth_sf_.b_z_ * cos(kinetic_.theta_) - kinetic_.u_ * kinetic_.q_ * cos(kinetic_.theta_) - 
            kinetic_.w_ * kinetic_.q_ * sin(kinetic_.theta_);
        depth_sf_.g_zb_ = depth_sf_.b_zb_ * cos(kinetic_.theta_);
        depth_sf_.g_zs_ = depth_sf_.b_zs_ * cos(kinetic_.theta_);
        depth_sf_.g_t_ = depth_sf_.b_t_;
        depth_sf_.g_tb_ = depth_sf_.b_tb_;
        depth_sf_.g_ts_ = depth_sf_.b_ts_;
        // printf("Temp:{g_z:%f g_zb:%f g_ts:%f g_tb:%f g_zs:%f}\n", depth_sf_.g_z_, depth_sf_.g_zb_, depth_sf_.g_ts_, depth_sf_.g_tb_, depth_sf_.g_zs_);
        // printf("Temp:{b_z:%f theta:%f u:%f q:%f w:%f b_zb:%f b_zs:%f b_t:%f b_tb%f b_ts%f\n}", 
        //        depth_sf_.b_z_, kinetic_.theta_, kinetic_.u_, kinetic_.q_, kinetic_.w_, depth_sf_.b_zb_, depth_sf_.b_zs_, depth_sf_.b_t_, depth_sf_.b_tb_, depth_sf_.b_ts_);
        
        // depth & theta_dot
        depth_sf_.dot_z_ = -kinetic_.u_ * sin(kinetic_.theta_) + kinetic_.w_ * cos(kinetic_.theta_);
        depth_sf_.dot_theta_ = kinetic_.q_;

        /*
        // Horizontal variable substitution
        horizon_sf_.a_yv_ = body_.m_ - dynamic_.y_dotv_;
        horizon_sf_.a_yr_ = body_.m_ * body_.x_g_ - dynamic_.y_dotr_;
        horizon_sf_.a_ydr_ = force_.y_uudr_ * kinetic_.u_ * kinetic_.u_;
        horizon_sf_.f_y_ = body_.m_ * body_.y_g_ * kinetic_.r_ * kinetic_.r_ - 
            body_.m_ * kinetic_.u_ * kinetic_.r_ + dynamic_.x_dotu_ * kinetic_.u_ * kinetic_.r_ +
            dynamic_.y_vv_ * kinetic_.v_ * fabs(kinetic_.v_) + 
            dynamic_.y_uv_ * kinetic_.u_ * kinetic_.v_ + 
            dynamic_.y_rr_ * kinetic_.r_ * fabs(kinetic_.r_) + 
            dynamic_.y_ur_ * kinetic_.u_ * kinetic_.r_;

        // Rolling variable substitution 
        horizon_sf_.a_pv_ = body_.m_ * body_.x_g_ - dynamic_.n_dotv_;
        horizon_sf_.a_pr_ = body_.i_zz_ - dynamic_.n_dotr_;
        horizon_sf_.a_pdr_ = force_.n_uudr_ * kinetic_.u_ * kinetic_.u_;
        horizon_sf_.f_p_ = -body_.m_ * body_.x_g_ * kinetic_.u_ * kinetic_.r_ - 
            body_.m_ * body_.y_g_ * kinetic_.v_ * kinetic_.r_ + 
            (dynamic_.y_dotv_ * kinetic_.v_ + dynamic_.y_dotr_ * kinetic_.r_) * kinetic_.u_ - 
            dynamic_.x_dotu_ * kinetic_.u_ * kinetic_.v_ +
            dynamic_.n_vv_ * kinetic_.v_ * fabs(kinetic_.v_) + 
            dynamic_.n_uv_ * kinetic_.u_ * kinetic_.v_ + 
            dynamic_.n_rr_ * kinetic_.r_ * fabs(kinetic_.r_) + 
            dynamic_.n_ur_ * kinetic_.u_ * kinetic_.r_;

        // dot_v & dot_r
        double common2 = (horizon_sf_.a_yv_ * horizon_sf_.a_pr_ - horizon_sf_.a_pv_ * horizon_sf_.a_yr_);
        horizon_sf_.b_y_ = (horizon_sf_.a_pr_ * horizon_sf_.f_y_ - horizon_sf_.a_yr_ * horizon_sf_.f_p_) / common2;
        horizon_sf_.b_ydr_ = (horizon_sf_.a_pr_ * horizon_sf_.a_ydr_ - horizon_sf_.a_yr_ * horizon_sf_.a_pdr_) / common2;
        horizon_sf_.b_p_ = (horizon_sf_.a_yv_ * horizon_sf_.f_p_ - horizon_sf_.a_pv_ * horizon_sf_.f_y_) / common2;
        horizon_sf_.b_pdr_ = (horizon_sf_.a_yv_ * horizon_sf_.a_pdr_ - horizon_sf_.a_pv_ * horizon_sf_.a_ydr_) / common2;
        // printf("common2:%f\n", common2);

        // dot_psi
        horizon_sf_.dot_psi_ = kinetic_.r_;
        */

        // Slide Model Control
        // Depth
        slide_model_.z_.e_ = kinetic_.z_ - mission_.z_.ref_;
        slide_model_.z_.dot_e_ = depth_sf_.dot_z_ - mission_.z_.ref_dot_;
        slide_model_.z_.s_ = slide_model_.z_.dot_e_ + ctrl_.z_.c_ * slide_model_.z_.e_;
        // Pitch
        slide_model_.theta_.e_ = kinetic_.theta_ - mission_.theta_.ref_;
        slide_model_.theta_.dot_e_ = depth_sf_.dot_theta_ - mission_.theta_.ref_dot_;
        slide_model_.theta_.s_ = slide_model_.theta_.dot_e_ + ctrl_.theta_.c_ * slide_model_.theta_.e_;
        // Yaw
        // slide_model_.psi_.e_ = kinetic_.psi_ - mission_.psi_.ref_;
        // slide_model_.psi_.dot_e_ = horizon_sf_.dot_psi_ - mission_.psi_.ref_dot_;
        // slide_model_.psi_.s_ = slide_model_.psi_.dot_e_ + ctrl_.psi_.c_ * slide_model_.psi_.e_;

        // Intermediate quantity computation
        slide_model_.z_.l_ = mission_.z_.ref_dot2_ - depth_sf_.g_z_ - ctrl_.z_.c_ * slide_model_.z_.dot_e_ - 
            ctrl_.z_.k_ * pow(fabs(slide_model_.z_.s_), ctrl_.z_.alpha_) * sat(slide_model_.z_.s_, ctrl_.bondary_thick_);
        slide_model_.theta_.l_ = mission_.theta_.ref_dot2_ - depth_sf_.g_t_ - ctrl_.theta_.c_ * slide_model_.theta_.dot_e_ - 
            ctrl_.theta_.k_ * pow(fabs(slide_model_.theta_.s_), ctrl_.theta_.alpha_) * sat(slide_model_.theta_.s_, ctrl_.bondary_thick_);
        // slide_model_.psi_.l_ = mission_.psi_.ref_dot2_ - horizon_sf_.b_p_ - ctrl_.psi_.c_ * slide_model_.psi_.dot_e_ - 
        //     ctrl_.psi_.k_ * pow(fabs(slide_model_.psi_.s_), ctrl_.psi_.alpha_) * sat(slide_model_.psi_.s_, ctrl_.bondary_thick_);

        // Command computation
        double common3 = depth_sf_.g_zb_ * depth_sf_.g_ts_ - depth_sf_.g_tb_ * depth_sf_.g_zs_;
        // printf("Temp:{g_zb:%f g_ts:%f g_tb:%f g_zs:%f}\n", depth_sf_.g_zb_, depth_sf_.g_ts_, depth_sf_.g_tb_, depth_sf_.g_zs_);
        deltab_ = (slide_model_.z_.l_ * depth_sf_.g_ts_ - slide_model_.theta_.l_ * depth_sf_.g_zs_) / common3;
        deltas_ = (slide_model_.theta_.l_ * depth_sf_.g_zb_ - slide_model_.z_.l_ * depth_sf_.g_tb_) / common3;
        // deltar_ = slide_model_.psi_.l_ / horizon_sf_.b_pdr_;
        deltar_ = 0.0;
    
        // printf("common3:%f\n", common3);
        if(fabs(deltab_) > 45 / 57.3){
            deltab_ = (45 / 57.3) * sign(deltab_);
        }
        if(fabs(deltas_) > 45 / 57.3){
            deltas_ = (45 / 57.3) * sign(deltas_);
        }
        if(fabs(deltar_) > 45 / 57.3){
            deltar_ = (45 / 57.3) * sign(deltar_);
        }

        // Print control value of forward, afterward and orientation rouder
        printf("forward fin: %f afterward fin: %f rouder: %f", deltab_, deltas_, deltar_);

        output.fwd_fin_ = deltab_;
        output.aft_fin_ = deltas_;
        output.rouder_ = deltar_;
    };
    
    void QTController::pidControllerRun(const QTKineticSensor& sensor, const QTControllerInput& input, QTControllerOutput& output, const double dt)
    {
        // Update kinetic parameters
        kinetic_.setPosition(sensor.x_, sensor.y_, sensor.z_, sensor.roll_, sensor.pitch_, sensor.yaw_);
        // kinetic_.setVelocity(sensor.x_dot_, sensor.y_dot_, sensor.z_dot_, sensor.roll_dot_, sensor.pitch_dot_, sensor.yaw_dot_);
        kinetic_.setVelocity(sensor.x_dot_, 0.0, sensor.z_dot_, sensor.roll_dot_, sensor.pitch_dot_, sensor.yaw_dot_);

        mission_.vertical_dist_ = (kinetic_.x_ - input.x_d_) * sin(input.pitch_d_) - (kinetic_.z_ - input.depth_d_) * cos(input.pitch_d_);
        printf("VerticalDist:%f\n", mission_.vertical_dist_);

        double line_dir = atan2(0.0, input.x_d_);

        double ye = (kinetic_.z_ - input.depth_d_) * std::cos(line_dir);

        // LOS logic
        mission_.theta_.ref_ = -atan2(ye, factor_);
        mission_.theta_.ref_dot_ = (mission_.theta_.ref_ - mission_.theta_.pre_ref_) / dt;
        mission_.theta_.ref_dot2_ = (mission_.theta_.ref_dot_ - mission_.theta_.pre_ref_dot_) / dt;
        mission_.theta_.det_ = mission_.theta_.ref_ - (kinetic_.theta_ - line_dir);
        if(mission_.theta_.det_ > pi){
            mission_.theta_.det_ -= 2 * pi;
        }
        if(mission_.theta_.det_ < -pi){
            mission_.theta_.det_ += 2 * pi;
        }
        mission_.theta_.Update();

        printf("ye:%f line_dir:%f ref_picth:%f\n", ye, line_dir, mission_.theta_.ref_);

        double det_pitch = mission_.theta_.det_;
        double pre_det_pitch = mission_.theta_.det_pre_;
        double pre2_det_pitch = mission_.theta_.det_pre2_;
        double r = pid_.p_*(det_pitch - pre_det_pitch) + pid_.i_ * pre_det_pitch + pid_.d_ * (det_pitch - 2 * pre2_det_pitch + pre_det_pitch);
        deltab_ = r;
        deltas_ = deltab_;
        deltar_ = 0.0;
        
        mission_.theta_.Update();
        
        if(fabs(deltab_) > 25 / 57.3){
            deltab_ = (25 / 57.3) * sign(deltab_);
        }
        if(fabs(deltas_) > 25 / 57.3){
            deltas_ = (25 / 57.3) * sign(deltas_);
        }
        if(fabs(deltar_) > 25 / 57.3){
            deltar_ = (25 / 57.3) * sign(deltar_);
        }

        // Print control value of forward, afterward and orientation rouder
        printf("forward fin: %f afterward fin: %f rouder: %f", deltab_, deltas_, deltar_);

        output.fwd_fin_ = deltab_;
        output.aft_fin_ = deltas_;
        output.rouder_ = deltar_;
    };
}; // ns
