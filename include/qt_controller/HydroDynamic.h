/*
 * Filename: Dynamic.h
 * Path: auv_controller
 * Created Date: Monday, September 7th 2020, 8:51:07 pm
 * Author: zhao wang
 * 
 * Copyright (c) 2020 Arms Lab of Huazhong University of Science and Technology
 */
#ifndef HYDRODYNAMIC_H_
#define HYDRODYNAMIC_H_

#include <vector>
#include <assert.h>
#include <utility>
#include <exception>
#include <eigen3/Eigen/Dense>

using std::move;
using std::exception;
using std::vector;
using Eigen::MatrixXf;

namespace qt_controller{
/**
 * @brief Hydrodynmamic class
 */ 
class HydroDynamic{
public:
    /**
     * @brief Default constructor
     */ 
    HydroDynamic(){
        vector<double> D_temp(6, 0);
        setAssociatedMass(D_temp);
        setDamping(D_temp);
    }


    /**
     * @brief Constructor
     * @param D D factors
     */ 
    HydroDynamic(vector<double>& D){
        setAssociatedMass(D);
        setDamping(D);
    }

    /**
     * @brief Constructor
     * @param ass_mass Associated mass vector
     * @param damping Damping vector
     */ 
    HydroDynamic(const vector<double>& ass_mass, const vector<double>& damping, const vector<double>& ctrl_input){
        assert(ass_mass.size() == ass_mass_num_);
        // associated mass parameters initialization
        for(int i = 0; i < ass_mass.size(); ++i){
            ass_mass_ << ass_mass[i];
        }
        printf("associated mass matrix initializing done");

        assert(damping_.size() == damping_num_);
        // damping matrix parameters initialization
        for(int i = 0; i < damping.size(); ++i){
            damping_ << damping[i];
        }
        printf("damping matrix initializing done");
    }

    /**
     * @brief Constructor
     * @param ass_mass Associated mass vector
     * @param damping Damping vector
     */ 
    HydroDynamic(const double ass_mass[36], const double damping[36], const double ctrl_input[16]){
        try{
            // associated mass parameters initialization
            for(int i = 0; i < 36; ++i){
                ass_mass_ << ass_mass[i];
            }
            printf("associated mass matrix initializing done");
            
            // damping matrix parameters initialization
            for(int i = 0; i < 36; ++i){
                damping_ << damping[i];
            }
            printf("damping matrix initializing done");
        }
        catch(exception& e){
            printf("error in hydrodynamic parameters initialization: %s", e.what());
        }
    }

    /**
     * @brief Set associated mass parameters
     * @param vec Input vector
     * @return Return true on parameters setting successfully, false on setting failed.
     */ 
    bool setAssociatedMass(const vector<double>& vec){
        try{
            ass_mass_ << -0.000949 * vec[3]; ass_mass_ << 0.0; ass_mass_  << 0.0; ass_mass_ << 0.0; ass_mass_ << 0.0; ass_mass_ << 0.0;
            ass_mass_ << 0.0;  ass_mass_ << -0.0408 * vec[3]; ass_mass_ << 0.0; ass_mass_ << 0.000547 * vec[4]; ass_mass_ << 0.0;  ass_mass_[11] = 0.00139 * vec[4];
            ass_mass_ << 0.0; ass_mass_ << 0.0; ass_mass_ << -0.0226 * vec[3]; ass_mass_ << 0.0; ass_mass_[16] = -0.0000510 * vec[4]; ass_mass_ << 0.0;
            ass_mass_ << 0.0; ass_mass_ << 0.000547 * vec[4]; ass_mass_ << 0.0; ass_mass_ << -0.000105 * vec[5]; ass_mass_ << 0.0; ass_mass_ << -0.00000370 * vec[5];
            ass_mass_ << 0.0; ass_mass_ << 0.0; ass_mass_ << -0.0000510 * vec[4]; ass_mass_ << 0.0; ass_mass_ << -0.00175 * vec[5]; ass_mass_ << 0.0;
            ass_mass_ << 0.0; ass_mass_ << 0.00139 * vec[4]; ass_mass_ << 0.0; ass_mass_ << -0.000003370 * vec[5]; ass_mass_ << 0.0; ass_mass_ << -0.00183 * vec[5];
        }
        catch(exception& e){
            printf("error to set associatevec mass parameters: %s", e.what());
            return false;
        }
    }

    /**
     * @brief Set associated mass parameters
     * @param ass_mass Associated mass vector
     * @return Return true on parameters setting successfully, false on setting failed.
     */ 
    bool setAssociatedMass(const double ass_mass[36]){
        try{
            // associated mass parameters initialization
            for(int i = 0; i < ass_mass_num_; ++i){
                ass_mass_ << ass_mass[i];
            }
            printf("success to set associated mass parameters");
            return true;
        }
        catch(exception& e){
            printf("error to set associated mass parameters");
            return false;
        }
    }

    /**
     * @brief Set damping parameters
     * @param D D factors
     * @return Return true on parameters setting successfully, false on setting failed.
     */ 
    bool setDamping(const vector<double>& D){
        try{
            damping_ << -0.00315 * D[2]; damping_ << 0.0; damping_  << 0.0; damping_ << 0.0; damping_ << 0.0; damping_ << 0.0;
            damping_ << 0.0;  damping_ << -0.099 * D[2]; damping_ << 0.0; damping_ <<  0.00677 * D[3]; damping_ << 0.0;  damping_[11] = 0.0250 * D[3];
            damping_ << 0.0; damping_ << 0.0; damping_ << -0.067 * D[2]; damping_ << 0.0; damping_[16] = -0.00658 * D[3]; damping_ << 0.0;
            damping_ << 0.0; damping_ << 0.00406 * D[3]; damping_ << 0.0; damping_ << -0.00145 * D[4]; damping_ << 0.0; damping_ << -0.000578 * D[4];
            damping_ << 0.0; damping_ << 0.0; damping_ << 0.00120 * D[3]; damping_ << 0.0; damping_ << -0.00687 * D[4]; damping_ << 0.0;
            damping_ << 0.0; damping_ << -0.005 * D[3]; damping_ << 0.0; damping_ << 0.000086 * D[4]; damping_ << 0.0; damping_ << -0.00747 * D[4];
        }
        catch(exception& e){
            printf("error to set associated mass parameters: %s", e.what());
            return false;
        }
    }
    
    /**
     * @brief Set Damping parameters
     * @param damping Damping vector
     * @return Return true on parameters setting successfully, false on setting failed.
     */ 
    bool setDamping(const double damping[36]){
        try{
            // damping parameters initialization
            for(int i = 0; i < damping_num_; ++i){
                damping_ << damping[i];
            }
            printf("success to set damping parameters");
            return true;
        }
        catch(exception& e){
            printf("error to set damping parameters");
            return false;
        }
    }

private:
    /**
     * xudot xvdot xwdot xpdot xqdot xrdot
     * yudot yvdot ywdot ypdot yqdot yrdot
     * zudot zvdot zwdot zpdot zqdot zrdot
     * kudot kvdot kwdot kpdot kqdot krdot
     * mudot mvdot mwdot mpdot mqdot mrdot
     * nudot nvdot nwdot npdot nqdot nrdot
     */ 
    MatrixXf ass_mass_; // associated mass matrix
    
    /**
     * Xuu Xuv Xuw Xup Xuq Xur
     * Yuu Yuv Yuw Yup Yuq Yur
     * Zuu Zuv Zuw Zup Zuq Zur
     * Kuu Kuv Kuw Kup Kuq Kur
     * Muu Muv Muw Mup Muq Mur
     * Nuu Nuv Nuw Nup Nuq Nur
     */
    MatrixXf damping_; // damping matrix
    
    const int ass_mass_num_ = 36;
    const int damping_num_ = 36;
}; // HydroDynamic
};// ns

#endif
