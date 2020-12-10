#ifndef BODYPARAMS_H_
#define BODYPARAMS_H_

#include <stdio.h>
#include <utility>
#include <assert.h>
#include <vector>
#include <math.h>
#include <exception>

using std::vector;
using std::exception;

const double PI = 3.141592654;

namespace qt_controller{
/**
 * @brief Body parameters of QT
 */ 
class BodyParams
{
public:
    /**
     * @brief Default constructor
     */
    BodyParams(){
        len_ = 8.534; mass_ = 4390; b_ = 4390; rho_ = 1025; grav_ = 9,81;
        weight_ = mass_ * grav_; bouy_ = b_ * grav_;
        
        // Body coordinate
        mx_ = 0; my_ = 0; mz_ = 0;
        bx_ = 0; by_ = 0; bz_ = -0.137;
        
        // Inertial
        ixx_ = 1315; iyy_ = 5900; izz_ = 5057;
        ixy_ = 0.0; ixz_ = 0.0;
        iyx_ = 0.0; iyz_ = 0.0;
        izx_ = 0.0; izy_ = 0.0;
        
        // D factors
        D_.resize(6, 0.0);
        D_[0] = 0.5 * rho_; D_[1] = D_[0] * len_; D_[2] = D_[1] * len_; D_[3] =
        D00_ = 0.7159;
        D04_ = std::pow(D00_, 4);
    }

    /**
     * @brief Constructor with inputs
     */ 
    BodyParams(const vector<double>& body_params){
        assert(body_params.size() == params_num_);
        len_ = body_params[0]; mass_ = body_params[1]; b_ = body_params[2]; 
        rho_ = body_params[3]; grav_ = body_params[4];
        weight_ = mass_ * grav_; bouy_ = b_ * grav_;
        
        // Body coordinate
        mx_ = body_params[5]; my_ = body_params[6]; mz_ = body_params[7];
        bx_ = body_params[8]; by_ = body_params[9]; bz_ = body_params[10];
        
        // Inertial
        ixx_ = body_params[11]; iyy_ = body_params[12]; izz_ = body_params[13];
        ixy_ = body_params[14]; ixz_ = body_params[15];
        iyx_ = body_params[16]; iyz_ = body_params[17];
        izx_ = body_params[18]; izy_ = body_params[19];
        
        // D factors
        D_.resize(5);
        D_[0] = 0.5 * rho_; D_[1] = D_[0] * len_; D_[2] = D_[1] * len_; D_[3] =
        D00_ = body_params[20];
        D04_ = std::pow(D00_, 4);
    }

    /**
     * @brief Constructor with inputs
     */ 
    BodyParams(const double body_params[21]){
        len_ = body_params[0]; mass_ = body_params[1]; b_ = body_params[2]; 
        rho_ = body_params[3]; grav_ = body_params[4];
        weight_ = mass_ * grav_; bouy_ = b_ * grav_;
        
        // Body coordinate
        mx_ = body_params[5]; my_ = body_params[6]; mz_ = body_params[7];
        bx_ = body_params[8]; by_ = body_params[9]; bz_ = body_params[10];
        
        // Inertial
        ixx_ = body_params[11]; iyy_ = body_params[12]; izz_ = body_params[13];
        ixy_ = body_params[14]; ixz_ = body_params[15];
        iyx_ = body_params[16]; iyz_ = body_params[17];
        izx_ = body_params[18]; izy_ = body_params[19];
        
        // D factors
        D_.resize(5);
        D_[0] = 0.5 * rho_; D_[1] = D_[0] * len_; D_[2] = D_[1] * len_; D_[3] =
        D00_ = body_params[20];
        D04_ = std::pow(D00_, 4);
    }

    /**
     * @brief Returns D factors
     * @return D_
     */ 
    vector<double>& getDfactors(){
        return D_;
    }

    /**
     * @brief Return D04
     * @return D04
     */ 
    double getD04(){
        return D04_;
    }

    /**
     * @brief rho 
     */
    double getRho(){
        return rho_;
    }

    /**
     * @brief Interface to set body parameters 
     * @param body_params Body parameters vector
     * @return True or False on whether the body parameters are set successfully
     */ 
    bool setBodyParams(const vector<double>& body_params){
        try{
            len_ = body_params[0]; mass_ = body_params[1]; b_ = body_params[2]; 
            rho_ = body_params[3]; grav_ = body_params[4];
            weight_ = mass_ * grav_; bouy_ = b_ * grav_;
            
            // Body coordinate
            mx_ = body_params[5]; my_ = body_params[6]; mz_ = body_params[7];
            bx_ = body_params[8]; by_ = body_params[9]; bz_ = body_params[10];
            
            // Inertial
            ixx_ = body_params[11]; iyy_ = body_params[12]; izz_ = body_params[13];
            ixy_ = body_params[14]; ixz_ = body_params[15];
            iyx_ = body_params[16]; iyz_ = body_params[17];
            izx_ = body_params[18]; izy_ = body_params[19];
            
            // D factors
            D_.resize(5);
            D_[0] = 0.5 * rho_; D_[1] = D_[0] * len_; D_[2] = D_[1] * len_; D_[3] =
            D00_ = body_params[20];
            D04_ = std::pow(D00_, 4);
            return true;
        }
        catch(exception& e){
            printf("Set body parameters failed: %s", e.what());
            return false;
        }
    }

    /**
     * @brief Interface to set body parameters 
     * @param body_params Body parameters array
     * @return True or False on whether the body parameters are set successfully
     */ 
    bool setBodyParams(double body_params[21]){
        try{
            len_ = body_params[0]; mass_ = body_params[1]; b_ = body_params[2]; 
            rho_ = body_params[3]; grav_ = body_params[4];
            weight_ = mass_ * grav_; bouy_ = b_ * grav_;
            
            // Body coordinate
            mx_ = body_params[5]; my_ = body_params[6]; mz_ = body_params[7];
            bx_ = body_params[8]; by_ = body_params[9]; bz_ = body_params[10];
            
            // Inertial
            ixx_ = body_params[11]; iyy_ = body_params[12]; izz_ = body_params[13];
            ixy_ = body_params[14]; ixz_ = body_params[15];
            iyx_ = body_params[16]; iyz_ = body_params[17];
            izx_ = body_params[18]; izy_ = body_params[19];
            
            // D factors
            D_.resize(5);
            D_[0] = 0.5 * rho_; D_[1] = D_[0] * len_; D_[2] = D_[1] * len_; D_[3] =
            D00_ = body_params[20];
            D04_ = std::pow(D00_, 4);
            return true;
        }
        catch(exception& e){
            printf("Set body parameters failed: %s", e.what());
            return false;
        }
    }
    
    /**
     * @brief Deconstructor
     */ 
    ~BodyParams(){};

private:
    double len_, mass_, b_, rho_, grav_, weight_, bouy_;
    double mx_, my_, mz_;
    double bx_, by_, bz_;
    double ixx_, iyy_, izz_, ixy_, ixz_, iyx_, iyz_, izx_, izy_;
    vector<double> D_; // D0, D1, D2, D3, D4, D5
    double D00_, D04_;
    const double A_ = PI * 0.5 * 0.5;

    const unsigned int params_num_ = 21;
};
}; // ns
#endif
