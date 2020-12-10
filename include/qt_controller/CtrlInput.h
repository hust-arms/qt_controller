#ifndef CTRLINPUT_H_
#define CTRLINPUT_H_

#include <stdio.h> 
#include <vector>
#include <exception>

using std::vector;
using std::exception;

namespace qt_controller{
/**
 * @brief Control input of QT
 */ 
class CtrlInput
{
public:
    /**
     * @brief Constructor
     * @param D D factor
     */ 
    CtrlInput(double rho, double D04, double D2) {
        rouder_.resize(rouder_num_, 0.0);
        rouder_[0] = 1.865; rouder_[1] = 1.865; rouder_[2] = -3.369; rouder_[3] = rouder_[2]; rouder_[4] = rouder_[2];
        rouder_[5] = -0.699; rouder_[6] = -rouder_[5]; rouder_[7] = -0.621; rouder_[8] = -rouder_[7]; rouder_[9] = 0.0;
        rouder_[10] = 0.013; rouder_[11] = 0.013; rouder_[12] = -0.0844; rouder_[13] = -0.0844; rouder_[14] = 0.0;

        setCtrlInput(rho, D04, D2);
    }
   
    /**
     * @brief Deconstructor
     */ 
    ~CtrlInput() {}

    /**                                                              
     * @brief Set control input parameters
     * @param D2 D factors
     * @return Return true on parameters setting successfully, false 
     */ 
    bool setCtrlInput(double rho, double D04, double D2){
        try{
            ctrl_input_[0] = 0.1351 * rho * D04;
            ctrl_input_[1] = 0.00917 * D2;
            ctrl_input_[2] = -0.010 * D2; ctrl_input_[3] = -0.010 * D2; ctrl_input_[4] = -0.010 * D2; ctrl_input_[5] = -0.010 * D2; 
            ctrl_input_[6] = rouder_[5] * ctrl_input_[2]; ctrl_input_[7] = rouder_[6] * ctrl_input_[3]; ctrl_input_[8] = rouder_[7] * ctrl_input_[4]; ctrl_input_[9] = rouder_[8] * ctrl_input_[5]; ctrl_input_[10] = -rouder_[14] * ctrl_input_[1];
            ctrl_input_[11] = -rouder_[0] * ctrl_input_[2]; ctrl_input_[12] = rouder_[1] * ctrl_input_[3]; ctrl_input_[13] = rouder_[2] * ctrl_input_[4]; ctrl_input_[14] = -rouder_[3] * ctrl_input_[5];
            ctrl_input_[15] = rouder_[4] * ctrl_input_[1];
        }
        catch(exception& e){
            printf("error to set associated mass parameters: %s", e.what());
            return false;
        }
    }


    /**                                                              
     * @brief Set ctrl_input parameters
     * @param ctrl_input Ctrl input vector
     * @return Return true on parameters setting successfully, false 
     */ 
    bool setCtrlInput(const vector<double>& ctrl_input){
        ctrl_input_.resize(ctrl_input_num_);
        try{
            // damping parameters initialization
            for(int i = 0; i < ctrl_input.size(); ++i){
                ctrl_input_[i] = ctrl_input[i];
            }
            printf("success to set damping parameters");
            return true;
        }
        catch(exception& e){
            printf("error to set damping mass parameters: %s", e.what());
            return false;
        }
    }


    /**
     * @brief Set ctrl_input parameters
     * @param ctrl_input Ctrl input vector
     * @return Return true on parameters setting successfully, false 
     */ 
    bool setCtrlInput(const double ctrl_input[16]){
        ctrl_input_.resize(ctrl_input_num_);
        try{
            // damping parameters initialization
            for(int i = 0; i < ctrl_input_num_; ++i){
                ctrl_input_[i] = ctrl_input[i];
            }
            printf("success to set damping parameters");
            return true;
        }
        catch(exception& e){
            printf("error to set damping mass parameters: %s", e.what());
            return false;
        }
    }


private:
    /**
     * Xdndn Ydr Zdfp Zdfs Zdap Zdas Kdfp Kdfs Kdap Kdas Kdr Mdfp Mdf
     */
    vector<double> ctrl_input_; // control input

    /**
     * xfp xfs xap xas xr 
     * yfp yfs yap yas yr 
     * zfp zfs zap zas zr
     */ 
    vector<double> rouder_; 

    const int ctrl_input_num_ = 16;
    const int rouder_num_ = 15;
};
}; // ns
#endif
