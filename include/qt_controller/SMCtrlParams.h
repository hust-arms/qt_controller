#ifndef SMCTRLPARAMS_H_
#define SMCTRLPARAMS_H_

namespace qt_controller{

/**
 * @brief Slide model control parameters
 */ 
struct SMCtrlParams
{
    double c_, k_, alpha_;
    double boundary_thick_;

    /**
     * @brief Set control parameters
     */ 
    void setCtrlParams(double c, double k, double alpha, double bt){
        c_ = c;
        k_ = k;
        alpha_ = alpha;
        boundary_thick_ = bt;
    }
}; // SMCtrlParams
}; // ns

#endif
