//
// Created by GING on 2019-01-01.
//

#include "PAIGE_Feature.h"

namespace PAIGE
{
    //Normalize PAIGE feature
    void PAIGE_Feature::normalize() {
        total_feat_.normalize();
    }

    PAIGE_Feature::PAIGE_Feature() : d_delta_x(50), d_delta_y(50), d_delta_o(100)
    {
        dx_.resize(d_delta_x);
        dy_.resize(d_delta_y);
        do_.resize(d_delta_o);
        total_feat_.resize(d_delta_x+d_delta_y+d_delta_o);
        initialize_dx_();
        initialize_dy_();
        initialize_do_();
        initialize_total_();
    }

    //Generate final PAIGE feature
    void PAIGE_Feature::merge()
    {
        //assign dX
        for(int i=0;i<d_delta_x;++i)
        {
            total_feat_(i)=dx_(i);
        }

        //assign dY
        for(int i=0;i<d_delta_y;++i)
        {
            total_feat_(d_delta_x+i)=dy_(i);
        }

        //assign dO
        for(int i=0;i<d_delta_o;++i)
        {
            total_feat_(d_delta_x+d_delta_y+i)=do_(i);
        }
    }

    void PAIGE_Feature::addDx(int index,float value)
    {
        dx_(index)+=value;
    }

    void PAIGE_Feature::addDy(int index,float value)
    {
        dy_(index)+=value;
    }

    void PAIGE_Feature::addDo(int index,float value)
    {
        do_(index)+=value;
    }

    void PAIGE_Feature::resetDeltaX(int new_DX) {
        d_delta_x = new_DX;
        initialize_dx_();
    }

    void PAIGE_Feature::resetDeltaY(int new_DY) {
        d_delta_x = new_DY;
        initialize_dy_();
    }

    void PAIGE_Feature::resetDeltaO(int new_DO) {
        d_delta_o = new_DO;
        initialize_do_();
    }

    int PAIGE_Feature::getDeltaX()
    {
        return d_delta_x;
    }

    int PAIGE_Feature::getDeltaY()
    {
        return d_delta_y;
    }

    int PAIGE_Feature::getDeltaO()
    {
        return d_delta_o;
    }


    void PAIGE_Feature::initialize_dx_()
    {
        for(int i=0;i<d_delta_x;++i)
        {
            dx_(i)=0;
        }
    }

    void PAIGE_Feature::initialize_dy_()
    {
        for(int i=0;i<d_delta_y;++i)
        {
            dy_(i)=0;
        }
    }

    void PAIGE_Feature::initialize_do_()
    {
        for(int i=0;i<d_delta_o;++i)
        {
            do_(i)=0;
        }
    }

    void PAIGE_Feature::initialize_total_()
    {
        for(int i=0;i<d_delta_x+d_delta_y+d_delta_o;++i)
        {
            total_feat_(i)=0;
        }
    }

    int PAIGE_Feature::getSize()
    {
        return getDeltaX()+getDeltaY()+getDeltaO();
    }

    PAIGE_DX PAIGE_Feature::getDx()
    {
        return dx_;
    }

    PAIGE_DY PAIGE_Feature::getDy()
    {
        return dy_;
    }

    PAIGE_DO PAIGE_Feature::getDo()
    {
        return do_;
    }

    Eigen::VectorXf PAIGE_Feature::getPAIGE()
    {
        return total_feat_;
    }
}

