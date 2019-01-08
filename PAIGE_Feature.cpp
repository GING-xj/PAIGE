//
// Created by GING on 2019-01-01.
//

#include "PAIGE_Feature.h"

namespace PAIGE
{
    void PAIGE_Feature::normalize() {
        dx_.normalize();
        dy_.normalize();
        do_.normalize();
    }

    PAIGE_Feature::PAIGE_Feature() : d_delta_x(50), d_delta_y(50), d_delta_o(100)
    {
        dx_.resize(d_delta_x);
        dy_.resize(d_delta_y);
        do_.resize(d_delta_o);
        initialize_dx_();
        initialize_dy_();
        initialize_do_();
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

    int PAIGE_Feature::getSize()
    {
        return getDeltaX()+getDeltaY()+getDeltaO();
    }
}

