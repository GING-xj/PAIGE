//
// Created by GING on 2019-01-01.
//

#include <vector>
#include <Eigen/Dense>
#include "Definition.h"

#ifndef PAIGE_PAIGE_FEATURE_H
#define PAIGE_PAIGE_FEATURE_H

namespace PAIGE {
    class PAIGE_Feature {
    public:

        using PAIGE_DX=Eigen::VectorXf;

        using PAIGE_DY=Eigen::VectorXf;

        using PAIGE_DO=Eigen::VectorXf;

        PAIGE_Feature();

        void addDx(int index,float value);

        void addDy(int index,float value);

        void addDo(int index,float value);

        void resetDeltaX(int new_DX) ;

        void resetDeltaY(int new_DY);

        void resetDeltaO(int new_DO);

        int getDeltaX();

        int getDeltaY();

        int getDeltaO();

        void normalize();

        int getSize();

        PAIGE_DX getDx();

        PAIGE_DY getDy();

        PAIGE_DO getDo();

    private:
        PAIGE_DX dx_;
        PAIGE_DY dy_;
        PAIGE_DO do_;
        int d_delta_x;
        int d_delta_y;
        int d_delta_o;

        void initialize_dx_();

        void initialize_dy_();

        void initialize_do_();
    };
}




#endif //PAIGE_PAIGE_FEATURE_H
