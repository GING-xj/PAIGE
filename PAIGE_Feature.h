//
// Created by GING on 2019-01-01.
//



#ifndef PAIGE_PAIGE_FEATURE_H
#define PAIGE_PAIGE_FEATURE_H

#include <vector>
#include <Eigen/Dense>


namespace PAIGE {

    using PAIGE_DX=Eigen::VectorXf;

    using PAIGE_DY=Eigen::VectorXf;

    using PAIGE_DO=Eigen::VectorXf;

    class PAIGE_Feature {

    public:


        PAIGE_Feature();

        void addDx(int index,float value);

        void addDy(int index,float value);

        void addDo(int index,float value);

        void resetDeltaX(int new_DX) ;

        void resetDeltaY(int new_DY);

        void resetDeltaO(int new_DO);

        void merge();

        int getDeltaX();

        int getDeltaY();

        int getDeltaO();

        void normalize();

        int getSize();

        PAIGE_DX getDx();

        PAIGE_DY getDy();

        PAIGE_DO getDo();

        Eigen::VectorXf getPAIGE();

    private:
        PAIGE_DX dx_;
        PAIGE_DY dy_;
        PAIGE_DO do_;
        int d_delta_x;
        int d_delta_y;
        int d_delta_o;

        Eigen::VectorXf total_feat_;

        void initialize_dx_();

        void initialize_dy_();

        void initialize_do_();

        void initialize_total_();
    };
}




#endif //PAIGE_PAIGE_FEATURE_H
