//
// Created by GING on 2019-01-04.
//



#ifndef PAIGE_PAIGE_TEST_H
#define PAIGE_PAIGE_TEST_H

#include <map>
#include <Eigen/Dense>
#include "openMVG/features/regions_factory.hpp"
#include "PAIGE_Feature.h"

namespace PAIGE_TEST
{

    class PAIGE_test {
    public:
        PAIGE_test():HIST_NUMS(2),DESC_DIM(3) {}

        //marginal bin in F_1
        using INT_BIN=std::map< int, int>;
        //marginal bin in F_x, F_y, F_o
        using FLOAT_BIN=std::map< int,float>;
        //histogram of F_1
        using INT_HISTOGRAM=std::map< int,INT_BIN >;
        //
        using FLOAT_HISTOGRAM=std::map< int,FLOAT_BIN >;

        using INT_HISTOGRAMS=std::map< int,INT_HISTOGRAM >;

        using FLOAT_HISTOGRAMS=std::map< int,FLOAT_HISTOGRAM >;

        using PAIGE_DX=std::map< int, int>;

        using PAIGE_DY=std::map< int, int>;

        using PAIGE_DO=std::map< int, int>;

        void setNumOfHistogram(int new_hist_num)
        {
            HIST_NUMS=new_hist_num;
        }

        void setDescDimension(int new_desc_dim)
        {
            DESC_DIM=new_desc_dim;
        }

        void calHistograms(
                        std::vector<float> x_vector,std::vector<float> y_vector,std::vector<float> o_vector,
                        std::vector<Eigen::VectorXf> descriptors,
                           INT_HISTOGRAMS & out_1_hist,
                           FLOAT_HISTOGRAMS & out_x_hist,
                           FLOAT_HISTOGRAMS & out_y_hist,
                           FLOAT_HISTOGRAMS & out_o_hist,
                           int width,
                           int height);

        void calPAIGE_Feature(INT_HISTOGRAMS & in_1_hist_1,
                              FLOAT_HISTOGRAMS & in_x_hist_1,
                              FLOAT_HISTOGRAMS & in_y_hist_1,
                              FLOAT_HISTOGRAMS & in_o_hist_1,
                              INT_HISTOGRAMS & in_1_hist_2,
                              FLOAT_HISTOGRAMS & in_x_hist_2,
                              FLOAT_HISTOGRAMS & in_y_hist_2,
                              FLOAT_HISTOGRAMS & in_o_hist_2,
                              PAIGE::PAIGE_Feature & paige_feature_forward,
                              PAIGE::PAIGE_Feature & paige_feature_backward);

    private:
        int HIST_NUMS;
        int DESC_DIM;

        void normalize_x(std::vector<float> & x_vec,int width)
        {
            float width_float=width;
            for(auto & x:x_vec)
            {
                x=x/width_float;
            }
        }

        void normalize_y(std::vector<float> & y_vec,int height)
        {
            float height_float=height;
            for(auto & y:y_vec)
            {
                y=y/height_float;
            }

        }

        std::vector<Eigen::VectorXf> normalize_desc(std::vector<Eigen::VectorXf> descs)
        {
            std::vector<Eigen::VectorXf> normalized_desc_vec;
            normalized_desc_vec.reserve(descs.size());

            Eigen::VectorXf v(DESC_DIM);
            for(const auto & desc:descs)
            {
                for(int i=0;i<DESC_DIM;++i)
                {
                    v(i)=desc(i,0);
                }
                v.normalize();
                normalized_desc_vec.push_back(v);
            }

            return normalized_desc_vec;
        }
    };
}



#endif //PAIGE_PAIGE_TEST_H
