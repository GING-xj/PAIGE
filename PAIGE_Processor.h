//
// Created by GING on 2019-01-03.
//



#ifndef PAIGE_PAIGE_CALCULATOR_H
#define PAIGE_PAIGE_CALCULATOR_H

#include <map>
#include <vector>
#include <Eigen/Dense>
#include "PAIGE_Feature.h"
#include "openMVG/features/regions_factory.hpp"
#include "openMVG/sfm/sfm_data.hpp"
#include "Histogram_Block.h"


namespace PAIGE
{

    class Histogram_Block;

    class PAIGE_Processor {

    public:
        PAIGE_Processor():HIST_NUMS(10),DESC_DIM(128) {}

        //marginal bin in F_1
        using INT_BIN=std::map<int, int>;
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

        void calHistograms(openMVG::features::SIFT_Regions * in_sift_regions,
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

        int calHistogramsFromImages(const std::string & img_path);


        int calHistogramsFromSfMData(const std::string & dir_to_sfm_data_string);

        //calculate PAIGE features and ground truth
        int calPAIGE_and_GT_fromJson(const std::string & json_path_string);

        bool calGroundTruthLabel(const std::string & imageNameL,const std::string & imageNameR);

    private:
        int HIST_NUMS;
        int DESC_DIM;
        std::unordered_map<int,std::shared_ptr<Histogram_Block>> _hist_blocks;


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

        std::vector<Eigen::VectorXf> normalize_desc(openMVG::features::SIFT_Regions::DescsT descs)
        {
            std::vector<Eigen::VectorXf> normalized_desc_vec;
            normalized_desc_vec.reserve(descs.size());

            for(const auto & desc:descs)
            {
                Eigen::VectorXf v(DESC_DIM);
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



#endif //PAIGE_PAIGE_CALCULATOR_H
