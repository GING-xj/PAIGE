//
// Created by GING on 2018-12-31.
//

#include <map>

#include "Definition.h"
#include "openMVG/features/regions_factory.hpp"
#include "PAIGE_Feature.h"


#ifndef PAIGE_PAIGE_CAL_H
#define PAIGE_PAIGE_CAL_H

using namespace openMVG::features;

void calHistograms(SIFT_Regions * in_sift_regions,INT_HISTOGRAMS out_1_hist,DOUBLE_HISTOGRAMS out_x_hist,DOUBLE_HISTOGRAMS out_y_hist,
        DOUBLE_HISTOGRAMS out_o_hist)
{
    SIFT_Regions::FeatsT features=in_sift_regions->Features();
    SIFT_Regions::DescsT descriptors=in_sift_regions->Descriptors();

    //normalize 1,x,y,o(reg)

    INT_HISTOGRAMS int_histograms;
    DOUBLE_HISTOGRAMS x_histograms,y_histograms,o_histograms;

    int histogram_start_int{0};
    int stride_int{0};

    int descs_size_int=descriptors.size();
    float x_float{0},y_float{0},o_float{0};
    int bin_location_int{0};

    for(int r=0;r<HIST_NUMS;++r)
    {
        for(int i=0;i<descs_size_int;++i)
        {
            for(int dim=0;dim<DESC_DIM;++dim)
            {
                x_float=features[i].x();
                y_float=features[i].y();
                o_float=features[i].orientation();

                bin_location_int=histogram_start_int+stride_int;

                histogram_start_int=pow(2,r)-1;
                stride_int=(double)descriptors[i](dim,0)/pow(2.0,-r);
                int_histograms[r][bin_location_int][dim]+=1;
                x_histograms[r][bin_location_int][dim]+=x_float;
                y_histograms[r][bin_location_int][dim]+=y_float;
                o_histograms[r][bin_location_int][dim]+=o_float;
            }
        }
    }
}


void calPAIGE_Feature(INT_HISTOGRAMS in_1_hist_1,DOUBLE_HISTOGRAMS in_x_hist_1,DOUBLE_HISTOGRAMS in_y_hist_1,
                      INT_HISTOGRAMS in_1_hist_2,DOUBLE_HISTOGRAMS in_x_hist_2,DOUBLE_HISTOGRAMS in_y_hist_2,
                      DOUBLE_HISTOGRAMS in_o_hist,PAIGE::PAIGE_Feature paige_feature_out)
{
    PAIGE::PAIGE_Feature paige_feature;
    double dx{0},dy{0},dz{0};

    for(int r=0;r<HIST_NUMS;++r)
    {
        for(int j=0;j<int(pow(2,r));++j)
        {
            for(int dim=0;dim<128;++dim)
            {
                if(in_1_hist_1[r][j][dim]!=0 && in_1_hist_2[r][j][dim]!=0)
                {

                }
            }
        }
    }
}


#endif //PAIGE_PAIGE_CAL_H
