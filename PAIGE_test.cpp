//
// Created by GING on 2019-01-04.
//

#include "PAIGE_test.h"

namespace PAIGE_TEST
{

    void PAIGE_test::calHistograms(
            std::vector<float> x_vector,std::vector<float> y_vector,std::vector<float> o_vector,
            std::vector<Eigen::VectorXf> descriptors,
            INT_HISTOGRAMS & out_1_hist,
            FLOAT_HISTOGRAMS & out_x_hist,
            FLOAT_HISTOGRAMS & out_y_hist,
            FLOAT_HISTOGRAMS & out_o_hist,
            int width,
            int height)
    {

        //normalize desc,x,y
        std::vector<Eigen::VectorXf> normalized_desc_vec=normalize_desc(descriptors);
        normalize_x(x_vector,width);
        normalize_y(y_vector,height);


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
                    x_float=x_vector[i];
                    y_float=y_vector[i];
                    o_float=o_vector[i];

                    histogram_start_int=pow(2,r)-1;
                    stride_int=(double)normalized_desc_vec[i](dim)/pow(2.0,-r);

                    if(stride_int==r+1)
                        stride_int-=1;

                    bin_location_int=histogram_start_int+stride_int;

                    out_1_hist[r][bin_location_int][dim]+=1;
                    out_x_hist[r][bin_location_int][dim]+=x_float;
                    out_y_hist[r][bin_location_int][dim]+=y_float;
                    out_o_hist[r][bin_location_int][dim]+=o_float;
                }
            }
        }
    }


    void PAIGE_test::calPAIGE_Feature(
            INT_HISTOGRAMS & in_1_hist_1,
            FLOAT_HISTOGRAMS & in_x_hist_1,
            FLOAT_HISTOGRAMS & in_y_hist_1,
            FLOAT_HISTOGRAMS & in_o_hist_1,
            INT_HISTOGRAMS & in_1_hist_2,
            FLOAT_HISTOGRAMS & in_x_hist_2,
            FLOAT_HISTOGRAMS & in_y_hist_2,
            FLOAT_HISTOGRAMS & in_o_hist_2,
            PAIGE::PAIGE_Feature & paige_feature_forward,
            PAIGE::PAIGE_Feature & paige_feature_backward)
    {

        float Dx{0},Dy{0},Do{0},Dx_shifted{0},Dy_shifted{0},Do_shifted{0};
        float Dx_minus{0},Dy_minus{0},Do_minus{0},Dx_minus_shifted{0},Dy_minus_shifted{0},Do_minus_shifted{0};
        int index_of_dx{0},index_of_dy{0},index_of_do{0};
        int index_of_minus_dx{0},index_of_minus_dy{0},index_of_minus_do{0};
        int d_delta_x,d_delta_y,d_delta_o;
        d_delta_x=paige_feature_forward.getDeltaX();
        d_delta_y=paige_feature_forward.getDeltaY();
        d_delta_o=paige_feature_forward.getDeltaO();
        float weight{0};
        float delta_x{0},delta_y{0},delta_o{0};
        float pi{3.1415926f};
        delta_x=2.0f/float(d_delta_x);
        delta_y=2.0f/float(d_delta_y);
        delta_o=4.0f*pi/(float)(d_delta_o);

        float normalized_var_x_1{0},normalized_var_x_2{0};
        float normalized_var_y_1{0},normalized_var_y_2{0};
        float normalized_var_o_1{0},normalized_var_o_2{0};
        int var_1_1{0},var_1_2{0};

        for(int r=0;r<HIST_NUMS;++r)
        {
            for(int j=0;j<int(pow(2.0,r));++j)
            {
                for(int dim=0;dim<DESC_DIM;++dim)
                {
                    if(in_1_hist_1[r][j][dim]!=0 && in_1_hist_2[r][j][dim]!=0)
                    {
                        var_1_1=in_1_hist_1[r][j][dim];
                        var_1_2=in_1_hist_2[r][j][dim];

                        normalized_var_x_1=in_x_hist_1[r][j][dim]/float(var_1_1);
                        normalized_var_x_2=in_x_hist_2[r][j][dim]/float(var_1_2);
                        normalized_var_y_1=in_y_hist_1[r][j][dim]/float(var_1_1);
                        normalized_var_y_2=in_y_hist_2[r][j][dim]/float(var_1_2);
                        normalized_var_o_1=in_o_hist_1[r][j][dim]/float(var_1_1);
                        normalized_var_o_2=in_o_hist_2[r][j][dim]/float(var_1_2);

                        Dx=normalized_var_x_1-normalized_var_x_2;
                        Dy=normalized_var_y_1-normalized_var_y_2;
                        Do=normalized_var_o_1-normalized_var_o_2;
                        Dx_minus=-Dx;
                        Dy_minus=-Dy;
                        Do_minus=-Do;

                        Dx_shifted=Dx+1.0f;
                        Dy_shifted=Dy+1.0f;
                        Do_shifted=Do+2.0f*pi;
                        Dx_minus_shifted=Dx_minus+1.0f;
                        Dy_minus_shifted=Dy_minus+1.0f;
                        Do_minus_shifted=Do_minus+2.0f*pi;

                        weight=float(pow(2,r-HIST_NUMS));
                        index_of_dx=Dx_shifted/delta_x;
                        index_of_dy=Dy_shifted/delta_y;
                        index_of_do=Do_shifted/delta_o;
                        index_of_minus_dx=Dx_minus_shifted/delta_x;
                        index_of_minus_dy=Dy_minus_shifted/delta_y;
                        index_of_minus_do=Do_minus_shifted/delta_o;

                        if(index_of_dx==d_delta_x)
                        {
                            index_of_dx-=1;
                        }

                        if(index_of_dy==d_delta_y)
                        {
                            index_of_dy-=1;
                        }

                        if(index_of_do==d_delta_o)
                        {
                            index_of_do-=1;
                        }

                        if(index_of_minus_dx==d_delta_x)
                        {
                            index_of_minus_dx-=1;
                        }

                        if(index_of_minus_dy==d_delta_y)
                        {
                            index_of_minus_dy-=1;
                        }

                        if(index_of_minus_do==d_delta_o)
                        {
                            index_of_minus_do-=1;
                        }

                        paige_feature_forward.addDx(index_of_dx,weight);
                        paige_feature_forward.addDy(index_of_dy,weight);
                        paige_feature_forward.addDo(index_of_do,weight);
                        paige_feature_backward.addDx(index_of_minus_dx,weight);
                        paige_feature_backward.addDy(index_of_minus_dy,weight);
                        paige_feature_backward.addDo(index_of_minus_do,weight);

                        //normalize
                        paige_feature_forward.normalize();
                        paige_feature_backward.normalize();
                    }
                }
            }
        }
    }
}


