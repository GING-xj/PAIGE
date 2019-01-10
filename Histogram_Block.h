//
// Created by GING on 2019-01-08.
//



#ifndef PAIGE_HISTOGRAM_BLOCK_H
#define PAIGE_HISTOGRAM_BLOCK_H



#include <map>


namespace PAIGE
{
    class Histogram_Block {
    public:

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

        std::string _image_path_string;
        int _id_view;
        INT_HISTOGRAMS _int_1_hist;
        FLOAT_HISTOGRAMS _float_x_hist;
        FLOAT_HISTOGRAMS _float_y_hist;
        FLOAT_HISTOGRAMS _float_o_hist;

        Histogram_Block():_id_view(-1)
        {
        }

        Histogram_Block(const std::string & image_path_string,
                int id_view,
                const INT_HISTOGRAMS & int_1_hist,
                const FLOAT_HISTOGRAMS & float_x_hist,
                const FLOAT_HISTOGRAMS & float_y_hist,
                const FLOAT_HISTOGRAMS & float_o_hist):_image_path_string(image_path_string),_id_view(id_view),_int_1_hist(int_1_hist),_float_x_hist(float_x_hist),
                _float_y_hist(float_y_hist),_float_o_hist(float_o_hist)
        {
        }

        ~Histogram_Block(){}


        /**
        * Serialization out
        * @param ar Archive
        */
        template <class Archive>
        void save( Archive & ar ) const;

        /**
        * @brief Serialization in
        * @param ar Archive
        */
        template <class Archive>
        void load( Archive & ar );

    };
}




#endif //PAIGE_HISTOGRAM_BLOCK_H
