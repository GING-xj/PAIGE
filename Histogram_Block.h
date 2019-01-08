//
// Created by GING on 2019-01-08.
//

#ifndef PAIGE_HISTOGRAM_BLOCK_H
#define PAIGE_HISTOGRAM_BLOCK_H

#include "cereal/types/polymorphic.hpp"

#include "PAIGE_Processor.h"

namespace PAIGE
{
    class Histogram_Block {
    public:
        std::string _image_path_string;
        INT_HISTOGRAMS _int_1_hist;
        FLOAT_HISTOGRAMS _float_x_hist;
        FLOAT_HISTOGRAMS _float_y_hist;
        FLOAT_HISTOGRAMS _float_o_hist;

        Histogram_Block()
        {
        }

        Histogram_Block(const std::string & image_path_string,
                const INT_HISTOGRAMS & int_1_hist,
                const FLOAT_HISTOGRAMS & float_x_hist,
                const FLOAT_HISTOGRAMS & float_y_hist,
                const FLOAT_HISTOGRAMS & float_o_hist):_image_path_string(image_path_string),_int_1_hist(int_1_hist),_float_x_hist(float_x_hist),
                _float_y_hist(float_y_hist),_float_o_hist(float_o_hist)
        {
        }

        ~Histogram_Block(){}


        /**
        * Serialization out
        * @param ar Archive
        */
        template <class Archive>
        void save( Archive & ar ) const
        {
                ar(cereal::make_nvp("Path",_image_path_string),
                        cereal::make_nvp("Hist_1",_int_1_hist),
                        cereal::make_nvp("Hist_x",_float_x_hist),
                        cereal::make_nvp("Hist_y",_float_y_hist),
                        cereal::make_nvp("Hist_o",_float_o_hist));
        }

        /**
        * @brief Serialization in
        * @param ar Archive
        */
        template <class Archive>
        void load( Archive & ar )
        {
                ar(cereal::make_nvp("Path",_image_path_string),
                        cereal::make_nvp("Hist_1",_int_1_hist),
                        cereal::make_nvp("Hist_x",_float_x_hist),
                        cereal::make_nvp("Hist_y",_float_y_hist),
                        cereal::make_nvp("Hist_o",_float_o_hist));
        }

    };
}




#endif //PAIGE_HISTOGRAM_BLOCK_H
