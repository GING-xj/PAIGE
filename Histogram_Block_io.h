//
// Created by xjxj on 19-1-9.
//

#ifndef PAIGE_HISTOGRAM_BLOCK_IO_H
#define PAIGE_HISTOGRAM_BLOCK_IO_H


#include "Histogram_Block.h"
#include "cereal/types/polymorphic.hpp"

/**
* Serialization out
* @param ar Archive
*/
template <class Archive>
void PAIGE::Histogram_Block::Histogram_Block::save( Archive & ar ) const
{
    ar(cereal::make_nvp("Path",_image_path_string),
       cereal::make_nvp("Id_view",_id_view),
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
void PAIGE::Histogram_Block::load( Archive & ar )
{
    ar(cereal::make_nvp("Path",_image_path_string),
       cereal::make_nvp("Id_view",_id_view),
       cereal::make_nvp("Hist_1",_int_1_hist),
       cereal::make_nvp("Hist_x",_float_x_hist),
       cereal::make_nvp("Hist_y",_float_y_hist),
       cereal::make_nvp("Hist_o",_float_o_hist));
}


#endif //PAIGE_HISTOGRAM_BLOCK_IO_H
