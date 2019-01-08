//
// Created by GING on 2018-12-31.
//

#include <map>

#ifndef PAIGE_DEFINITION_H
#define PAIGE_DEFINITION_H

using INT_BIN=std::map< int, int>;

using FLOAT_BIN=std::map< int, float>;

using INT_HISTOGRAM=std::map< int,INT_BIN >;

using FLOAT_HISTOGRAM=std::map< int,FLOAT_BIN >;

using INT_HISTOGRAMS=std::map< int,INT_HISTOGRAM >;

using FLOAT_HISTOGRAMS=std::map< int,FLOAT_HISTOGRAM >;

using PAIGE_DX=std::map< int, int>;

using PAIGE_DY=std::map< int, int>;

using PAIGE_DO=std::map< int, int>;

const int HIST_NUMS=9;

const int DESC_DIM=128;

#endif //PAIGE_DEFINITION_H
