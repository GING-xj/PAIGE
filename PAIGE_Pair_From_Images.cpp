//
// Created by xjxj on 19-1-16.
//
// Calculate possible geometry-verified image pairs with trained model

#include "cereal/archives/json.hpp"
#include "cereal/types/string.hpp"
#include "cereal/types/map.hpp"

#include <iostream>
#include <fstream>
#include <string>
#include <shark/Data/Dataset.h>
#include <shark/Data/Csv.h>

#include "PAIGE_Feature.h"
#include "Histogram_Block.h"
#include "PAIGE_Processor.h"

#include <shark/Algorithms/Trainers/RFTrainer.h>
#include "openMVG/third_party/stlplus3/filesystemSimplified/file_system.hpp"


int main(int argc,char **argv)
{
    if(argc!=3)
    {
        std::cout<<"Usage "<<std::endl;
        std::cout<<"*:~/Path/To/Program   Dir/To/Images  Path/To/Model "<<std::endl;
        std::cout<<"e.g. :~/home/PAIGE_Pair_From_Images  /home/Images/ /home/trained.model"<<std::endl;
        return EXIT_FAILURE;
    }

    std::string img_dir_string=argv[1];
    std::string folder_current_full_string=stlplus::folder_current_full();
    std::string folder_Pair_output_string=stlplus::create_filespec(folder_current_full_string,"PAIGE_pair");

    //Create folder to save PAIGE pair
    if(!stlplus::folder_create(folder_Pair_output_string))
    {
        std::cout<<"Cannot Create folder PAIGE_pair"<<std::endl;
        return EXIT_FAILURE;
    }

    std::string model_name=argv[2];
    if(!stlplus::file_exists(model_name))
    {
        std::cout<<"Cannot find input model"<<std::endl;
        return EXIT_FAILURE;
    }

    //Load model
    shark::RFClassifier model;
    std::ifstream ifs(model_name);
    boost::archive::text_iarchive ia(ifs);
    model.read(ia);
    ifs.close();

    //Calculate histograms
    PAIGE::PAIGE_Processor processor;
    processor.calHistogramsFromImages(img_dir_string);

    std::cout<<"******"<<std::endl;
    std::cout<<"Listing Hist Files"<<std::endl;
    std::cout<<std::endl;


    std::vector<std::string> folder_files_vec=stlplus::folder_files(folder_current_full_string);
    std::vector<std::string> hist_files_vec;

    //Collecting json files
    for(auto iter=folder_files_vec.begin();iter!=folder_files_vec.end();++iter)
    {
        std::string extname=stlplus::extension_part(*iter);
        if(extname!="json")
            continue;

        hist_files_vec.push_back(*iter);
    }


    //Calculate PAIGE features and classify them
    std::string hist_fileL,hist_fileR;
    unsigned int label;
    for(auto hist_iter=hist_files_vec.begin();hist_iter!=hist_files_vec.end();++hist_iter)
    {
        auto hist_iter_plus_one=hist_iter;
        for(++hist_iter_plus_one;hist_iter_plus_one!=hist_files_vec.end();++hist_iter_plus_one)
        {
            hist_fileL=stlplus::create_filespec(stlplus::folder_current_full(),*hist_iter);
            hist_fileR=stlplus::create_filespec(stlplus::folder_current_full(),*hist_iter_plus_one);

            std::ifstream isL(hist_fileL);
            if(!isL.is_open())
            {
                std::cerr<<"Cannot open "<<hist_fileL<<std::endl;
                return EXIT_FAILURE;
            }

            std::ifstream isR(hist_fileR);
            if(!isR.is_open())
            {
                std::cerr<<"Cannot open "<<hist_fileR<<std::endl;
                return EXIT_FAILURE;
            }

            std::cout<<"Loading "<<*hist_iter<<std::endl;
            PAIGE::Histogram_Block hist_blockL;
            {
                cereal::JSONInputArchive archive(isL);
                archive(cereal::make_nvp("Hist_Block",hist_blockL));
            }
            isL.close();

            std::cout<<"Loading "<<*hist_iter_plus_one<<std::endl;
            PAIGE::Histogram_Block hist_blockR;
            {
                cereal::JSONInputArchive archive(isR);
                archive(cereal::make_nvp("Hist_Block",hist_blockR));
            }
            isR.close();


            std::string filenameL,filenameR;
            filenameL=hist_blockL._image_path_string;
            filenameR=hist_blockR._image_path_string;

            std::cout<<"Current image pair:"<<std::endl;
            std::cout<<filenameL<<std::endl;
            std::cout<<filenameR<<std::endl<<std::endl;


            std::cout<<"Calculating PAIGE feature"<<std::endl;

            PAIGE::PAIGE_Feature paige_forward,paige_backward;
            processor.calPAIGE_Feature(
                    hist_blockL._int_1_hist,
                    hist_blockL._float_x_hist,
                    hist_blockL._float_y_hist,
                    hist_blockL._float_o_hist,
                    hist_blockR._int_1_hist,
                    hist_blockR._float_x_hist,
                    hist_blockR._float_y_hist,
                    hist_blockR._float_o_hist,
                    paige_forward,
                    paige_backward);


            //Classify PAIGE feature
            shark::RealVector v1(paige_forward.getSize()),v2(paige_forward.getSize());
            Eigen::VectorXf final_feat_forward=paige_forward.getPAIGE(),final_feat_backward=paige_backward.getPAIGE();

            for(int i=0;i<paige_forward.getSize();++i)
            {
                v1(i)=final_feat_forward(i);
                v2(i)=final_feat_backward(i);
            }

            shark::RealVector prediction_for_forward,prediction_for_backward;
            prediction_for_forward=model(v1);
            prediction_for_backward=model(v2);

            //Determine the label of the current pair
            if(prediction_for_forward(0)>prediction_for_forward(1))
            {
                if(prediction_for_backward(0)>prediction_for_backward(1))
                {
                    label=0;
                }
                else
                {
                    if(prediction_for_forward(0)>prediction_for_backward(1))
                    {
                        label=0;
                    }
                    else
                    {
                        label=1;
                    }
                }
            }
            else
            {
                if(prediction_for_backward(0)<=prediction_for_backward(1))
                {
                    label=1;
                }
                else
                {
                    if(prediction_for_forward(1)>prediction_for_backward(0))
                    {
                        label=1;
                    }
                    else
                    {
                        label=0;
                    }
                }
            }

            if(label==1)
            {
                std::ofstream os("PAIGE_pair_from_images.txt",std::ios::app);
                os<<filenameL<<" "<<filenameR<<"\n";
                os.close();
            }

        }
    }

    return EXIT_SUCCESS;
}



