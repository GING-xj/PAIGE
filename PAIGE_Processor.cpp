//
// Created by GING on 2019-01-03.
//

#include "cereal/archives/json.hpp"
#include "cereal/types/string.hpp"
#include "cereal/types/map.hpp"

#include <vector>
#include <shark/Data/Dataset.h>
#include <shark/Data/Csv.h>

#include "PAIGE_Processor.h"
#include "openMVG/image/image_io.hpp"
#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"
#include "openMVG/matching/regions_matcher.hpp"
#include "openMVG/multiview/solver_fundamental_kernel.hpp"
#include "openMVG/features/sift/SIFT_Anatomy_Image_Describer.hpp"
#include "openMVG/robust_estimation/robust_estimator_ACRansac.hpp"
#include "openMVG/robust_estimation/robust_estimator_ACRansacKernelAdaptator.hpp"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"



namespace PAIGE
{

    void PAIGE_Processor::calHistograms(
                       openMVG::features::SIFT_Regions * in_sift_regions,
                       INT_HISTOGRAMS & out_1_hist,
                       FLOAT_HISTOGRAMS & out_x_hist,
                       FLOAT_HISTOGRAMS & out_y_hist,
                       FLOAT_HISTOGRAMS & out_o_hist,
                       int width,
                       int height)
    {
        openMVG::features::SIFT_Regions::FeatsT features=in_sift_regions->Features();
        openMVG::features::SIFT_Regions::DescsT descriptors=in_sift_regions->Descriptors();

        std::vector<float> x_vector,y_vector,o_vector;
        x_vector.reserve(features.size());
        y_vector.reserve(features.size());
        o_vector.reserve(features.size());

        for(int i=0;i<features.size();++i)
        {
            x_vector.push_back(features[i].x());
            y_vector.push_back(features[i].y());
            o_vector.push_back(features[i].orientation());
        }

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


    void PAIGE_Processor::calPAIGE_Feature(
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


    int PAIGE_Processor::calHistogramsFromImages(const std::string & img_dir_string)
    {
        if(!stlplus::folder_exists(img_dir_string))
        {
            std::cerr<<"Input directory "<<img_dir_string<<" doesn't exist"<<std::endl;
            return EXIT_FAILURE;
        }

        std::vector<std::string> images_vector=stlplus::folder_files(img_dir_string);
        std::sort(images_vector.begin(),images_vector.end());

        int width{0},height{0};

        INT_HISTOGRAMS int_1_hists;
        FLOAT_HISTOGRAMS float_x_histograms,float_y_histograms,float_o_histograms;


//        std::ofstream os("data.json");
        int count{0};
        for(const auto & img_name:images_vector)
        {
            std::string image_full_path=stlplus::create_filespec(img_dir_string,img_name);

            //check extension name
            if(openMVG::image::GetFormat(image_full_path.c_str()) == openMVG::image::Unknown)
            {
                std::cerr<<image_full_path<<" : Unknown image file format"<<std::endl;
                continue;
            }

            //check if readable
            openMVG::image::ImageHeader imageHeader;
            if(!openMVG::image::ReadImageHeader(image_full_path.c_str(),&imageHeader))
            {
                std::cout<<image_full_path<<" : Image cannot be read"<<std::endl;
                continue;
            }

            width=imageHeader.width;
            height=imageHeader.height;

            std::string image_descTypr_str="SIFT";

            openMVG::image::Image<unsigned char> image;
            ReadImage(image_full_path.c_str(),&image);

            std::unique_ptr<openMVG::features::Image_describer> image_desc_SIFT;
            image_desc_SIFT.reset(new openMVG::features::SIFT_Anatomy_Image_describer(openMVG::features::SIFT_Anatomy_Image_describer::Params()));

            //Calculate SIFT Descriptor
            std::map<openMVG::IndexT ,std::unique_ptr<openMVG::features::Regions>> regions_perImage;
            image_desc_SIFT->Describe(image,regions_perImage[0]);


            openMVG::features::SIFT_Regions * sift_region_unique_ptr= dynamic_cast<openMVG::features::SIFT_Regions *>(regions_perImage[0].get());

            int_1_hists.clear();
            float_x_histograms.clear();
            float_y_histograms.clear();
            float_o_histograms.clear();

            //Calculate Histograms
            calHistograms(sift_region_unique_ptr,int_1_hists,float_x_histograms,float_y_histograms,float_o_histograms,width,height);

            Histogram_Block block(image_full_path,int_1_hists,float_x_histograms,float_y_histograms,float_o_histograms);

            _hist_blocks[count++]=std::make_shared<Histogram_Block>(block);

//            cereal::JSONOutputArchive archive(os);
//            archive(cereal::make_nvp("Path",image_full_path));
//            archive(cereal::make_nvp("Hist_1",int_1_hists));
//            archive(cereal::make_nvp("Hist_x",float_x_histograms));
//            archive(cereal::make_nvp("Hist_y",float_y_histograms));
//            archive(cereal::make_nvp("Hist_o",float_o_histograms));
        }

        std::ofstream os("data.json");
        if(!os.is_open())
        {
            return EXIT_FAILURE;
        }

        {
            cereal::JSONOutputArchive archive(os);
            archive(cereal::make_nvp("Hist_Blocks",_hist_blocks));
        }
        os.close();

        return EXIT_SUCCESS;
    }


    int PAIGE_Processor::calHistogramsFromSfMData(const std::string & path_to_sfm_data_string)
    {

        openMVG::sfm::SfM_Data sfm_data;
        if (!Load(sfm_data, path_to_sfm_data_string, openMVG::sfm::ESfM_Data(openMVG::sfm::VIEWS))) {
            std::cerr << std::endl
                      << "The input file \""<< path_to_sfm_data_string << "\" cannot be read" << std::endl;
            return EXIT_FAILURE;
        }




        return EXIT_SUCCESS;
    }

    //naive implementation
    int PAIGE_Processor::calPAIGE_and_GT_fromJson(const std::string & json_path_string)
    {
        std::unordered_map<int,std::shared_ptr<Histogram_Block>> hist_blocks;

        std::ifstream is(json_path_string);
        if(!is.is_open())
        {
            return EXIT_FAILURE;
        }

        {
            cereal::JSONInputArchive archive(is);
            archive(cereal::make_nvp("Hist_Blocks",hist_blocks));
        }
        is.close();


        for(auto block_iter=hist_blocks.begin();block_iter!=hist_blocks.end();++block_iter)
        {
            auto block_iter_plus_one=block_iter;
            for(++block_iter_plus_one;block_iter_plus_one!=hist_blocks.end();++block_iter_plus_one)
            {
                PAIGE_Feature paige_forward,paige_backward;
                calPAIGE_Feature(block_iter->second->_int_1_hist,block_iter->second->_float_x_hist,block_iter->second->_float_y_hist,
                        block_iter->second->_float_o_hist,block_iter_plus_one->second->_int_1_hist,block_iter_plus_one->second->_float_x_hist,
                                 block_iter_plus_one->second->_float_y_hist,block_iter_plus_one->second->_float_o_hist,
                                 paige_forward,paige_backward);

                std::string filenameL,filenameR;
                filenameL=block_iter->second->_image_path_string;
                filenameR=block_iter_plus_one->second->_image_path_string;

                std::vector<shark::RealVector> inputs;
                std::vector<unsigned int> labels;

                shark::RealVector v1(paige_forward.getSize()),v2(paige_forward.getSize());
                PAIGE::PAIGE_Feature::PAIGE_DX dX_f=paige_forward.getDx(),dX_b=paige_backward.getDx();
                PAIGE::PAIGE_Feature::PAIGE_DY dY_f=paige_forward.getDy(),dY_b=paige_backward.getDy();
                PAIGE::PAIGE_Feature::PAIGE_DO dO_f=paige_forward.getDo(),dO_b=paige_backward.getDo();

                for(int i=0;i<paige_forward.getDeltaX();++i)
                {
                    v1(i)=dX_f(i);
                    v2(i)=dX_b(i);
                }

                for(int i=v1.size();i<v1.size()+paige_forward.getDeltaY();++i)
                {
                    v1(i)=dY_f(i);
                    v2(i)=dY_b(i);
                }

                for(int i=v1.size();i<v1.size()+paige_forward.getDeltaO();++i)
                {
                    v1(i)=dO_f(i);
                    v2(i)=dO_b(i);
                }

                inputs.push_back(v1);
                inputs.push_back(v2);

                if(calGroundTruthLabel(filenameL,filenameR))
                {
                    labels.push_back(1);
                    labels.push_back(1);
                }
                else
                {
                    labels.push_back(0);
                    labels.push_back(0);
                }


                shark::ClassificationDataset data=shark::createLabeledDataFromRange(inputs,labels);
                std::ofstream os("data.csv",std::ios::app);
                shark::detail::exportCSV_labeled(inputs,labels,os,shark::FIRST_COLUMN,',');
            }
        }

        return EXIT_SUCCESS;
    }

    bool PAIGE_Processor::calGroundTruthLabel(const std::string & imageNameL,const std::string & imageNameR)
    {
        openMVG::image::Image<unsigned char> imageL, imageR;
        openMVG::image::ReadImage(imageNameL.c_str(), &imageL);
        openMVG::image::ReadImage(imageNameR.c_str(), &imageR);

        //--
        // Detect regions thanks to an image_describer
        //--
        using namespace openMVG::features;
        std::unique_ptr<Image_describer> image_describer(new SIFT_Anatomy_Image_describer);
        std::map<openMVG::IndexT, std::unique_ptr<openMVG::features::Regions>> regions_perImage;
        image_describer->Describe(imageL, regions_perImage[0]);
        image_describer->Describe(imageR, regions_perImage[1]);

        const SIFT_Regions* regionsL = dynamic_cast<SIFT_Regions*>(regions_perImage.at(0).get());
        const SIFT_Regions* regionsR = dynamic_cast<SIFT_Regions*>(regions_perImage.at(1).get());

        const PointFeatures
                featsL = regions_perImage.at(0)->GetRegionsPositions(),
                featsR = regions_perImage.at(1)->GetRegionsPositions();


        std::vector<openMVG::matching::IndMatch> vec_PutativeMatches;
        //-- Perform matching -> find Nearest neighbor, filtered with Distance ratio
        {
            // Find corresponding points
            openMVG::matching::DistanceRatioMatch(
                    0.8, openMVG::matching::BRUTE_FORCE_L2,
                    *regions_perImage.at(0).get(),
                    *regions_perImage.at(1).get(),
                    vec_PutativeMatches);

        }

        // Fundamental geometry filtering of putative matches
        {
            //A. get back interest point and send it to the robust estimation framework
            openMVG::Mat xL(2, vec_PutativeMatches.size());
            openMVG::Mat xR(2, vec_PutativeMatches.size());

            for (size_t k = 0; k < vec_PutativeMatches.size(); ++k)  {
                const PointFeature & imaL = featsL[vec_PutativeMatches[k].i_];
                const PointFeature & imaR = featsR[vec_PutativeMatches[k].j_];
                xL.col(k) = imaL.coords().cast<double>();
                xR.col(k) = imaR.coords().cast<double>();
            }

            //-- Fundamental robust estimation
            std::vector<uint32_t> vec_inliers;
            using KernelType =
            openMVG::robust::ACKernelAdaptor<
                    openMVG::fundamental::kernel::SevenPointSolver,
                    openMVG::fundamental::kernel::SymmetricEpipolarDistanceError,
                    openMVG::UnnormalizerT,
                    openMVG::Mat3>;

            KernelType kernel(
                    xL, imageL.Width(), imageL.Height(),
                    xR, imageR.Width(), imageR.Height(),
                    true); // configure as point to line error model.

            openMVG::Mat3 F;
            const std::pair<double,double> ACRansacOut = ACRANSAC(kernel, vec_inliers, 1024, &F,
                                                                  openMVG::Square(4.0), // Upper bound of authorized threshold
                                                                  true);
            const double & thresholdF = ACRansacOut.first;

            // Check the fundamental support some point to be considered as valid
            if (vec_inliers.size() > KernelType::MINIMUM_SAMPLES *2.5) {

                std::cout << "\nFound a fundamental under the confidence threshold of: "
                          << thresholdF << " pixels\n\twith: " << vec_inliers.size() << " inliers"
                          << " from: " << vec_PutativeMatches.size()
                          << " putatives correspondences"
                          << std::endl;

                //compute residuals
                std::vector<double> vec_residuals(vec_inliers.size(), 0.0);
                for ( size_t i = 0; i < vec_inliers.size(); ++i)  {
                    const SIOPointFeature & LL = regionsL->Features()[vec_PutativeMatches[vec_inliers[i]].i_];
                    const SIOPointFeature & RR = regionsR->Features()[vec_PutativeMatches[vec_inliers[i]].j_];
                    // residual computation
                    vec_residuals[i] = std::sqrt(KernelType::ErrorT::Error(F,
                                                                           LL.coords().cast<double>(),
                                                                           RR.coords().cast<double>()));
                }

                // Display some statistics of reprojection errors
                float dMin, dMax, dMean, dMedian;
                openMVG::minMaxMeanMedian<float>(vec_residuals.cbegin(), vec_residuals.cend(),
                                        dMin, dMax, dMean, dMedian);

                std::cout << std::endl
                          << "Fundamental matrix estimation, residuals statistics:" << "\n"
                          << "\t-- Residual min:\t" << dMin << std::endl
                          << "\t-- Residual median:\t" << dMedian << std::endl
                          << "\t-- Residual max:\t "  << dMax << std::endl
                          << "\t-- Residual mean:\t " << dMean << std::endl;
            }
            else  {
                std::cout << "ACRANSAC was unable to estimate a rigid fundamental"
                          << std::endl;
                return false;
            }
        }

        return true;

    }


}





