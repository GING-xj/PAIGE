#include <iostream>
#include <string>

#include "PAIGE_Feature.h"
#include "Histogram_Block.h"
#include "PAIGE_Processor.h"

#include "openMVG/third_party/stlplus3/filesystemSimplified/file_system.hpp"



int main(int argc,char **argv) {



    if(argc!=2)
    {
        std::cout<<"Usage "<<std::endl;
        std::cout<<"*:~/Path/To/Program Dir/To/Images"<<std::endl;
        return EXIT_FAILURE;
    }

    std::string image_dir_string;
    image_dir_string=argv[1];

    PAIGE::PAIGE_Processor processor;
    processor.calHistogramsFromImages(image_dir_string);



    std::string current_dir_string=stlplus::folder_current_full();
    processor.calPAIGE_and_GT_fromJson(current_dir_string);



//
//    PAIGE::PAIGE_Processor processor;
//    std::string img_dir_string="/home/xjxj/data/NotreDame/test";
//    processor.calHistogramsFromImages(img_dir_string);
//    std::string json_path_string="/home/xjxj/data/NotreDame/PAIGE_output_0/histograms_data.json";
//    processor.calPAIGE_and_GT_fromJson(json_path_string);



//    std::string dir_to="/home/xjxj/SfM_output/i23d_test/reconstruction_sequential";
//    std::string path_to=stlplus::create_filespec(dir_to,"sfm_data.json");
//    std::cout<<"path:"<<path_to<<std::endl;
//
//    if(stlplus::file_exists(path_to))
//    {
//        std::cout<<"exist\n";
//    }
//    else
//    {
//        std::cout<<"doesn't exist\n";
//    }
//
//    std::unordered_map<int,std::shared_ptr<PAIGE::Histogram_Block>> blocks;
//
//    int count{0};
//
//    PAIGE::PAIGE_Processor::INT_HISTOGRAMS int_h1;
//    PAIGE::PAIGE_Processor::FLOAT_HISTOGRAMS float_h1_x,float_h1_y,float_h1_o;
//    int_h1[1][1][1]=1,int_h1[2][2][2]=2;
//    float_h1_x[3][3][3]=3,float_h1_y[4][4][4]=4,float_h1_o[5][5][5]=5,float_h1_o[6][6][6]=12;
//    PAIGE::Histogram_Block block1("/home/ss",int_h1,float_h1_x,float_h1_y,float_h1_o);
//
//    blocks[count++]=std::make_shared<PAIGE::Histogram_Block>(block1);
//
//    PAIGE::PAIGE_Processor::INT_HISTOGRAMS int_h2;
//    PAIGE::PAIGE_Processor::FLOAT_HISTOGRAMS float_h2_x,float_h2_y,float_h2_o;
//    int_h2[1][1][1]=2,int_h2[2][2][2]=4;
//    float_h2_x[3][3][3]=6,float_h2_y[4][4][4]=8,float_h2_o[5][5][5]=10,float_h2_o[6][6][6]=12;
//    PAIGE::Histogram_Block block2("/home/ss",int_h2,float_h2_x,float_h2_y,float_h2_o);
//
//    blocks[count++]=std::make_shared<PAIGE::Histogram_Block>(block2);
//
//
//    PAIGE::PAIGE_Processor::INT_HISTOGRAMS int_h3;
//    PAIGE::PAIGE_Processor::FLOAT_HISTOGRAMS float_h3_x,float_h3_y,float_h3_o;
//    int_h3[1][1][1]=3,int_h3[2][2][2]=6;
//    float_h3_x[3][3][3]=9,float_h3_y[4][4][4]=12,float_h3_o[5][5][5]=15,float_h3_o[6][6][6]=18;
//    PAIGE::Histogram_Block block3("/home/ss",int_h3,float_h3_x,float_h3_y,float_h3_o);
//
//    blocks[count++]=std::make_shared<PAIGE::Histogram_Block>(block3);
//
//    std::ofstream os("data.json");
//    {
//        cereal::JSONOutputArchive archive(os);
//        archive(cereal::make_nvp("Blocks",blocks));
//    }
//    os.close();
//
//    std::unordered_map<int,std::shared_ptr<PAIGE::Histogram_Block>> in_blocks;
//    std::ifstream is("data.json");
//    {
//        cereal::JSONInputArchive archive(is);
//        archive(cereal::make_nvp("Blocks",in_blocks));
//    }
//    is.close();
//
//    for(const auto it:in_blocks)
//    {
//        std::cout<<it.first<<":"<<it.second->_image_path_string<<std::endl;
//        std::cout<<"int: "<<it.second->_int_1_hist[1][1][1]<<" x: "<<it.second->_float_x_hist[3][3][3]
//        <<" y: "<<it.second->_float_y_hist[4][4][4]<<" o: "<<it.second->_float_o_hist[5][5][5]<<std::endl;
//    }
//
//    for(auto it=in_blocks.begin();it!=in_blocks.end();++it)
//    {
//        auto it1=it;
//        for(++it1;it1!=in_blocks.end();++it1) {
//            std::cout << it1->first << ":" << it1->second->_image_path_string << std::endl;
//            std::cout << "int: " << it1->second->_int_1_hist[1][1][1] << " x: " << it1->second->_float_x_hist[3][3][3]
//                      << " y: " << it1->second->_float_y_hist[4][4][4] << " o: " << it1->second->_float_o_hist[5][5][5]
//                      << std::endl;
//        }
//
//    }
////
//    std::string path="/users/xujun/CLionProjects/Incremental_SfM/Images";
//    std::vector<std::string> imgs_vector=stlplus::folder_files(path);
//    std::sort(imgs_vector.begin(),imgs_vector.end());
//
//    for(const auto & img_name:imgs_vector)
//    {
//        std::cout<<img_name<<std::endl;
//    }
//
//    for(const auto & img_name:imgs_vector)
//    {
//        std::string full_path=stlplus::create_filespec(path,img_name);
//        std::cout<<full_path<<std::endl;
//    }
//
//
//    shark::RealVector v(3);
//    v(0)=1.0,v(1)=2.5,v(2)=7.123451211312321;
//    shark::RealVector v1(3);
//    v1(0)=3.3,v1(1)=3.4,v1(2)=-7.9;
//    shark::RealVector v2(3);
//    v2(0)=2.2,v2(1)=3.3,v2(2)=3.4;
//    shark::RealVector v3(3);
//    v3(0)=7.9,v3(1)=8.8,v3(2)=99.9;
//
//
//    std::vector<shark::RealVector> inputs;
//    std::vector<unsigned int> labels;
//
//
//
//    inputs.push_back(v);
//    inputs.push_back(v1);
//    inputs.push_back(v2);
//    inputs.push_back(v3);
//
//    labels.push_back(0);
//    labels.push_back(1);
//    labels.push_back(1);
//    labels.push_back(0);
//
//    shark::ClassificationDataset data=shark::createLabeledDataFromRange(inputs,labels);
//    shark::exportCSV(data,"data.csv",shark::FIRST_COLUMN);
//
//    std::ofstream os("data.csv",std::ios::app);
//    shark::detail::exportCSV_labeled(inputs,labels,os,shark::FIRST_COLUMN,',');
//    shark::detail::exportCSV_labeled(inputs,labels,os,shark::FIRST_COLUMN,',');
//    shark::detail::exportCSV_labeled(inputs,labels,os,shark::FIRST_COLUMN,',');
//
//    shark::ClassificationDataset data_in;
//    shark::importCSV(data_in,"data.csv",shark::FIRST_COLUMN);
//
//
//    for(const auto & batch:data_in.inputs().batches())
//    {
//        std::cout<<batch<<std::endl;
//    }
//
//    for(const auto & batch:data_in.labels().batches())
//    {
//        std::cout<<batch<<std::endl;
//    }

//
//    shark::ClassificationDataset data_in;
//    shark::importCSV(data_in,"data.csv",shark::FIRST_COLUMN);
//
//
//    for(const auto & batch:data_in.inputs().batches())
//    {
//        std::cout<<batch<<std::endl;
//    }
//
//    for(const auto & batch:data_in.labels().batches())
//    {
//        std::cout<<batch<<std::endl;
//    }
//
//    {


//        std::ofstream os("data.json",std::ios::binary | std::ios::out);
//        for(int i=0;i<5;++i)
//        {
//            cereal::JSONOutputArchive archive(os);
//            std::stringstream stream;
//            stream<<i;
//            std::string str="/home/xjxj/"+stream.str();
//            std::map<int, std::map<int, std::map<int, int>>> m;
//            m[1][1][1]=2*i;
//            m[2][2][2]=4*i;
//            m[3][3][3]=6*i;
//            m[4][4][4]=8*i;
//
//            archive(cereal::make_nvp("Path",str));
//            archive(cereal::make_nvp("Histogram",m));
//        }
//        os.close();
//
//
//
//        std::ifstream is;
//        std::map<int,std::map<int,std::map<int,int>>> m={};
//        std::string str="";
//
//        is.open("data.json",std::ios::binary | std::ios::in);
//
//        for(int i=0;i<5;++i)
//        {
//            cereal::JSONInputArchive   archive(is);
//            archive("Path",str);
//            archive("Histogram",m);
//            //archive(str,m);
//            archive(cereal::make_nvp("Path",str));
//            archive(cereal::make_nvp("Histogram",m));
//            std::cout<<str<<std::endl;
//        }
//        is.close();


//        std::string imgpath = "/home/xjxj/daaaaaaa.jpg";
//
//        std::map<int, std::map<int, std::map<int, int>>> m;
//        m[0][0][0] = 19;
//        m[0][1][2] = 31;
//        m[1][3][5] = 72;
//        m[1][18][8] = 33;
//        m[13][2][7] = 17;
//
//
//        archive(cereal::make_nvp("Path", imgpath));
//        archive(cereal::make_nvp("Histogram", m));
//
//        std::string imgpath1="/hone/saj/as.jpg";
//        std::map<int, std::map<int, std::map<int, int>>> m1;
//        m1[1][1][1]=1;
//        m1[2][2][2]=2;
//        m1[3][3][3]=3;
//        m1[4][4][4]=4;
//
//        archive(cereal::make_nvp("Path",imgpath1));
//        archive(cereal::make_nvp("Histogram",m1));
//    }
//
//
//    {
//        std::ifstream is;
//        std::map<int,std::map<int,std::map<int,int>>> m={};
//        std::string str="";
//
//        is.open("data.json",std::ios::in);
//        cereal::JSONInputArchive   archive(is);
//        archive(str,m);
//
//        std::cout<<str<<std::endl;
//
//        using std::cout;
//        using std::endl;
//
//        cout<<"Non zero:\n";
//        cout<<m[0][0][0]<<endl;
//        cout<<m[0][1][2]<<endl;
//        cout<<m[1][3][5]<<endl;
//        cout<<m[1][18][8]<<endl;
//        cout<<m[13][2][7]<<endl;
//
//
//        cout<<"Zero:\n";
//        cout<<m[0][0][1]<<endl;
//        cout<<m[0][0][2]<<endl;
//        cout<<m[1][1][1]<<endl;
//        cout<<m[1][18][7]<<endl;
//        cout<<m[1][18][9]<<endl;
//        cout<<m[1][17][8]<<endl;
//        cout<<m[13][2][6]<<endl;
//        cout<<m[13][2][8]<<endl;
//        cout<<m[13][1][7]<<endl;
//        cout<<m[12][2][7]<<endl;
//    }





//    INT_HISTOGRAMS int_hist_1,int_hist_2;
//    FLOAT_HISTOGRAMS hist_x_1,hist_x_2,hist_y_1,hist_y_2,hist_o_1,hist_o_2;
//
//    int_hist_1[0][0][0]=3;
//    int_hist_1[0][0][1]=3;
//    int_hist_1[0][0][2]=3;
//    int_hist_1[1][0][0]=1;
//    int_hist_1[1][0][2]=2;
//    int_hist_1[1][1][0]=2;
//    int_hist_1[1][1][1]=3;
//    int_hist_1[1][1][2]=1;
//    int_hist_1.clear();
//
//    using std::cout;
//    using std::endl;
//
//    cout<<int_hist_1[0][0][0]<<endl
//    <<int_hist_1[0][0][1]<<endl
//    <<int_hist_1[0][0][2]<<endl
//    <<int_hist_1[1][0][0]<<endl
//    <<int_hist_1[1][0][2]<<endl
//    <<int_hist_1[1][1][0]<<endl
//    <<int_hist_1[1][1][1]<<endl
//    <<int_hist_1[1][1][2]<<endl;

//
//    hist_x_1[0][0][0]=0.3;
//    hist_x_1[0][0][1]=0.6;
//    hist_x_1[0][0][2]=0.9;
//    hist_x_1[1][0][0]=0.1;
//    hist_x_1[1][0][2]=0.6;
//    hist_x_1[1][1][0]=0.2;
//    hist_x_1[1][1][1]=0.6;
//    hist_x_1[1][1][2]=0.3;
//
//
//    hist_y_1[0][0][0]=0.3;
//    hist_y_1[0][0][1]=0.6;
//    hist_y_1[0][0][2]=0.9;
//    hist_y_1[1][0][0]=0.1;
//    hist_y_1[1][0][2]=0.6;
//    hist_y_1[1][1][0]=0.2;
//    hist_y_1[1][1][1]=0.6;
//    hist_y_1[1][1][2]=0.3;
//
//
//    hist_o_1[0][0][0]=0.3;
//    hist_o_1[0][0][1]=0.6;
//    hist_o_1[0][0][2]=0.9;
//    hist_o_1[1][0][0]=0.1;
//    hist_o_1[1][0][2]=0.6;
//    hist_o_1[1][1][0]=0.2;
//    hist_o_1[1][1][1]=0.6;
//    hist_o_1[1][1][2]=0.3;
//
//
//
//    int_hist_2[0][0][0]=4;
//    int_hist_2[0][0][1]=4;
//    int_hist_2[0][0][2]=4;
//    int_hist_2[1][0][0]=2;
//    int_hist_2[1][0][1]=1;
//    int_hist_2[1][0][2]=4;
//    int_hist_2[1][1][0]=2;
//    int_hist_2[1][1][1]=3;
//
//    hist_x_2[0][0][0]=0.4;
//    hist_x_2[0][0][1]=0.8;
//    hist_x_2[0][0][2]=1.2;
//    hist_x_2[1][0][0]=0.2;
//    hist_x_2[1][0][1]=0.2;
//    hist_x_2[1][0][2]=1.2;
//    hist_x_2[1][1][0]=0.2;
//    hist_x_2[1][1][1]=0.6;
//
//
//    hist_y_2[0][0][0]=0.4;
//    hist_y_2[0][0][1]=0.8;
//    hist_y_2[0][0][2]=1.2;
//    hist_y_2[1][0][0]=0.2;
//    hist_y_2[1][0][1]=0.2;
//    hist_y_2[1][0][2]=1.2;
//    hist_y_2[1][1][0]=0.2;
//    hist_y_2[1][1][1]=0.6;
//
//
//    hist_o_2[0][0][0]=0.4;
//    hist_o_2[0][0][1]=0.8;
//    hist_o_2[0][0][2]=1.2;
//    hist_o_2[1][0][0]=0.2;
//    hist_o_2[1][0][1]=0.2;
//    hist_o_2[1][0][2]=1.2;
//    hist_o_2[1][1][0]=0.2;
//    hist_o_2[1][1][1]=0.6;
//
//    PAIGE::PAIGE_Feature feature_forward,feature_backward;
//
//    steady_clock::time_point t1=steady_clock::now();
//
//    PAIGE_test test;
//    test.calPAIGE_Feature(int_hist_1,hist_x_1,hist_y_1,hist_o_1,int_hist_2,hist_x_2,hist_y_2,hist_o_2,feature_forward,feature_backward);
//
//    steady_clock::time_point t2=steady_clock::now();
//    duration<double> time_span=duration_cast<duration<double>>(t2-t1);
//    std::cout<<"duration:"<<time_span.count()<<std::endl;

//    int width{500},height{1024};
//    std::vector<float> x_vector,y_vector,o_vector;
//    float pii=3.1415926;
//    x_vector.push_back(323);
//    x_vector.push_back(147);
//    x_vector.push_back(246);
//    y_vector.push_back(524);
//    y_vector.push_back(983);
//    y_vector.push_back(247);
//    o_vector.push_back(0.5f*pii);
//    o_vector.push_back(0.7f*pii);
//    o_vector.push_back(0.02f*pii);
//
//    std::vector<Eigen::VectorXf> descs;
//    Eigen::VectorXf desc1,desc2,desc3;
//    desc1.resize(5),desc2.resize(5),desc3.resize(5);
//    desc1(0)=2,desc2(0)=10,desc3(0)=7;
//    desc1(1)=3,desc2(1)=2,desc3(1)=7;
//    desc1(2)=0,desc2(2)=9,desc3(2)=0;
//    desc1(3)=7,desc2(3)=9,desc3(3)=0;
//    desc1(4)=1,desc2(4)=0,desc3(4)=1;
//    descs.push_back(desc1);
//    descs.push_back(desc2);
//    descs.push_back(desc3);
//
//    INT_HISTOGRAMS hist_1;
//    FLOAT_HISTOGRAMS hist_x,hist_y,hist_o;
//
//    PAIGE_TEST::PAIGE_test test;
//
//    steady_clock::time_point t1=steady_clock::now();
//
//    test.calHistograms(x_vector,y_vector,o_vector,descs,hist_1,hist_x,hist_y,hist_o,width,height);
//
//    steady_clock::time_point t2=steady_clock::now();
//    duration<double> time_span=duration_cast<duration<double>>(t2-t1);
//    std::cout<<"duration:"<<time_span.count()<<std::endl;

//    using namespace std::chrono;
//    const int row=2500;
//    const int col=128;
//
//    double a1[row][col],a2[row][col],a3[row][col];
//
//    srand((int)time(0));
//
//    for(int i=0;i<row;++i)
//    {
//        for(int j=0;j<col;++j)
//        {
//            a1[i][j]=rand()%10000;
//            a2[i][j]=rand()%10000;
//        }
//    }
//
//
//    steady_clock::time_point t1=steady_clock::now();
//
//
//    for(int i=0;i<row;++i)
//    {
//        for(int j=0;j<col;++j)
//        {
//            a3[i][j]=a1[i][j]+a2[i][j];
//        }
//    }
//
//    steady_clock::time_point t2=steady_clock::now();
//
//    duration<double> time_span=duration_cast<duration<double>>(t2-t1);
//    std::cout<<"It takes "<<time_span.count()<<" seconds to sum all in for loop\n";
//
//    std::cout<<a3[0][0]<<" "<<a3[0][1]<<std::endl;
//
//
////
//    std::string image_descTypr_str="SIFT";
//    std::string path_to_image_str="/users/xujun/CLionProjects/Incremental_SfM/Images/0.jpg";
//
//    Image<unsigned char> image;
//    ReadImage(path_to_image_str.c_str(),&image);
//
//    std::unique_ptr<Image_describer> image_desc_SIFT;
//    image_desc_SIFT.reset(new SIFT_Anatomy_Image_describer(SIFT_Anatomy_Image_describer::Params()));
//
//    std::map<openMVG::IndexT ,std::unique_ptr<Regions>> regions_perImage;
//    image_desc_SIFT->Describe(image,regions_perImage[0]);
//
//    SIFT_Regions * region_unique_ptr= dynamic_cast<SIFT_Regions *>(regions_perImage[0].get());
//    SIFT_Regions::FeatsT features=region_unique_ptr->Features();
//    SIFT_Regions::DescsT descs=region_unique_ptr->Descriptors();
//


//    float x_min{10000},x_max{-1};
//    float y_min{10000},y_max{-1};
//    for(const auto feature:features)
//    {
//        if(feature.x()<x_min)
//        {
//            x_min=feature.x();
//        }
//        if(feature.x()>x_max)
//        {
//            x_max=feature.x();
//        }
//        if(feature.y()<y_min)
//        {
//            y_min=feature.y();
//        }
//        if(feature.y()>y_max)
//        {
//            y_max=feature.y();
//        }
//    }
//    std::cout<<"X Min:"<<x_min<<std::endl;
//    std::cout<<"X Max:"<<x_max<<std::endl;
//    std::cout<<"Y Min:"<<y_min<<std::endl;
//    std::cout<<"Y Max:"<<y_max<<std::endl;
//
//    SIFT_Regions::DescriptorT desc_single=descs[1];
//    std::cout<<desc_single(0,0)<<std::endl;
//    std::cout<<desc_single(1,0)<<std::endl;
//    std::cout<<desc_single(2,0)<<std::endl;
//    std::cout<<desc_single.rows()<<std::endl;
//    auto data=desc_single.data();
//
//    float x=desc_single(0,0);
//    float y=desc_single(1,0);
//    float z=desc_single(2,0);
//
//    std::vector<float> desc_single_vec;
//    for(int i=0;i<128;++i)
//    {
//        desc_single_vec.push_back(desc_single(i,0));
//    }


//    std::cout<<"desc size:\n"<<desc_single.size()<<std::endl;
//    Eigen::VectorXf v(128);
//    for(int i=0;i<128;++i)
//    {
//        v(i)=desc_single(i,0);
//    }
//    std::cout<<"Before Normalize:\n"<<v<<std::endl;
//    std::cout<<"Norm:"<<v.norm()<<std::endl;
//    v.normalize();
//    std::cout<<"After Normalze:\n"<<v<<std::endl;
//
//    std::cout<<v(0)<<" "<<v(1)<<std::endl;
//
//    Eigen::VectorXf v;
//    v.resize(10);
//    std::cout<<"v:\n";
//    std::cout<<v<<std::endl;
//    std::cout<<"v size:"<<v.size()<<std::endl;
//    v.resize(20);
//    std::cout<<"v:\n";
//    std::cout<<v<<std::endl;
//    std::cout<<"v size:"<<v.size()<<std::endl;
//    v.resize(8);
//    std::cout<<"v:\n";
//    std::cout<<v<<std::endl;
//    std::cout<<"v size:"<<v.size()<<std::endl;


//    int sum{0};
//    t1=steady_clock::now();

//#pragma omp parallel for
//    for(int i=0;i<descs.size();++i)
//    {
//        for(int j=0;j<128;++j)
//        {
//            ++sum;
//        }
//    }
//    t2=steady_clock::now();
//
//    time_span=duration_cast<duration<double>>(t2-t1);
//    std::cout<<sum<<std::endl;
//    std::cout<<"It takes "<<time_span.count()<<" seconds\n";

//
//    std::cout<<"In hist1:\n";
//    for(const auto & hist:hist_1)
//    {
//        std::cout<<"hist "<<hist.first<<":\n";
//        for(const auto & bin:hist.second)
//        {
//            std::cout<<"bin "<<bin.first<<std::endl;
//            for(const auto & val:bin.second)
//            {
//                std::cout<<"val "<<val.first<<" :"<<val.second<<std::endl;
//            }
//        }
//    }
//
//    std::cout<<"In histx:\n";
//    for(const auto & hist:hist_x)
//    {
//        std::cout<<"hist "<<hist.first<<":\n";
//        for(const auto & bin:hist.second)
//        {
//            std::cout<<"bin "<<bin.first<<std::endl;
//            for(const auto & val:bin.second)
//            {
//                std::cout<<"val "<<val.first<<" :"<<val.second<<std::endl;
//            }
//        }
//    }
//
//    std::cout<<"In histy:\n";
//    for(const auto & hist:hist_y)
//    {
//        std::cout<<"hist "<<hist.first<<":\n";
//        for(const auto & bin:hist.second)
//        {
//            std::cout<<"bin "<<bin.first<<std::endl;
//            for(const auto & val:bin.second)
//            {
//                std::cout<<"val "<<val.first<<" :"<<val.second<<std::endl;
//            }
//        }
//    }
//
//    std::cout<<"In histo  :\n";
//    for(const auto & hist:hist_o)
//    {
//        std::cout<<"hist "<<hist.first<<":\n";
//        for(const auto & bin:hist.second)
//        {
//            std::cout<<"bin "<<bin.first<<std::endl;
//            for(const auto & val:bin.second)
//            {
//                std::cout<<"val "<<val.first<<" :"<<val.second<<std::endl;
//            }
//        }
//    }


    return 0;
}