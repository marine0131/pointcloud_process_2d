#include <vector>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <fstream>

#include "utils.h"
#include "calc_features.h"


using namespace std;

struct Args{
    string data_folder;
    string output;
    float  mean_k;
    float std_dev_mult_thresh;
    float leaf_size_x;
    float leaf_size_y;
    float leaf_size_z;
    Args()
    {
        output = "./feat.xml";
        mean_k = 10.0;
        std_dev_mult_thresh = 1.0;
        leaf_size_x = 0.02;
        leaf_size_y = 0.02;
        leaf_size_z = 0.02;
    }
};


void cluster_filter(Args args, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud)
{ 
     // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (
     //         new pcl::PointCloud<pcl::PointXYZ>);
     //using statistical removal
     pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
     sor.setInputCloud(cloud);
     sor.setMeanK(args.mean_k);
     sor.setStddevMulThresh(args.std_dev_mult_thresh);
     sor.filter(*cloud);
     // voxel filter
     pcl::VoxelGrid<pcl::PointXYZ> sor1;
     sor1.setInputCloud(cloud);
     sor1.setLeafSize(args.leaf_size_x, args.leaf_size_y, args.leaf_size_z);
     sor1.filter(*filtered_cloud);
} 

vector<float> get_features(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    return calcPeopleFeatures(cloud);
}

void save_features(vector<vector<float> >* features, string filename)
{
    ofstream outf;
    cout << "saving features to file " << filename << endl;
    outf.open(filename);
    for(vector<vector<float> >::iterator it= features->begin(); it!= features->end(); ++it)
    {
        for(vector<float>::iterator iit =it->begin(); iit!=it->end(); ++iit)
            outf << to_string(*iit) << " ";
        outf << "\n";
    }
}

  
int main(int argc, char** argv)
{ 
    Args args;
    vector<string> arg_vec(argv+1, argv+argc);
    for(vector<string>::iterator it = arg_vec.begin(); it!= arg_vec.end(); ++it)
    {
        int pos = it->find("=");
        string arg_string = it->substr(2, pos-2);
        if(arg_string.compare("help") == 0){
            cout << "--data_folder=/pcdfiles/folder/\n"
                << "--output=output.xml\n"
                << "--mean_k=10 \n"
                << "--std_dev_mult_thresh=1.0\n"
                << "leaf_size_x/y/z=0.02"<< endl;
            return 0;
        }
        else if(arg_string.compare("data_folder") == 0){
            args.data_folder= it->substr(pos+1);
        }
        else if(arg_string.compare("output") == 0){
            args.output= it->substr(pos+1);
        }
        else if(arg_string.compare("mean_k") == 0){
            args.mean_k = stof(it->substr(pos+1));
        }
        else if(arg_string.compare("std_dev_mult_thresh") == 0){
            args.std_dev_mult_thresh= stof(it->substr(pos+1));
        }
        else if(arg_string.compare("leaf_size_x") == 0){
            args.leaf_size_x= stof(it->substr(pos+1));
        }
        else if(arg_string.compare("leaf_size_y") == 0){
            args.leaf_size_y= stof(it->substr(pos+1));
        }
        else if(arg_string.compare("leaf_size_z") == 0){
            args.leaf_size_z= stof(it->substr(pos+1));
        }
        else{
            cout << "error: input the right arguments: " << arg_string<< endl;
            return 0;
        }
    }
    if(args.data_folder.empty())
    {
        cout << "no training data specified, use --data_folder=****" <<endl;
        return -1;
    }

    float response_idx = 0.f;
    vector<vector<float> > features;

    vector<string> training_folders = get_dirs(args.data_folder.c_str());
    for(vector<string>::iterator it = training_folders.begin(); it != training_folders.end(); ++it)
    {
        vector<string> pcdfiles= get_files(it->c_str());
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for(vector<string>::iterator file_it = pcdfiles.begin(); 
                file_it!=pcdfiles.end(); ++file_it)
        {
            cout << "loading file " << *file_it << endl;
            if(pcl::io::loadPCDFile<pcl::PointXYZ>(*file_it, *cloud) == -1)
            {
                cout << "couldn't load file" << endl;
                continue;
            }
            cluster_filter(args, cloud, filtered_cloud);
            vector<float> feat =  get_features(filtered_cloud);
            feat.push_back(response_idx);
            features.push_back(feat);
        }
        response_idx += 1;
    }

    save_features(&features, args.output);
}
