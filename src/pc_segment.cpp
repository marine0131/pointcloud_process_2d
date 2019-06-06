#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>   
#include <pcl/common/transforms.h>   

#include "pc_segment.h"
#include "utils.h"

using namespace std;


class PclSegment
{
public:
    float mean_k_;
    float std_dev_mult_thresh_;
    float leaf_size_x_;
    float leaf_size_y_;
    float leaf_size_z_;
    float cluster_tolerance_;
    float min_cluster_size_;
    float max_cluster_size_;
    string output_folder_;

    PclSegment(Args args)
    {
        output_folder_ = args.output_folder;
        mean_k_             = args.mean_k;
        std_dev_mult_thresh_ = args.std_dev_mult_thresh;
        leaf_size_x_        = args.leaf_size_x;
        leaf_size_y_        = args.leaf_size_y;
        leaf_size_z_        = args.leaf_size_z;
        cluster_tolerance_  = args.cluster_tolerance;
        min_cluster_size_   = args.min_cluster_size;
        max_cluster_size_   = args.max_cluster_size;
    }

    void segment(string pcdfile)
    {
        cout << "Loading data from file: "<< pcdfile << endl;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if(pcl::io::loadPCDFile<pcl::PointXYZ>(pcdfile, *cloud) == -1)
        {
            cout << "couldn't load file" << endl;
            return;
        }

        //**************pcl filter***************
        //filtering the cloud by using statistical removal
        // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
        // if(pcl_cloud->points.size() > 10)
        // {
        //     pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;                          
        //     sor.setInputCloud(pcl_cloud);                                                   
        //     sor.setMeanK(mean_k_);                                                           
        //     sor.setStddevMulThresh(std_dev_mult_thresh_);                                                
        //     sor.filter(*pcl_cloud);   

        //     // voxel
        //     pcl::VoxelGrid<pcl::PointXYZ> sor1;                                         
        //     sor1.setInputCloud(pcl_cloud);                                                  
        //     sor1.setLeafSize(leaf_size_x_, leaf_size_y_, leaf_size_z_);                                      
        //     sor1.filter(*cloud_filtered);  
        // }

        //*************pcl cluster**************
        std::vector<pcl::PointIndices> cluster_indices;
        if(cloud->points.size() > min_cluster_size_)
        {
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
            tree->setInputCloud (cloud);  

            pcl::EuclideanClusterExtraction<pcl::PointXYZ> ece;
            ece.setClusterTolerance(cluster_tolerance_); //15cm, the unit is meter
            ece.setMinClusterSize(min_cluster_size_); //the minimun number of points to form a cluster is 15
            ece.setMaxClusterSize(max_cluster_size_); //the maximum number of points in a cluster
            ece.setSearchMethod(tree);
            ece.setInputCloud(cloud);
            ece.extract(cluster_indices);
        }


        //*************extract and save cluster ****************
        int ind = 0;
        for (vector<pcl::PointIndices>::const_iterator it=cluster_indices.begin(); it!=cluster_indices.end(); ++it)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cluster (new pcl::PointCloud<pcl::PointXYZ>);
            float c_x = 0.0;
            float c_y = 0.0;
            for (vector<int>::const_iterator j=it->indices.begin(); j!=it->indices.end(); ++j)
            {
                cluster->points.push_back(cloud->points[(int)(*j)]);
                c_x += cloud->points[*j].x;
                c_y += cloud->points[*j].y;
            }
            c_x /= cluster->points.size();
            c_y /= cluster->points.size();

            cluster->width = cluster->points.size();
            cluster->height = 1;
            cluster->is_dense = true; //no invalid points        
            for (pcl::PointCloud<pcl::PointXYZ>::iterator j=cluster->begin(); j!=cluster->end(); ++j)
            {
                j->x = j->x-c_x;
                j->y = j->y-c_y;
            }

            // save pcl to pcd file
            int pos = pcdfile.find_last_of('/');
            int len = pcdfile.find_last_of('.') - pos -1;
            string file_name(pcdfile.substr(pos+1, len));
            file_name = output_folder_ + file_name + "_" + to_string(ind) + ".pcd";
            pcl::io::savePCDFile(file_name, *cluster);
            cout <<  "saving to file: " << file_name <<endl;
                
            cluster->clear();
            ind ++;
        }
 
    }

};


int main(int argc, char **argv)
{

    Args args;
    vector<string> arg_vec(argv+1, argv+argc);
    for(vector<string>::iterator it = arg_vec.begin(); it!= arg_vec.end(); ++it)
    {
        int pos = it->find("=");
        string arg_string = it->substr(2, pos-2);
        if(arg_string.compare("help") == 0){
            cout << "--input_folder=/input/folder\n" 
                << "--output_folder=/output/folder\n" 
                << "--cluster_toleranc=0.3\n"
                << "--min_cluster_siz=10\n"
                << "--max_cluster_siz=2000" << endl;
            return 0;
        }
        else if(arg_string.compare("input_folder") == 0){
            args.input_folder = it->substr(pos+1);
        }
        else if(arg_string.compare("output_folder") == 0){
            args.output_folder = it->substr(pos+1);
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
        else if(arg_string.compare("cluster_tolerance") == 0){
            args.cluster_tolerance= stof(it->substr(pos+1));
        }
        else if(arg_string.compare("min_cluster_size") == 0){
            args.min_cluster_size= stoi(it->substr(pos+1));
        }
        else if(arg_string.compare("max_cluster_size") == 0){
            args.max_cluster_size= stoi(it->substr(pos+1));
        }
        else{
            cout << "error: input the right arguments: " << arg_string<< endl;
            return 0;
        }
    }
    if(args.input_folder.empty() || args.output_folder.empty())
    {
        cout << "no input_folder or output_folder specified!" <<endl;
        return -1;
    }

    vector<string> files = get_files(args.input_folder.c_str());

    PclSegment ps(args);

    cout << "Loading data..." << endl;
    for (vector<string>::iterator it = files.begin(); it != files.end(); ++it)
    {
        ps.segment(*it);
    }
    return 0;
}
