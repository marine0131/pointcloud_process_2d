#include <iostream>
#include <vector>
#include <cstdio>
#include <string>
#include <pcl/visualization/cloud_viewer.h>   
#include <pcl/visualization/pcl_visualizer.h>   
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "utils.h"

using namespace std;
int Idx = 0;
bool DelFlag = false;


void keyboardEventCallback(const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
    if(event.getKeySym() == "n" && event.keyDown())
        Idx ++;
    if(event.getKeySym() == "p" && event.keyDown())
        Idx --;
    if(event.getKeySym() == "d" && event.keyDown())
    {
        DelFlag = true;
    }
}

int pcd_load(string filename, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    cout << "reading file: " << filename <<endl;
    if(-1 == pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *cloud))
    {
        cout << "cloud reading failed" << endl;
        return -1;
    }
    return 0;
}

bool delete_file(string filename)
{
    if(-1 == remove(filename.c_str()))
        return false;
    return true;
}


int main(int argc, char** argv)
{
    if(argc < 2)
    {
        cout << "not enough argument, please add folder" << endl;
        return -1;
    }
    if(strcmp(argv[1], "--h") == 0 || strcmp(argv[1], "-help")==0)
    {
        cout << "specify the folder you want to open" <<endl ;
        cout << "\tpress 'n' to show next point cloud" << endl;
        cout << "\tpress 'p' to show previos point cloud" << endl;
        cout << "\tpress 'd' to delete current point cloud" << endl;
        return 0;
    }

    char* folder = argv[1];
    vector<string> pcd_file_list = get_files(folder);

    pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::visualization::CloudViewer viewer("Viewer");
    viewer.registerKeyboardCallback(keyboardEventCallback, (void*)&viewer);

    int idx = Idx;
    pcd_load(pcd_file_list[idx], cloud);
    viewer.showCloud(cloud);

    while(!viewer.wasStopped())
    {
        if(Idx == pcd_file_list.size())
        {
            cout << "end of folder" <<endl;
            break;
        }
        if(DelFlag)
        {
            cout << "deleting file: " << pcd_file_list[Idx] <<endl;
            if(!delete_file(pcd_file_list[Idx]))
                cout << "delete failed!!" <<endl;
            pcd_file_list.erase(pcd_file_list.begin()+Idx);
            DelFlag = false;
            pcd_load(pcd_file_list[Idx], cloud);
            cout << "left files: " << pcd_file_list.size()-Idx << endl;

            viewer.showCloud(cloud);
        }

        if(idx != Idx)
        {
            pcd_load(pcd_file_list[Idx], cloud);
            viewer.showCloud(cloud);
            idx = Idx;
        }
    }
    return 0;

}
