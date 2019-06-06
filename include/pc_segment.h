#ifndef PC_SEGMENTATION_H_
#define PC_SEGMENTATION_H_

#include <string>
using namespace std;

struct Args{
    string input_folder;
    string output_folder;
    float  mean_k;
    float std_dev_mult_thresh;
      float leaf_size_x;
      float leaf_size_y;
      float leaf_size_z;
      float cluster_tolerance;
      int min_cluster_size;
      int max_cluster_size;
      Args()
      {
          mean_k = 10.0;
          std_dev_mult_thresh = 1.0;
          leaf_size_x = 0.02;
          leaf_size_y = 0.02;
          leaf_size_z = 0.02;
          cluster_tolerance = 0.2;
          min_cluster_size = 10;
          max_cluster_size = 2000;

      }
};


#endif
