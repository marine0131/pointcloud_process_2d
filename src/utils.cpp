#include <dirent.h>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <regex>

#include "utils.h"


vector<string> get_files(const char* folder)
{
      struct dirent *ptr;
      DIR *dir;
      dir = opendir(folder);
      if(dir == NULL)
      {
          cout << "can not open folder: " << folder <<endl;
          return vector<string>();
      }
      vector<string> files;
      string folder_str(folder);
      while((ptr=readdir(dir))!=NULL)
      {
          // skip "." and ".." file
          if(ptr->d_name[0] == '.')
              continue;
          string cc = folder_str + string(ptr->d_name); 
          //files.push_back(const_cast<char*>(cc.c_str()));
          files.push_back(cc);
      }
      closedir(dir);
      sort(files.begin(), files.end());
      return files;
}

vector<string> get_dirs(const char* folder)
{
      struct dirent *ptr;
      DIR *dir;
      dir = opendir(folder);
      vector<string> files;
      string folder_str(folder);
      while((ptr=readdir(dir))!=NULL)
      {
          // skip "." and ".." file
          if(ptr->d_name[0] == '.')
              continue;
          string cc = folder_str + string(ptr->d_name)+"/"; 
          //files.push_back(const_cast<char*>(cc.c_str()));
          files.push_back(cc);
      }
      closedir(dir);
      sort(files.begin(), files.end());
      return files;
}

void read_2dmat_from_file(string filename, vector<vector<float> >* mat)
{
    ifstream is(filename);
    if(!is)
    {
        cout << "file not found!"<< endl;
        return;
    }
    regex pat_regex("\\d+(\\.\\d+)?");

    string line;
    vector<float> vec;
    while(getline(is, line))
    {
        for(sregex_iterator it(line.begin(), line.end(), pat_regex), end_it; it != end_it; ++it)
        {
            vec.push_back(stof(it->str()));
        }
        mat->push_back(vec);
        vec.clear();
    }

}
