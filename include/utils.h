#ifndef UTILS_H_
#define UTILS_H_


#include <string>
#include <vector>

using namespace std;

vector<string> get_files(const char* folder);
vector<string> get_dirs(const char* folder);
void read_2dmat_from_file(string, vector<vector<float> >*);

#endif
