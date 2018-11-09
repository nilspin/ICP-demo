#ifndef DEBUGHELPER_HPP
#define DEBUGHELPER_HPP

#include <iostream>
#include <array>
#include <fstream>
#include <vector>
#include <algorithm>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/string_cast.hpp>

using glm::vec3;
using glm::vec4;
using std::vector;
using std::cout;
using std::ofstream;
using std::fill_n;

template<typename T>
void WriteArrayToFile(const vector<T> h_array, std::string filename) {
  cout<<"Filename : "<<filename<<"\n";
  ofstream fout(filename.c_str());
  for(const T& v : h_array) {
    fout<<glm::to_string(v)<<"\n";
  }
  fout.close();
}

template<typename T>
void ClearVector(vector<T> V) {
  fill_n(V.begin(), V.size(), T(0));
}

template<typename T>
void PrintArray(const vector<T> h_array) {
  for(const T& v : h_array) {
    cout<<glm::to_string(v)<<"\n";
  }
}
#endif