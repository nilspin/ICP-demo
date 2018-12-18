#ifndef DEBUGHELPER_HPP
#define DEBUGHELPER_HPP

#include <iostream>
#include <array>
#include <fstream>
#include <vector>
#include <algorithm>
#include <stdexcept>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/string_cast.hpp>

using glm::vec3;
using glm::vec4;
using std::vector;
using std::cout;
using std::ofstream;
using std::fill;

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
void ClearVector(vector<T>& V) {
  fill(V.begin(), V.end(), T(0));
  //for_each(V.begin(), V.end(), [](T& temp){temp=T(0);});
}

template<typename T>
void PrintArray(const vector<T> h_array) {
  for(const T& v : h_array) {
    cout<<glm::to_string(v)<<"\n";
  }
}

template<typename T>
void checkEquality(const vector<T>& A, const vector<T>& B)  {
  for(auto i=0; i < A.size();  ++i) {
    if(A[i]!=B[i]){
      std::runtime_error("Mismatch at position "+std::to_string(i));
    }
  }
  cout<<"Arrays are same.\n";
}
#endif