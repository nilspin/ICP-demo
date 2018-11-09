#ifndef COMMON_H
#define COMMON_H

#include <cstdint>
#include <limits>

const float fx = 525.0f;
const float fy =  525.0f;
const float cx =  319.5f;
const float cy =  239.5f;

const float distThres = 1.0f;
const float normalThres = -1.0f;
const float idealError = 0.0f;

const int numCols =  640;
const int numRows =  480;

//const float MAXF = std::numeric_limits<float>::max();
//const float MINF = std::numeric_limits<float>::quiet_NaN();
const float MINF = std::numeric_limits<float>::min();

//#define MINF 0xff800000
//#define MAXF 0x7F7FFFFF

#endif