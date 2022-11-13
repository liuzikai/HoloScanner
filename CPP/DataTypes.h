//
// Created by Zikai Liu on 11/13/22.
//

#ifndef HOLOSCANNER_DATATYPES_H
#define HOLOSCANNER_DATATYPES_H

#include <cstdint>
#include <vector>
#include <Eigen/Eigen>

using timestamp_t = uint64_t;

typedef std::vector<Eigen::Vector3f> PCD;  // point cloud data

class PCDSource {
public:

    virtual bool getNextPCDSince(timestamp_t &timestamp, PCD &pcd) = 0;
};

#endif //HOLOSCANNER_DATATYPES_H
