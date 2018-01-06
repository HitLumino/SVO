/*
 * homography.cpp
 * Adaptation of PTAM-GPL HomographyInit class.
 * https://github.com/Oxford-PTAM/PTAM-GPL
 * Licence: GPLv3
 * Copyright 2008 Isis Innovation Limited
 *
 *  Created on: Sep 2, 2012
 *      by: cforster
 *
 * This class implements the homography decomposition of Faugeras and Lustman's
 * 1988 tech report. Code converted to Eigen from PTAM.
 *
 */

#ifndef ESSENTIAL_H_
#define ESSENTIAL_H_
#include <Eigen/Core>
#include <Eigen/StdVector>
#include <Eigen/SVD>

namespace vk{
using namespace Eigen;
using namespace std;

struct EssentialDecomposition
{
    Vector3d t;
    Matrix3d R;
};

class Essential {
public:   
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    Essential(const vector<Vector2d,  aligned_allocator<Vector2d> >& _fts1, 
                const vector<Vector2d,  aligned_allocator<Vector2d> >& _fts2, 
                double _error_multiplier2, 
                double _thresh_in_px);
    
    
    
    

};
}

#endif