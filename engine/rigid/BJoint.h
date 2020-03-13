#ifndef _BJOINT_H_
#define _BJOINT_H_

#include <Eigen/Dense>
#include <string>
#include "Joint.h"
#include "Body.h"

class BJoint : public Joint
{
public:
	// methods
	BJoint(Body *_body_p, Eigen::Affine3d &transform_p, Body *_body_c, Eigen::Affine3d &transform_c);

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif  // _BJOINT_H_