#ifndef _JOINT_H_
#define _JOINT_H_

#include <Eigen/Dense>
#include <string>
#include "Body.h"

class Joint
{
public:
	// methods
	Joint(Body *_body_p, Eigen::Affine3d &transform_p, Body *_body_c, Eigen::Affine3d &transform_c);
	
	virtual Eigen::Affine3d GetJointBeforeTransform();
	virtual Eigen::Affine3d GetJointAfterTransform();
	virtual Eigen::Affine3d GetJointTransform();

public:
	int idx;
	int dof;
	char type;
	std::string name;

	Body* body_p;
	Body* body_c;

	Eigen::Affine3d T_p;
	Eigen::Affine3d T_c;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif  // _BJOINT_H_