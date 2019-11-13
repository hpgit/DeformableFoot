#ifndef _BJOINT_H_
#define _BJOINT_H_

#include <Eigen/Dense>
#include <string>
#include "Body.h"

class BJoint
{
public:
	// methods
	BJoint(Body *_body_p, Eigen::Affine3d &transform_p, Body *_body_c, Eigen::Affine3d &transform_c);
	
	Eigen::Affine3d GetJointBeforeTransform();
	Eigen::Affine3d GetJointAfterTransform();
	Eigen::Affine3d GetJointTransform();

public:
	int idx;
	int dof;
	std::string name;

	Body* body_p;
	Body* body_c;

	Eigen::Affine3d T_p;
	Eigen::Affine3d T_c;
	
};

#endif  // _BJOINT_H_