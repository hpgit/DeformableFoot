#ifndef _BJOINT_H_
#define _BJOINT_H_

#include <Eigen/Dense>
#include <string>
#include "Body.h"

class BJoint
{
public:
	// methods
	BJoint();
	~BJoint(); 
	Eigen::Isometry3d GetJointBeforeTransform();
	Eigen::Isometry3d GetJointAfterTransform();

public:
	int idx;
	int dof;
	std::string name;

	Body* body_p;
	Body* body_c;

	Eigen::Isometry3d T_p;
	Eigen::Isometry3d T_c;
	
};

#endif  // _BJOINT_H_