#ifndef _RJOINT_H_
#define _RJOINT_H_

#include <Eigen/Dense>
#include <string>
#include "Joint.h"
#include "Body.h"

class RJoint : public Joint
{
public:
	// methods
	RJoint(Body *_body_p, Eigen::Affine3d &transform_p, Body *_body_c, Eigen::Affine3d &transform_c);

	Eigen::Vector3d axis;

	void SetAxis(const Eigen::Ref<const Eigen::Vector3d>& _axis) {axis = _axis;}
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif  // _RJOINT_H_