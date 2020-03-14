#include "RJoint.h"

RJoint::RJoint(Body *_body_p, Eigen::Affine3d &transform_p, Body *_body_c, Eigen::Affine3d &transform_c)
	: Joint(_body_p, transform_p, _body_c, transform_c)
{
	this->dof = 1;
	this->type = 'R';
	this->axis << 0., 0., 1.;
}



