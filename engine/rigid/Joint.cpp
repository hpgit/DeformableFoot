#include "Joint.h"

Joint::Joint(Body *_body_p, Eigen::Affine3d &transform_p, Body *_body_c, Eigen::Affine3d &transform_c) {
	this->body_p = _body_p;
	this->T_p = transform_p;
	this->body_c = _body_c;
	this->T_c = transform_c;

	this->dof = 3;
	this->idx = 0;
	this->name = "";
    this->type = 'X';
}

Eigen::Affine3d Joint::GetJointBeforeTransform() {
	return this->body_p->Transform() * this->T_p;
}

Eigen::Affine3d Joint::GetJointAfterTransform() {
	return this->body_c->Transform() * this->T_c;
}

Eigen::Affine3d Joint::GetJointTransform() {
	return this->GetJointBeforeTransform().inverse() * this->GetJointAfterTransform();
}