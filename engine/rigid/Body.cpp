#include "Body.h"
#include "../util/hpmath.h"

#include <iostream>

Body::Body(const char* name, Eigen::Affine3d &init_transform, double _mass) {
	this->name = name;
	this->T_g = init_transform;
	this->mass = _mass;
	this->M_b = _mass * Eigen::MatrixXd(6, 6).setIdentity();
	this->v_g = Eigen::VectorXd(6);
	this->v_g.setZero();

	this->idx = 0;

	this->box_size << 1.5, 0.5, 0.5;
}


void Body::SetMass(double _mass) {
    this->mass = _mass;
	for(Eigen::Index i=0; i < 6; i++)
		this->M_b(i, i) = _mass;
}

Eigen::MatrixXd Body::GetMassMatrixGlobalCom() {
//	Eigen::Matrix3d inertia = this->M_b.block<3, 3>(3, 3);
//	Eigen::Matrix3d rot = this->T_g.linear();
	Eigen::MatrixXd M_g = this->M_b;
	M_g.block<3, 3>(3, 3) = this->T_g.linear() * this->M_b.block<3, 3>(3, 3) * this->T_g.linear().transpose();

	return M_g;
}

Eigen::Vector3d Body::ToWorld(const Eigen::Vector3d &local_offset) {
	return this->T_g.linear() * local_offset + this->T_g.translation();
}

Eigen::MatrixXd Body::GetLinearJacobian(const Eigen::Vector3d &local_offset) {
	Eigen::MatrixXd J(3, 6);
//	Eigen::Vector3d arm = this->T_g.linear() * local_offset;
	J.block<3, 3>(0, 0) = -GetCrossMatrixForm(this->T_g.linear() * local_offset);
	J.block<3, 3>(0, 3).setIdentity();

//	std::cout << J << std::endl;

	return J;
}

double Body::GetKineticEnergy() {
	return .5 * this->v_g.dot(this->GetMassMatrixGlobalCom() * this->v_g);
}

double Body::GetPotentialEnergy(const Eigen::Vector3d &gravity) {
	return -gravity.dot(this->GetMassMatrix() * this->ToWorld());
}

void Body::SetBoxSize(const Eigen::Vector3d &size) {
	this->box_size = size;
}