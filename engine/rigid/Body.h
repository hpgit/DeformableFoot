#ifndef _BODY_H_
#define _BODY_H_

#include <string>
#include <Eigen/Dense>

class Body
{
public:
	// methods
	Body(const char* name, Eigen::Affine3d &init_transform, double mass=1.);

	Eigen::Affine3d Transform() { return this->T_g; }
	Eigen::Matrix3d Rotation() { return this->T_g.linear(); }
	Eigen::Vector3d Translation() { return this->T_g.translation(); }
	
	Eigen::VectorXd BodyVelGlobal() { return this->v_g; }

	void SetMass(double mass);
	Eigen::MatrixXd GetMassMatrix() { return this->M_b; }
	Eigen::MatrixXd GetMassMatrixGlobalCom();

	Eigen::Vector3d ToWorld(const Eigen::Vector3d& local_offset=Eigen::Vector3d::Zero());

	Eigen::MatrixXd GetLinearJacobian(const Eigen::Vector3d& local_offset=Eigen::Vector3d::Zero());

	double GetKineticEnergy();
	double GetPotentialEnergy(const Eigen::Vector3d& _g);

	void SetBoxSize(const Eigen::Vector3d& _size);

public:
	// class variables
	int idx;
	std::string name;
	double mass;
	Eigen::MatrixXd M_b;

	Eigen::Affine3d T_g;
	Eigen::VectorXd v_g;

	Eigen::Vector3d box_size;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif  // _BODY_H_