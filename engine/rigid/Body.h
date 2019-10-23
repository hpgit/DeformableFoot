#ifndef _BODY_H_
#define _BODY_H_

#include <string>
#include <Eigen/Dense>

class Body
{
public:
	// methods
	Body();
	~Body();

	Eigen::Isometry3d Transform();
	Eigen::Rotate3d Rotation();
	Eigen::Vector3d Translation();
	
	Eigen::Vector6d BodyVelGlobal();

	void SetMass(double _m);
	Eigen::MatrixXd GetMassMatrix();
	Eigen::MatrixXd GetMassMatrixGlobalCom();

	Eigen::Vector3d ToWorld(Eigen::Vector3d& local_offset);

	Eigen::MatrixXd GetLinearJacobian(Eigen::Vector3d& local_offset);

	double GetKineticEnergy();
	double GetPotentialEnergy(Eigen::Vector3d& _g);

	void SetBoxSize(Eigen::Vector3d& _size);

public:
	// class variables
	int idx;
	std::string name;
	Eigen::Isometry3d T_g;
	Eigen::MatrixXd M_b;
	Eigen::Vector6d v_g;

	Eigen::Vector3d box_size;

	
};

#endif  // _BODY_H_