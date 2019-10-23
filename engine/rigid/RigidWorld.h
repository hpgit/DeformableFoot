#ifndef _RIGIDWORLD_H_
#define _RIGIDWORLD_H_


#include <vector>
#include <pair>
#include <Eigen/Dense>
#include "Body.hpp"
#include "BJoint.hpp"

class RigidWorld {
public:
	// methods
	RigidWorld(double h);
    ~RigidWorld();

	int GetNumBody();
    int GetConstraintNum();
    
    void AddBody(Body* _body);
    void AddJoint(BJoint* _joint);
    
    Body* GetBody(int body_idx);
    BJoint* GetJoint(int joint_idx);

    void SetGravity(Eigen::Vector3d& _gravity);

    Eigen::MatrixXd GetJointJacobian(int joint_idx);
    Eigen::MatrixXd GetJointJacobianRemain(int joint_idx);
    std::pair<Eigen::MatrixXd, Eigen::MatrixXd> GetSystemMatrix();

    void IntegratePosition(Eigen::Vector3d& vel);
    void step();

    double GetKineticEnergy();
    double GetPotentialEnergy();

    void draw();

public:
	// class variables
	double h;
    std::vector<Body> bodies;
    std::vector<BJoint> joints;
    Eigen::Vector3d gravity;
};

#endif  // _RIGIDWORLD_H_