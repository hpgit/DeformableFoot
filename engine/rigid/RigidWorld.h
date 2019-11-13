#ifndef _RIGIDWORLD_H_
#define _RIGIDWORLD_H_


#include <vector>
#include <map>
//#include <pair>
#include <Eigen/Dense>
#include "Body.h"
#include "BJoint.h"

using Eigen::Ref;

class RigidWorld {
public:
	// methods
	explicit RigidWorld(double h=1./300.);
	~RigidWorld();

	int GetNumBody() { return this->bodies.size(); }
    int GetConstraintNum();
    
    void AddBody(Body* _body);
    void AddJoint(BJoint* _joint);
    
    Body* GetBody(int body_idx) { return this->bodies[body_idx]; }
    BJoint* GetJoint(int joint_idx) { return this->joints[joint_idx]; }
    Body* GetBodyByName(const std::string &body_name) { return this->bodies[this->name_to_idx[body_name]];}

    void SetGravity(const Ref<const Eigen::Vector3d>& _gravity) { gravity = _gravity; }

    Eigen::MatrixXd GetJointJacobian(int joint_idx);
    Eigen::MatrixXd GetJointJacobianRemain(int joint_idx);
    void GetSystemMatrix(Ref<Eigen::MatrixXd> Aout, Ref<Eigen::VectorXd> bout);

    void IntegratePosition(const Ref<const Eigen::VectorXd> &vel);
    void Step();

    double GetKineticEnergy();
    double GetPotentialEnergy();

    void Draw();

public:
	// class variables
	double h;
    std::vector<Body *> bodies;
    std::vector<BJoint *> joints;
    std::map<std::string, int> name_to_idx;
    Eigen::Vector3d gravity;
};

#endif  // _RIGIDWORLD_H_