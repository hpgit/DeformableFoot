#include "RigidWorld.h"
#include "../util/hpmath.h"

#ifndef GL_SILENCE_DEPRECATION
#define GL_SILENCE_DEPRECATION
#endif

#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include <iostream>

using Eigen::Ref;

RigidWorld::RigidWorld(double _h) {
    this->h = _h;
    this->bodies.clear();
    this->joints.clear();

    this->name_to_idx.clear();
    this->gravity << 0., -9.8, 0.;
}

RigidWorld::~RigidWorld() {
    for (std::vector<Body *>::size_type i=0; i < this->bodies.size(); i++)
        delete this->bodies[i];
    for (std::vector<BJoint *>::size_type i=0; i < this->joints.size(); i++)
        delete this->joints[i];
}


int RigidWorld::GetConstraintNum() {
    int num = 0;
    for (std::vector<BJoint *>::size_type i=0; i < this->joints.size(); i++)
        num += 6 - this->joints[i]->dof;

    return num;
}

void RigidWorld::AddBody(Body *_body) {
    this->bodies.push_back(_body);
    _body->idx = this->bodies.size() - 1;
    this->name_to_idx[_body->name] = _body->idx;
}

void RigidWorld::AddJoint(BJoint *_joint) {
    this->joints.push_back(_joint);
    _joint->idx = this->joints.size() - 1;
}

Eigen::MatrixXd RigidWorld::GetJointJacobian(int joint_idx) {
    BJoint *joint = this->joints[joint_idx];
    Eigen::MatrixXd K_joint_i(3, 6*this->bodies.size());
    K_joint_i.setZero();
    K_joint_i.block<3, 6>(0, joint->body_p->idx * 6) = joint->body_p->GetLinearJacobian(joint->T_p.translation());
    K_joint_i.block<3, 6>(0, joint->body_c->idx * 6) = -joint->body_c->GetLinearJacobian(joint->T_c.translation());

    return K_joint_i;
}

Eigen::MatrixXd RigidWorld::GetJointJacobianRemain(int joint_idx) {
    BJoint *joint = this->joints[joint_idx];
    return -joint->body_p->ToWorld(joint->T_p.translation()) + joint->body_c->ToWorld(joint->T_c.translation());
}

void RigidWorld::GetSystemMatrix(Ref<Eigen::MatrixXd> Aout, Ref<Eigen::VectorXd> bout) {
    int n_bodies = this->bodies.size();
    int constraint_dof = this->GetConstraintNum();

//    Aout.resize(6 * n_bodies + constraint_dof ,  6 * n_bodies + constraint_dof);
//    bout.resize(6 * n_bodies + constraint_dof);
    assert(Aout.rows() == Aout.cols() && Aout.rows() == bout.rows() && Aout.rows() == 6*n_bodies + constraint_dof);

    // set M_hat
    Eigen::MatrixXd M_hat(6*n_bodies, 6*n_bodies);
    M_hat.setZero();
    for (std::vector<Body *>::size_type i=0; i < this->bodies.size(); i++)
        M_hat.block<6, 6>(6*i, 6*i) = this->bodies[i]->GetMassMatrixGlobalCom();

    // about joint constraints
    Eigen::MatrixXd K_joint(constraint_dof, 6*n_bodies);
    K_joint.setZero();
    int dof_offset = 0;
    for (std::vector<BJoint *>::size_type j=0; j < this->joints.size(); j++) {
        K_joint.block(dof_offset, 0, 6 - this->joints[j]->dof, 6*n_bodies) = this->GetJointJacobian(j);
        dof_offset += 6 - this->joints[j]->dof;
    }

//    std::cout << K_joint << std::endl;

    // set A
    Aout.topLeftCorner(6*n_bodies, 6*n_bodies) = M_hat;
    Aout.topRightCorner(6*n_bodies, constraint_dof) = -this->h * K_joint.transpose();
    Aout.bottomLeftCorner(constraint_dof, 6*n_bodies) = this->h * K_joint;
    Aout.bottomRightCorner(constraint_dof, constraint_dof).setZero();

//    std::cout << Aout << std::endl;

    // set b
    for (std::vector<Body *>::size_type i=0; i < this->bodies.size(); i++) {
        Body *body = this->bodies[i];
        Eigen::VectorXd v = body->BodyVelGlobal();
        Eigen::VectorXd Mv = body->GetMassMatrixGlobalCom() * v;
        bout.segment<6>(6*i) = Mv;
        bout.segment<3>(6*i) -= v.head<3>().cross(Mv.head<3>());
        bout.segment<3>(6*i+3) += this->h * body->mass * this->gravity;
    }

    // set torque
    // b[:3] += h * np.dot(joint_transformation[0][0][:3, :3].T, torque)
    // b[6:9] += -h * np.dot(joint_transformation[0][1][:3, :3].T, torque)

    // adjust b about joint constraints
    dof_offset = 6*n_bodies;
    for (std::vector<BJoint *>::size_type j=0; j < this->joints.size(); j++) {
        bout.segment(dof_offset, 6-this->joints[j]->dof) = this->GetJointJacobianRemain(j);
        dof_offset += 6 - this->joints[j]->dof;
    }
}

void RigidWorld::IntegratePosition(const Eigen::Ref<const Eigen::VectorXd> &vel) {
    Eigen::JacobiSVD<Eigen::Matrix3d> svd;
    for (std::vector<Body *>::size_type i=0; i < this->bodies.size(); i++) {
        Body *body = this->bodies[i];
        body->v_g = vel.segment<6>(6*i);
        Eigen::Matrix3d R_new = (Eigen::Matrix3d::Identity() + this->h * GetCrossMatrixForm(body->v_g.head<3>())) * body->Rotation();
        svd.compute(R_new, Eigen::ComputeFullU | Eigen::ComputeFullV);
        body->T_g.linear() = svd.matrixU() * svd.matrixV().transpose();
        body->T_g.translation() += this->h * body->v_g.tail<3>();
    }
}

void RigidWorld::Step() {
    int n_bodies = this->bodies.size();
    int constraint_dof = this->GetConstraintNum();

    Eigen::MatrixXd A(6 * n_bodies + constraint_dof ,  6 * n_bodies + constraint_dof);
    Eigen::VectorXd b(6 * n_bodies + constraint_dof);
    this->GetSystemMatrix(A, b);

//    std::cout << A << std::endl;
//    std::cout << "b:" << std::endl;
//    std::cout << b << std::endl;


    Eigen::VectorXd _x = A.ldlt().solve(b);
//    std::cout << "_x:" << std::endl;
//    std::cout << _x << std::endl;
    this->IntegratePosition(_x.head(6*this->bodies.size()));
}

double RigidWorld::GetKineticEnergy() {
    double energy = 0.;
    for (std::vector<Body *>::size_type i = 0; i < this->bodies.size(); i++) {
        Body *body = this->bodies[i];
        energy += body->GetKineticEnergy();
    }

    return energy;
}

double RigidWorld::GetPotentialEnergy() {
    double energy = 0.;
    for (std::vector<Body *>::size_type i = 0; i < this->bodies.size(); i++) {
        Body *body = this->bodies[i];
        energy += body->GetPotentialEnergy(Eigen::Vector3d(0., -9.8, 0.));
    }

    return energy;
}

void RigidWorld::Draw() {
    for (std::vector<Body *>::size_type i = 0; i < this->bodies.size(); i++) {
        Body *body = this->bodies[i];
        glPushMatrix();
        glMultMatrixd(body->Transform().matrix().data());
//        Eigen::AngleAxisd(body->Rotation()).axis();
        glScaled(body->box_size[0], body->box_size[1], body->box_size[2]);
        glColor3f(1., 0., 0.);
        glutSolidCube(1.);
        glPopMatrix();
    }

    // draw com
    glPushMatrix();
    Eigen::Vector3d com;
    com.setZero();
    for (std::vector<Body *>::size_type i = 0; i < this->bodies.size(); i++) {
        Body *body = this->bodies[i];
        com += body->ToWorld();
    }
    com /= this->bodies.size();
    glTranslated(com[0], com[1], com[2]);
    glPointSize(10.);
    glBegin(GL_POINTS);
    glColor3f(1., 1., 0.);
    glVertex3f(0., 0., 0.);
    glEnd();
    glPopMatrix();
}