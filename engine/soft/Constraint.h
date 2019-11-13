//
// Created by trif on 29/10/2019.
//

#ifndef DEFORMABLEFOOT_CONSTRAINT_H
#define DEFORMABLEFOOT_CONSTRAINT_H

#include <Eigen/Dense>
#include <iostream>

class CorotateConstraint {
public:
    CorotateConstraint(Eigen::VectorXd &_points, Eigen::VectorXi &_indices, double _mu=100000.)
    : points(_points), indices(_indices), mu(_mu)
    {
        this->init_A();
        this->init_S();
        this->init();
    }

    void init_S();
    void init_A();
    void init();

    Eigen::Matrix3d edge_matrix(Eigen::VectorXd &_points);
    Eigen::MatrixXd deformation_gradient(Eigen::VectorXd &points);
    Eigen::MatrixXd get_muSTATBp(Eigen::VectorXd &points);
    Eigen::MatrixXd get_L();


public:
    Eigen::VectorXd &points;
    Eigen::VectorXi indices;
    double mu;

    Eigen::Matrix3d invE_ref;
//    Eigen::MatrixXd A;
    Eigen::MatrixXd S;
    Eigen::MatrixXd L;
    Eigen::MatrixXd muSTAT;

private:
    Eigen::Matrix3d mR;
    Eigen::Vector3d mp_0;
    Eigen::Matrix3d mE;
    Eigen::JacobiSVD<Eigen::Matrix3d> msvd;
};


#endif //DEFORMABLEFOOT_CONSTRAINT_H
