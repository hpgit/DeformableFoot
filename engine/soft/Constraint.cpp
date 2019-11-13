//
// Created by trif on 29/10/2019.
//

#include "Constraint.h"

#include <iostream>

void CorotateConstraint::init_S() {
    this->S = Eigen::MatrixXd(12, this->points.rows());
    this->S.setZero();
    for(Eigen::Index i = 0; i < this->indices.rows(); i++) {
        this->S(3*i, 3*this->indices[i]) = 1.;
        this->S(3*i+1, 3*this->indices[i]+1) = 1.;
        this->S(3*i+2, 3*this->indices[i]+2) = 1.;
    }
}

void CorotateConstraint::init_A() {
    this->invE_ref = this->edge_matrix(this->points).inverse();
//    this->A = Eigen::MatrixXd(9, 12);
//    this->A <<
//            -invE_ref(0, 0) - invE_ref(1, 0) - invE_ref(2, 0), 0, 0, invE_ref(0, 0), 0, 0, invE_ref(1, 0), 0, 0, invE_ref(2, 0), 0, 0,
//            0, -invE_ref(0, 0) - invE_ref(1, 0) - invE_ref(2, 0), 0, 0, invE_ref(0, 0), 0, 0, invE_ref(1, 0), 0, 0, invE_ref(2, 0), 0,
//            0, 0, -invE_ref(0, 0) - invE_ref(1, 0) - invE_ref(2, 0), 0, 0, invE_ref(0, 0), 0, 0, invE_ref(1, 0), 0, 0, invE_ref(2, 0),
//            -invE_ref(0, 1) - invE_ref(1, 1) - invE_ref(2, 1), 0, 0, invE_ref(0, 1), 0, 0, invE_ref(1, 1), 0, 0, invE_ref(2, 1), 0, 0,
//            0, -invE_ref(0, 1) - invE_ref(1, 1) - invE_ref(2, 1), 0, 0, invE_ref(0, 1), 0, 0, invE_ref(1, 1), 0, 0, invE_ref(2, 1), 0,
//            0, 0, -invE_ref(0, 1) - invE_ref(1, 1) - invE_ref(2, 1), 0, 0, invE_ref(0, 1), 0, 0, invE_ref(1, 1), 0, 0, invE_ref(2, 1),
//            -invE_ref(0, 2) - invE_ref(1, 2) - invE_ref(2, 2), 0, 0, invE_ref(0, 2), 0, 0, invE_ref(1, 2), 0, 0, invE_ref(2, 2), 0, 0,
//            0, -invE_ref(0, 2) - invE_ref(1, 2) - invE_ref(2, 2), 0, 0, invE_ref(0, 2), 0, 0, invE_ref(1, 2), 0, 0, invE_ref(2, 2), 0,
//            0, 0, -invE_ref(0, 2) - invE_ref(1, 2) - invE_ref(2, 2), 0, 0, invE_ref(0, 2), 0, 0, invE_ref(1, 2), 0, 0, invE_ref(2, 2)
//        ;
}

void CorotateConstraint::init() {
    Eigen::MatrixXd A(9, 12);
    A <<
                        -invE_ref(0, 0) - invE_ref(1, 0) - invE_ref(2, 0), 0, 0, invE_ref(0, 0), 0, 0, invE_ref(1, 0), 0, 0, invE_ref(2, 0), 0, 0,
                        0, -invE_ref(0, 0) - invE_ref(1, 0) - invE_ref(2, 0), 0, 0, invE_ref(0, 0), 0, 0, invE_ref(1, 0), 0, 0, invE_ref(2, 0), 0,
                        0, 0, -invE_ref(0, 0) - invE_ref(1, 0) - invE_ref(2, 0), 0, 0, invE_ref(0, 0), 0, 0, invE_ref(1, 0), 0, 0, invE_ref(2, 0),
                        -invE_ref(0, 1) - invE_ref(1, 1) - invE_ref(2, 1), 0, 0, invE_ref(0, 1), 0, 0, invE_ref(1, 1), 0, 0, invE_ref(2, 1), 0, 0,
                        0, -invE_ref(0, 1) - invE_ref(1, 1) - invE_ref(2, 1), 0, 0, invE_ref(0, 1), 0, 0, invE_ref(1, 1), 0, 0, invE_ref(2, 1), 0,
                        0, 0, -invE_ref(0, 1) - invE_ref(1, 1) - invE_ref(2, 1), 0, 0, invE_ref(0, 1), 0, 0, invE_ref(1, 1), 0, 0, invE_ref(2, 1),
                        -invE_ref(0, 2) - invE_ref(1, 2) - invE_ref(2, 2), 0, 0, invE_ref(0, 2), 0, 0, invE_ref(1, 2), 0, 0, invE_ref(2, 2), 0, 0,
                        0, -invE_ref(0, 2) - invE_ref(1, 2) - invE_ref(2, 2), 0, 0, invE_ref(0, 2), 0, 0, invE_ref(1, 2), 0, 0, invE_ref(2, 2), 0,
                        0, 0, -invE_ref(0, 2) - invE_ref(1, 2) - invE_ref(2, 2), 0, 0, invE_ref(0, 2), 0, 0, invE_ref(1, 2), 0, 0, invE_ref(2, 2)
                    ;
    Eigen::MatrixXd AS = A * this->S;
//    Eigen::MatrixXd AS = this->A * this->S;
    this->muSTAT = this->mu * AS.transpose();
    this->L = this->muSTAT * AS;
}

Eigen::Matrix3d CorotateConstraint::edge_matrix(Eigen::VectorXd &_points) {
    this->mp_0 = _points.segment<3>(3*this->indices[0]);
    for (Eigen::Index i=0; i < 3; i++)
        this->mE.col(i) = _points.segment<3>(3*this->indices[i+1]) - this->mp_0;

    return this->mE;
}

Eigen::MatrixXd CorotateConstraint::deformation_gradient(Eigen::VectorXd &_points) {
    return this->edge_matrix(_points) * this->invE_ref;
}

Eigen::MatrixXd CorotateConstraint::get_L() {
    return this->L;
}

Eigen::MatrixXd CorotateConstraint::get_muSTATBp(Eigen::VectorXd &_points) {
    Eigen::JacobiSVD<Eigen::Matrix3d>svd(this->edge_matrix(this->points) * this->invE_ref, Eigen::ComputeFullU | Eigen::ComputeFullV);
    mR = svd.matrixU() * svd.matrixV().transpose();

    return this->muSTAT * Eigen::Map<Eigen::VectorXd>(mR.data(), mR.size());
}
