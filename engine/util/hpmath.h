//
// Created by trif on 07/11/2019.
//

#ifndef DEFORMABLEFOOT_HPMATH_H
#define DEFORMABLEFOOT_HPMATH_H

#include <Eigen/Dense>

Eigen::Matrix3d GetCrossMatrixForm(const Eigen::Ref<const Eigen::Vector3d> &w);

#endif //DEFORMABLEFOOT_HPMATH_H
