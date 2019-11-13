//
// Created by trif on 07/11/2019.
//

#include "hpmath.h"

Eigen::Matrix3d GetCrossMatrixForm(const Eigen::Ref<const Eigen::Vector3d> &w) {
    Eigen::Matrix3d W;
    W <<
    0., -w[2], w[1],
    w[2], 0., -w[0],
    -w[1], w[0], 0.;

    return W;
}