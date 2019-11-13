//
// Created by trif on 06/11/2019.
//

#ifndef DEFORMABLEFOOT_CAMERA_H
#define DEFORMABLEFOOT_CAMERA_H

#include <Eigen/Dense>

class Camera {
public:
    Camera();

    Eigen::Affine3d getSE3();

    void transform();

    Eigen::Vector3d center;
    double rotateY;
    double rotateX;
    double distance;
};


#endif //DEFORMABLEFOOT_CAMERA_H
