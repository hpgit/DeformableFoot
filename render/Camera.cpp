//
// Created by trif on 06/11/2019.
//

#include "Camera.h"

#ifndef GL_SILENCE_DEPRECATION
#define GL_SILENCE_DEPRECATION
#endif

#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif

Camera::Camera() {
    this->center << 0., 0., 0.;
    this->rotateX = -30.0 / 180. * M_PI;
    this->rotateY = 0.;
    this->distance = 20.;
}

Eigen::Affine3d Camera::getSE3() {
    Eigen::Affine3d SE3;
    SE3.translation() = this->center;
    SE3.rotate(Eigen::AngleAxisd(this->rotateY, Eigen::Vector3d::UnitY()));
    SE3.rotate(Eigen::AngleAxisd(this->rotateX, Eigen::Vector3d::UnitX()));
    SE3.translate(Eigen::Vector3d(0., 0., this->distance));
    return SE3;
}

void Camera::transform() {
//    glMultMatrixd(this->getSE3().inverse().matrix().transpose().data());
    glTranslated(0., 0., -this->distance);
    glRotated(-this->rotateX, 1., 0., 0.);
    glRotated(-this->rotateY, 0., 1., 0.);
    glTranslated(this->center[0], this->center[1], this->center[2]);

}
