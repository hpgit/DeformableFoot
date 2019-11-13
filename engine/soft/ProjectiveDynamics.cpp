//
// Created by trif on 29/10/2019.
//

#include "ProjectiveDynamics.h"
#ifndef GL_SILENCE_DEPRECATION
#define GL_SILENCE_DEPRECATION
#endif

#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif

#include <iostream>


void ProjectiveDynamics::init() {
    this->dx = Eigen::VectorXd(this->x.rows()).setZero();
    this->M = Eigen::MatrixXd(this->x.rows(), this->x.rows()).setIdentity();
    this->M_h_sq = this->inv_h_sq * Eigen::MatrixXd(this->x.rows(), this->x.rows()).setIdentity();
    this->invM = this->M;
    this->L = Eigen::MatrixXd(this->x.rows(), this->x.rows()).setZero();
    this->M_L = Eigen::MatrixXd(this->x.rows(), this->x.rows()).setZero();
    this->invM_L = Eigen::MatrixXd(this->x.rows(), this->x.rows()).setZero();

    this->n_points = this->x.rows() / 3;

    for(Eigen::Index i=0; i < this->cells.rows()/4 ; i++) {
        Eigen::VectorXi indices = this->cells.segment<4>(4*i);
        this->constraints.emplace_back(this->x, indices);
    }

    this->update_invM_L();
}

int ProjectiveDynamics::get_total_dof() {
    return this->n_points * 3;
}

Eigen::Vector3d ProjectiveDynamics::get_node(int node_idx) {
    return this->x.segment<3>(3*node_idx, 3);
}

int ProjectiveDynamics::add_position_constraint(int point_idx) {
    return 0;
}

void ProjectiveDynamics::update_invM_L() {
    this->L = Eigen::MatrixXd::Zero(this->M.rows(), this->M.cols());
    for (auto constraint : this->constraints)
        this->L += constraint.get_L();

    this->M_L = this->M_h_sq + this->L;
    this->invM_L = this->M_L.inverse();
}

Eigen::VectorXd ProjectiveDynamics::global_solve(Eigen::VectorXd &SABp, Eigen::VectorXd &s_n) {
    return this->invM_L * ( this->M_h_sq * s_n + SABp );
}

Eigen::VectorXd ProjectiveDynamics::local_solve(Eigen::VectorXd &x_next) {
    Eigen::VectorXd ret(x_next.rows());
    ret.setZero();
    for(int i=0; i < this->constraints.size(); i++)
        ret += this->constraints[i].get_muSTATBp(x_next);

    return ret;
}

void ProjectiveDynamics::step() {
    Eigen::Vector3d gravity;
    gravity << 0., -9.8, 0.;
    Eigen::VectorXd f_ext = this->M * gravity.replicate(this->n_points, 1);
    Eigen::VectorXd s_n = this->x + this->dx + this->h_sq * this->invM * f_ext;
    Eigen::VectorXd x_next = s_n;

    Eigen::VectorXd SABp(x_next.rows());

    for(int i=0; i<this->iter_num; i++) {
        SABp = this->local_solve(x_next);
        x_next = this->global_solve(SABp, s_n);
    }

    for(int i=0; i < this->n_points; i++) {
        if (x_next[3*i + 1] < 0.)
            x_next[3*i + 1] = 0.;
    }

    this->dx = x_next - this->x;
    this->x = x_next;
}

void ProjectiveDynamics::draw() {
    float gray_color[3] = {.6, .6, .6};
    glBegin(GL_QUADS);
    glColor3fv(gray_color);
    glVertex3f(-20., 0., -20.);
    glColor3fv(gray_color);
    glVertex3f(-20., 0., 20.);
    glColor3fv(gray_color);
    glVertex3f(20., 0., 20.);
    glColor3fv(gray_color);
    glVertex3f(20., 0., -20.);
    glEnd();

    glPushMatrix();

    for(Eigen::Index i=0; i < this->cells.rows(); i += 4) {
        break;
        Eigen::Vector4i face = this->cells.segment<4>(i);
        if (face[0] < 13 && face[1] < 13 && face[2] < 13) {
            glBegin(GL_TRIANGLES);
            glColor3f(1., 0., 0.);
            glVertex3f(this->x(3 * face[0]), this->x(3 * face[0] + 1), this->x(3 * face[0] + 2));
            glColor3f(1., 0., 0.);
            glVertex3f(this->x(3 * face[1]), this->x(3 * face[1] + 1), this->x(3 * face[1] + 2));
            glColor3f(1., 0., 0.);
            glVertex3f(this->x(3 * face[2]), this->x(3 * face[2] + 1), this->x(3 * face[2] + 2));
            glEnd();
        }
        if (face[0] < 13 && face[1] < 13 && face[3] < 13) {
            glBegin(GL_TRIANGLES);
            glColor3f(1., 0., 0.);
            glVertex3f(this->x(3 * face[0]), this->x(3 * face[0] + 1), this->x(3 * face[0] + 2));
            glColor3f(1., 0., 0.);
            glVertex3f(this->x(3 * face[1]), this->x(3 * face[1] + 1), this->x(3 * face[1] + 2));
            glColor3f(1., 0., 0.);
            glVertex3f(this->x(3 * face[3]), this->x(3 * face[3] + 1), this->x(3 * face[3] + 2));
            glEnd();
        }
        if (face[0] < 13 && face[3] < 13 && face[2] < 13) {
            glBegin(GL_TRIANGLES);
            glColor3f(1., 0., 0.);
            glVertex3f(this->x(3 * face[0]), this->x(3 * face[0] + 1), this->x(3 * face[0] + 2));
            glColor3f(1., 0., 0.);
            glVertex3f(this->x(3 * face[3]), this->x(3 * face[3] + 1), this->x(3 * face[3] + 2));
            glColor3f(1., 0., 0.);
            glVertex3f(this->x(3 * face[2]), this->x(3 * face[2] + 1), this->x(3 * face[2] + 2));
            glEnd();
        }
        if (face[3] < 13 && face[1] < 13 && face[2] < 13) {
            glBegin(GL_TRIANGLES);
            glColor3f(1., 0., 0.);
            glVertex3f(this->x(3 * face[3]), this->x(3 * face[3] + 1), this->x(3 * face[3] + 2));
            glColor3f(1., 0., 0.);
            glVertex3f(this->x(3 * face[1]), this->x(3 * face[1] + 1), this->x(3 * face[1] + 2));
            glColor3f(1., 0., 0.);
            glVertex3f(this->x(3 * face[2]), this->x(3 * face[2] + 1), this->x(3 * face[2] + 2));
            glEnd();
        }

//        glBegin(GL_LINE_LOOP);
//        glColor3f(0., 0., 0.);
//        glVertex3f(this->x(3 * face[0]), this->x(3 * face[0] + 1), this->x(3 * face[0] + 2));
//        glColor3f(0., 0., 0.);
//        glVertex3f(this->x(3 * face[1]), this->x(3 * face[1] + 1), this->x(3 * face[1] + 2));
//        glColor3f(0., 0., 0.);
//        glVertex3f(this->x(3 * face[2]), this->x(3 * face[2] + 1), this->x(3 * face[2] + 2));
//        glEnd();
//        glBegin(GL_LINE_LOOP);
//        glColor3f(0., 0., 0.);
//        glVertex3f(this->x(3 * face[0]), this->x(3 * face[0] + 1), this->x(3 * face[0] + 2));
//        glColor3f(0., 0., 0.);
//        glVertex3f(this->x(3 * face[1]), this->x(3 * face[1] + 1), this->x(3 * face[1] + 2));
//        glColor3f(0., 0., 0.);
//        glVertex3f(this->x(3 * face[3]), this->x(3 * face[3] + 1), this->x(3 * face[3] + 2));
//        glEnd();
//        glBegin(GL_LINE_LOOP);
//        glColor3f(0., 0., 0.);
//        glVertex3f(this->x(3 * face[0]), this->x(3 * face[0] + 1), this->x(3 * face[0] + 2));
//        glColor3f(0., 0., 0.);
//        glVertex3f(this->x(3 * face[3]), this->x(3 * face[3] + 1), this->x(3 * face[3] + 2));
//        glColor3f(0., 0., 0.);
//        glVertex3f(this->x(3 * face[2]), this->x(3 * face[2] + 1), this->x(3 * face[2] + 2));
//        glEnd();
//        glBegin(GL_LINE_LOOP);
//        glColor3f(0., 0., 0.);
//        glVertex3f(this->x(3 * face[3]), this->x(3 * face[3] + 1), this->x(3 * face[3] + 2));
//        glColor3f(0., 0., 0.);
//        glVertex3f(this->x(3 * face[1]), this->x(3 * face[1] + 1), this->x(3 * face[1] + 2));
//        glColor3f(0., 0., 0.);
//        glVertex3f(this->x(3 * face[2]), this->x(3 * face[2] + 1), this->x(3 * face[2] + 2));
//        glEnd();
    }
    for(Eigen::Index i=0; i < this->faces.rows(); i += 3) {
        Eigen::Vector3i face = this->faces.segment<3>(i);

        glBegin(GL_TRIANGLES);
        glColor3f(1., 0., 0.);
        glVertex3f(this->x(3 * face[0]), this->x(3 * face[0] + 1), this->x(3 * face[0] + 2));
        glColor3f(1., 0., 0.);
        glVertex3f(this->x(3 * face[1]), this->x(3 * face[1] + 1), this->x(3 * face[1] + 2));
        glColor3f(1., 0., 0.);
        glVertex3f(this->x(3 * face[2]), this->x(3 * face[2] + 1), this->x(3 * face[2] + 2));
        glEnd();

        glBegin(GL_LINE_LOOP);
        glColor3f(0., 0., 0.);
        glVertex3f(this->x(3 * face[0]), this->x(3 * face[0] + 1), this->x(3 * face[0] + 2));
        glColor3f(0., 0., 0.);
        glVertex3f(this->x(3 * face[1]), this->x(3 * face[1] + 1), this->x(3 * face[1] + 2));
        glColor3f(0., 0., 0.);
        glVertex3f(this->x(3 * face[2]), this->x(3 * face[2] + 1), this->x(3 * face[2] + 2));
        glEnd();

    }
    glPopMatrix();
}
