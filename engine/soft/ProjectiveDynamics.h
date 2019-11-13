//
// Created by trif on 29/10/2019.
//

#ifndef DEFORMABLEFOOT_PROJECTIVEDYNAMICS_H
#define DEFORMABLEFOOT_PROJECTIVEDYNAMICS_H

#include <vector>
#include <Eigen/Dense>
#include "Constraint.h"

class ProjectiveDynamics {
public:
    ProjectiveDynamics(Eigen::VectorXd &points, Eigen::VectorXi &_cells, double _h=0.033333333, int _iter_num=5)
            : x(points), iter_num(_iter_num), h(_h), h_sq(_h * _h), inv_h_sq(1./(_h * _h)) {
        this->cells = _cells;

        this->init();
    }

    ProjectiveDynamics(Eigen::VectorXd &points, Eigen::VectorXi &_cells, Eigen::VectorXi &_faces, double _h=0.033333333, int _iter_num=5)
        : x(points), iter_num(_iter_num), h(_h), h_sq(_h * _h), inv_h_sq(1./(_h * _h)) {
        this->faces = _faces;
        this->cells = _cells;
        this->n_points = points.rows() / 3;
        this->init();
    }
    
    void init();
    int get_total_dof();
    Eigen::Vector3d get_node(int node_idx);
    int add_position_constraint(int point_idx);
    void update_invM_L();
    
    Eigen::VectorXd global_solve(Eigen::VectorXd &SABp, Eigen::VectorXd &s_n);
    Eigen::VectorXd local_solve(Eigen::VectorXd &x_next);

    void step();
    void draw();

public:
    Eigen::VectorXd &x;
    Eigen::VectorXd dx;
    Eigen::VectorXi cells;
    Eigen::VectorXi faces;

    int iter_num;
    double h;
    double h_sq;
    double inv_h_sq;

    Eigen::MatrixXd M;
    Eigen::MatrixXd M_h_sq;
    Eigen::MatrixXd invM;
    Eigen::MatrixXd L;
    Eigen::MatrixXd M_L;
    Eigen::MatrixXd invM_L;

    int n_points;
    std::vector< CorotateConstraint > constraints;
    std::vector< int > constrained_idx;
};


#endif //DEFORMABLEFOOT_PROJECTIVEDYNAMICS_H
