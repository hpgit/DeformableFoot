//
// Created by Hwangpil Park on 04/03/2020.
//

#ifndef DEFORMABLEFOOT_MIXEDWORLD_H
#define DEFORMABLEFOOT_MIXEDWORLD_H

#include "../rigid/RigidWorld.h"
#include "../soft/ProjectiveDynamics.h"
#include <vector>
#include <tuple>
#include <Eigen/Dense>

class MixedWorld {
public:
    MixedWorld(Eigen::VectorXd &_points, Eigen::VectorXi &_cells, Eigen::VectorXi &_faces, double _h, int _iter_num);
    void assign_node_to_body(int body_idx, int node_idx);
    void update_B_hat();
    void step();
    void draw();

    RigidWorld *rigid_world;
    ProjectiveDynamics *soft_world;
    int n_points;
    int iter_num;

    double h;
    double h_sq;
    double invh;
    double invh_sq;

    std::vector< std::tuple<int, int, Eigen::Vector3d> > connection;
    std::vector< std::tuple<int, Eigen::Vector3d> > connection_for_nodes;

    Eigen::MatrixXd B_hat;
    Eigen::VectorXd b_hat;

    std::vector<int> pure_soft_indices;
};


#endif //DEFORMABLEFOOT_MIXEDWORLD_H
