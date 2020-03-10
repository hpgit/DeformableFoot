//
// Created by Hwangpil Park on 04/03/2020.
//

#include "MixedWorld.h"


MixedWorld::MixedWorld(Eigen::VectorXd &_points, Eigen::VectorXi &_cells, Eigen::VectorXi &_faces, double _h, int _iter_num)
: iter_num(_iter_num), h(_h), h_sq(_h*_h), invh(1./_h), invh_sq(1./(_h*_h))
{
    this->rigid_world = new RigidWorld(_h);
    this->soft_world = new ProjectiveDynamics(_points, _cells, _faces, _h, _iter_num);

    this->n_points = this->soft_world->n_points;

    this->connection.clear();
    this->connection_for_nodes.clear();
    for(std::vector<std::tuple<int, Eigen::Vector3d> >::size_type i=0; i < this->n_points; i++)
        this->connection_for_nodes.emplace_back(std::make_tuple(-1, Eigen::Vector3d::Zero()));

    this->B_hat = Eigen::MatrixXd::Zero(0, 0);
    this->b_hat = Eigen::VectorXd::Zero(0);

    this->pure_soft_indices.clear();
    for(std::vector<int>::size_type i=0; i < 3 * this->n_points; i++)
        this->pure_soft_indices.push_back(i);
}

void MixedWorld::assign_node_to_body(int body_idx, int node_idx) {
    auto body = this->rigid_world->GetBody(body_idx);
    auto node_position = this->soft_world->get_node(node_idx);
    auto body_to_point_t = body->Transform().inverse() * node_position;
    // constraint_idx = this->soft_world.add_position_constraint(node_idx)
    this->connection.emplace_back(std::make_tuple(body_idx, node_idx, body_to_point_t));
    std::get<0>(this->connection_for_nodes[node_idx]) = body_idx;
    std::get<1>(this->connection_for_nodes[node_idx]) = body_to_point_t;

    for (int i = 0; i < 3; i++) {
        pure_soft_indices.erase(
               std::remove(pure_soft_indices.begin(), pure_soft_indices.end(), 3 * node_idx + i),
               pure_soft_indices.end()
               );
    }
}

void MixedWorld::update_B_hat(){
    int soft_all_dof = 3 * this->n_points;
    int soft_dof = this->pure_soft_indices.size();
    int rigid_dof = 6 * this->rigid_world->GetNumBody();
    int rigid_constraint_dof = this->rigid_world->GetConstraintNum();
    int mixed_constraint_dof = rigid_dof + rigid_constraint_dof;

    this->B_hat = Eigen::MatrixXd::Zero(soft_all_dof, soft_dof + rigid_dof+rigid_constraint_dof);
    this->b_hat = Eigen::VectorXd::Zero(soft_all_dof);

    for(int i=0;i < this->pure_soft_indices.size(); i++){
        this->B_hat(this->pure_soft_indices[i], i) = 1.;
    }

    int body_idx, node_idx;
    Eigen::Vector3d body_to_point_t;
    for(auto &_connection : this->connection) {
        std::tie(body_idx, node_idx, body_to_point_t) = _connection;
        auto body = this->rigid_world->GetBody(body_idx);
        B_hat.block(3*node_idx, soft_dof + 6*body_idx, 3, 6)
            = this->h * body->GetLinearJacobian(body_to_point_t);
        b_hat.segment(3*node_idx, 3) = body->ToWorld(body_to_point_t);
    }
}

void MixedWorld::step() {
    int soft_all_dof = 3 * this->n_points;
    int soft_dof = this->pure_soft_indices.size();
    int rigid_dof = 6 * this->rigid_world->GetNumBody();
    int rigid_constraint_dof = this->rigid_world->GetConstraintNum();

    int mixed_dof = soft_dof + rigid_dof + rigid_constraint_dof;
    int mixed_constraint_dof = rigid_dof + rigid_constraint_dof;

    int total_dof = mixed_dof + mixed_constraint_dof;

    this->update_B_hat();

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(total_dof, total_dof);
    Eigen::VectorXd b = Eigen::VectorXd::Zero(total_dof);

    Eigen::MatrixXd A_rigid(rigid_dof + rigid_constraint_dof, rigid_dof + rigid_constraint_dof);
    Eigen::VectorXd b_rigid(rigid_dof + rigid_constraint_dof);
    this->rigid_world->GetSystemMatrix(A_rigid, b_rigid);
    Eigen::VectorXd v_rigid_next = A_rigid.ldlt().solve(b_rigid).head(rigid_dof);

    Eigen::MatrixXd A_mixed = this->B_hat.transpose() * (this->soft_world->M_L * this->B_hat);

    A.topLeftCorner(mixed_dof, mixed_dof) = A_mixed;
    A.block(mixed_dof, soft_dof, total_dof - mixed_dof, mixed_dof - soft_dof) = A_rigid;
    Eigen::MatrixXd BTLB = B_hat.transpose() * (this->soft_world->L * B_hat);
    A.block(mixed_dof, 0, rigid_dof, mixed_dof) += this->h * BTLB.block(soft_dof, 0, rigid_dof, mixed_dof);
    A.topRightCorner(mixed_dof, total_dof - mixed_dof) = A.bottomLeftCorner(total_dof - mixed_dof, mixed_dof).transpose();

    Eigen::VectorXd f_ext = this->soft_world->M * this->rigid_world->gravity.colwise().replicate(this->n_points);
    Eigen::VectorXd s_n = this->soft_world->x + this->soft_world->dx + this->h_sq * (this->soft_world->invM * f_ext);

    Eigen::VectorXd pure_soft_s_n = Eigen::VectorXd::Zero(this->pure_soft_indices.size());
    for (auto i = 0; i < this->pure_soft_indices.size(); i++) {
        pure_soft_s_n[i] = s_n[this->pure_soft_indices[i]];
    }
    Eigen::VectorXd temp_vec = Eigen::VectorXd::Zero(pure_soft_s_n.rows() + v_rigid_next.rows() + rigid_constraint_dof);
    temp_vec.head(pure_soft_indices.size()) = pure_soft_s_n;
    temp_vec.segment(pure_soft_indices.size(), v_rigid_next.rows()) = v_rigid_next;
    Eigen::VectorXd x_next = this->B_hat * temp_vec + this->b_hat;
    Eigen::VectorXd _x_next;

    for (auto i=0; i < this->iter_num; i++) {
        // for rigid
        b.tail(total_dof - mixed_dof) = b_rigid;

        // for soft
        b.head(mixed_dof) = this->B_hat.transpose() *
                (
                    this->soft_world->M_h_sq * (s_n - this->b_hat)
                    - this->soft_world->L * this->b_hat
                );

        b.segment(mixed_dof, rigid_dof) += -this->h *
                (
                    this->B_hat.transpose().block(soft_dof, 0, rigid_dof, soft_all_dof) * (this->soft_world->L * this->b_hat)
                );

        // for soft local solve
        Eigen::MatrixXd muSTATBp;
        for (auto &constraint : this->soft_world->constraints) {
            muSTATBp = constraint.get_muSTATBp(x_next);
            b.head(mixed_dof) += this->B_hat.transpose() * muSTATBp;
            b.segment(mixed_dof, rigid_dof) += this->h * (this->B_hat.transpose().block(soft_dof, 0, rigid_dof, soft_all_dof) * muSTATBp);
        }
        _x_next = A.ldlt().solve(b);
        x_next = this->B_hat * _x_next.head(mixed_dof) + this->b_hat;
    }

    for (auto i = 1; i < x_next.rows(); i += 3) {
        if (x_next[i] < 0.)
            x_next[i] = 0.;
    }

    this->rigid_world->IntegratePosition(_x_next.segment(soft_dof, rigid_dof));

    int body_idx, node_idx;
    Eigen::Vector3d body_to_point_t;
    for (auto &_connection : this->connection) {
        std::tie(body_idx, node_idx, body_to_point_t) = _connection;
        auto body = this->rigid_world->GetBody(body_idx);
        x_next.segment(3*node_idx, 3) = body->ToWorld(body_to_point_t);
    }

    this->soft_world->dx = x_next - this->soft_world->x;
    this->soft_world->x = x_next;
}


void MixedWorld::draw() {
    this->soft_world->draw();
    this->rigid_world->Draw();
    // this->update_B_hat()
}
