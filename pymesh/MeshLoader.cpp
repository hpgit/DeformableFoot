//
// Created by trif on 19/11/2019.
//

#include "MeshLoader.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <regex>

#include "Exception.h"

namespace PyMesh {

    MeshLoader::MeshLoader(const std::string &filename) {
        std::string filename_node(filename);
        std::string filename_ele(filename);
        std::string filename_face(filename);
        filename_node.append(".1.node");
        filename_ele.append(".1.ele");
        filename_face.append(".1.face");

        parse_nodes(filename_node);
        parse_elements(filename_ele);
        parse_faces(filename_face);
    }

    void MeshLoader::parse_nodes(const std::string &filename) {
        std::ifstream fin(filename.c_str(), std::ios::in);

        if (!fin.is_open()) {
            std::stringstream err_msg;
            err_msg << "failed to open file \"" << filename << "\"";
            throw IOError(err_msg.str());
        }

        // Parse nodes
        std::string buf;
        int dim;
        double data;

        fin >> this->m_nodes_num >> dim >> buf >> buf;


        this->m_nodes = Eigen::VectorXd(this->m_nodes_num * dim);

        for (size_t i = 0; i < this->m_nodes_num; i++) {
            for (int j = 0; j < dim + 1; j++) {
                fin >> data;
                if (j > 0) this->m_nodes[i * dim + j-1] = data;
            }
        }
        fin.close();
    }

    void MeshLoader::parse_elements(const std::string &filename) {
        std::ifstream fin(filename.c_str(), std::ios::in);

        if (!fin.is_open()) {
            std::stringstream err_msg;
            err_msg << "failed to open file \"" << filename << "\"";
            throw IOError(err_msg.str());
        }

        // Parse nodes
        std::string buf;
        int dim;
        int data;

        fin >> this->m_elements_num >> buf >> buf;

        this->m_elements = Eigen::VectorXi(this->m_elements_num * 4);

        for (size_t i = 0; i < this->m_elements_num; i++) {
            for (int j = 0; j < 5; j++) {
                fin >> data;
                if (j > 0) this->m_elements[4*i + j-1] = data;
            }
        }
        fin.close();
    }

    void MeshLoader::parse_faces(const std::string &filename) {
        std::ifstream fin(filename.c_str(), std::ios::in);

        if (!fin.is_open()) {
            std::stringstream err_msg;
            err_msg << "failed to open file \"" << filename << "\"";
            throw IOError(err_msg.str());
        }

        // Parse nodes
        std::string buf;
        int data;

        fin >> this->m_faces_num >> buf;

        this->m_faces = Eigen::VectorXi(this->m_faces_num * 3);

        for (size_t i = 0; i < this->m_faces_num; i++) {
            for (int j = 0; j < 5; j++) {
                fin >> data;
                if (j > 0) this->m_faces[3*i + j-1] = data;
            }
        }
        fin.close();

    }


}
