//
// Created by trif on 19/11/2019.
//

#ifndef DEFORMABLEFOOT_MESHLOADER_H
#define DEFORMABLEFOOT_MESHLOADER_H

#include <Eigen/Core>
#include <Eigen/Dense>

namespace PyMesh {

typedef double Float;
typedef Eigen::VectorXd VectorF;
typedef Eigen::VectorXi VectorI;

    class MeshLoader {

    public:
        MeshLoader(const std::string &filename);

    public:
        enum ErrorCode {
            INVALID_FORMAT,
            NOT_IMPLEMENTED
        };
        const VectorF &get_nodes() const { return m_nodes; }
        const VectorI &get_elements() const { return m_elements; }
        const VectorI &get_faces() const { return m_faces; }

        void parse_nodes(const std::string &filename);
        void parse_elements(const std::string &filename);
        void parse_faces(const std::string &filename);

    public:
        bool m_binary;
        size_t m_data_size;
        size_t m_nodes_per_element;
        size_t m_element_type;
        size_t m_nodes_num;
        size_t m_elements_num;
        size_t m_faces_num;
        VectorF m_nodes;
        VectorI m_elements;
        VectorI m_faces;
    };

}

#endif //DEFORMABLEFOOT_MESHLOADER_H
