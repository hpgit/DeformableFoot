//
// Created by trif on 06/11/2019.
//

#ifndef DEFORMABLEFOOT_OBJLOADER_H
#define DEFORMABLEFOOT_OBJLOADER_H

#include <Eigen/Dense>
#include <fstream>
#include <vector>

namespace PyMesh {

    class ObjLoader {
    public:
        ObjLoader(const std::string &filename);

        std::vector<double> vertex;
        std::vector<int> faces;
    private:
        static void parse_unknown_field(std::ifstream &fin,
                                        const std::string &fieldname);
    };

}

#endif //DEFORMABLEFOOT_OBJLOADER_H
