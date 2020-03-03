//
// Created by trif on 06/11/2019.
//

#include "ObjLoader.h"
#include "Exception.h"
#include <sstream>
#include <iostream>
#include <vector>

PyMesh::ObjLoader::ObjLoader(const std::string &filename) {
    std::ifstream fin(filename.c_str(), std::ios::in);

    if (!fin.is_open()) {
        std::stringstream err_msg;
        err_msg << "failed to open file \"" << filename << "\"";
        throw PyMesh::IOError(err_msg.str());
    }

    std::string buf;

    while (!fin.eof()) {
        buf.clear();
        fin >> buf;
        if (buf == "v") {
            for(int i=0; i<3; i++) {
                fin >> buf;
                vertex.push_back(atof(buf.c_str()));
            }
        } else if (buf == "f") {
            for(int i=0; i<3; i++) {
                fin >> buf;
                faces.push_back(atoi(buf.c_str())-1);
            }
        } else if (fin.eof()) {
            break;
        } else {
            parse_unknown_field(fin, buf);
        }
    }
    fin.close();


}

void PyMesh::ObjLoader::parse_unknown_field(std::ifstream& fin,
                                    const std::string& fieldname) {
    std::cerr << "Warning: \"" << fieldname << "\" not supported yet.  Ignored." << std::endl;
    std::string endmark = fieldname.substr(0,1) + "End"
                          + fieldname.substr(1,fieldname.size()-1);

    std::string buf;
    while (buf != endmark && !fin.eof()) {
        fin >> buf;
    }
}

