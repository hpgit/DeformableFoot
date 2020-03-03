//
// Created by trif on 05/11/2019.
//

#include "engine/soft/ProjectiveDynamics.h"
#include "pymesh/MshLoader.h"
#include "pymesh/MeshLoader.h"
#include "pymesh/ObjLoader.h"
#include <Eigen/Dense>
#include <iostream>

#ifndef GL_SILENCE_DEPRECATION
#define GL_SILENCE_DEPRECATION
#endif

#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include "render/Camera.h"

#include <flann/flann.hpp>


ProjectiveDynamics *proj;

Camera camera;
int last_x, last_y;
bool mouse_downed = false;

int mouse_button = 0;

void
reshape(int w, int h)
{
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glFrustum(-.08, .08, -.045, .045, 0.1, 1000.);
    glMatrixMode(GL_MODELVIEW);
}

void
display()
{
    glLoadIdentity();

//    gluLookAt(0., 0., -1., 0., 0., 0., 0., 1., 0.);
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    camera.transform();
//    glScaled(20., 20., 20.);
    proj->draw();
    glutSwapBuffers();
}

void keyboard(unsigned char c, int x, int y) {
    if (c == 27) {
        // escape
        delete proj;
        exit(EXIT_SUCCESS);
    }
    else if ( c == 32) {
        // space
        proj->step();
        glutPostRedisplay();
    }
    else if( c == 93) {
        // ]
        camera.distance -= 0.1;
        glutPostRedisplay();
    }
    else if( c == 91) {
        // [
        camera.distance += 0.1;
        glutPostRedisplay();
    }
}

void mouse(int button, int state, int x, int y) {
    if (state == GLUT_DOWN) {
        last_x = x;
        last_y = y;
        mouse_downed = true;
        mouse_button = button;
    }
    else {
        last_x = x;
        last_y = y;
        mouse_button = 0;
    }
}

void mouse_motion(int x, int y) {
    int dx = x - last_x;
    int dy = y - last_y;

    if (mouse_button == GLUT_LEFT_BUTTON) {
        camera.rotateY -= dx / 45. * M_PI;
        camera.rotateX -= dy / 45. * M_PI;
        glutPostRedisplay();
    }
    else if (mouse_button == GLUT_RIGHT_BUTTON) {
        camera.distance -= dy/100.;
        glutPostRedisplay();
    }

    last_x = x;
    last_y = y;
}

void timer(int value) {
    proj->step();

    glutPostRedisplay();
    glutTimerFunc(33, timer, 0);
}

int main(int argc, char **argv) {
//    PyMesh::MshLoader mshloader("../data/cube_.msh");
    PyMesh::MeshLoader mshloader("../data/cube");
    Eigen::VectorXd nodes = mshloader.get_nodes();
    Eigen::VectorXi elems = mshloader.get_elements();
    Eigen::VectorXi faces = mshloader.get_faces();

//    std::cout << nodes << std::endl;
//    std::cout << elems << std::endl;
    std::cout << faces << std::endl;

//    PyMesh::ObjLoader objloader("../data/cube__sf.obj");
//
//    int nn = 1;
//
//    flann::Matrix<double> dataset(nodes.data(), nodes.rows()/3, 3);
//    flann::Matrix<double> query(objloader.vertex.data(), objloader.vertex.size()/3, 3);
//
//    flann::Matrix<int> indices(new int[query.rows*nn], query.rows, nn);
//    flann::Matrix<double> dists(new double[query.rows*nn], query.rows, nn);
//
//    // construct an randomized kd-tree index using 4 kd-trees
//    flann::Index<flann::L2<double> > index(dataset, flann::KDTreeIndexParams(4));
//    index.buildIndex();
//
//    // do a knn search, using 128 checks
//    index.knnSearch(query, indices, dists, nn, flann::SearchParams(128));
//
//    for(int i=0; i<indices.rows; i++) {
//        std::cout << *indices[i] << " " << *dists[i]<< std::endl;
//    }
//
//    Eigen::VectorXi faces(objloader.faces.size());
//    for(auto i=0; i < objloader.faces.size(); i++)
//        faces(i) = *indices[objloader.faces[i]];

//    Eigen::VectorXd nodes(12);
//    Eigen::VectorXi elems(4);
//    Eigen::VectorXi faces(12);
//    nodes << 1., 2., 0., 0., 3., 0., 0., 2., 1., 0., 2., 0.;
//    elems << 0, 1, 2, 3;
//    faces << 0, 1, 2, 0, 1, 3, 0, 2, 3, 1, 2, 3;

//    ProjectiveDynamics proj(nodes, elems);
    proj = new ProjectiveDynamics(nodes, elems, faces);
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
    glutInitWindowSize(1920, 1080);
    glutCreateWindow("single triangle");
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);
    glutMouseFunc(mouse);
    glutMotionFunc(mouse_motion);
    glutTimerFunc(33, timer, 0);
    glutMainLoop();

    return 0;
}

