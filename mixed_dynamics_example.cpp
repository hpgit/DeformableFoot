//
// Created by Hwangpil Park on 04/03/2020.
//

//
// Created by trif on 05/11/2019.
//

#include "engine/mixed/MixedWorld.h"
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


MixedWorld *mw;

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
    mw->draw();
    glutSwapBuffers();
}

void keyboard(unsigned char c, int x, int y) {
    if (c == 27) {
        // escape
        delete mw;
        exit(EXIT_SUCCESS);
    }
    else if ( c == 32) {
        // space
        mw->step();
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
    mw->step();

    glutPostRedisplay();
    glutTimerFunc(33, timer, 0);
}


void init_rigid_world(RigidWorld *_world) {
    Eigen::Affine3d body0_transform, body1_transform;
    body0_transform.translation() = Eigen::Vector3d(-1., 2., 0.);
    body0_transform.linear().setIdentity();
    body1_transform.translation() = Eigen::Vector3d(1., 2., 0.);
    body1_transform.linear().setIdentity();
    Body* body0 = new Body("body0", body0_transform, 10.);
    Body* body1 = new Body("body1", body1_transform, 10.);

    _world->AddBody(body0);
    _world->AddBody(body1);

    Eigen::Affine3d joint0_body_p_transform, joint0_body_c_transform;
    joint0_body_p_transform.translation() = Eigen::Vector3d(1., 0., 0.);
    joint0_body_p_transform.linear().setIdentity();
    joint0_body_c_transform.translation() = Eigen::Vector3d(-1., 0., 0.);
    joint0_body_c_transform.linear().setIdentity();

    BJoint *joint0 = new BJoint(body0, joint0_body_p_transform, body1, joint0_body_c_transform);

    _world->AddJoint(joint0);

//    body0->v_g[2] = -1.;
}


void init_mixed_world(MixedWorld *_world) {
    for (auto i=0; i<_world->soft_world->n_points ; i++) {
        if(_world->soft_world->x[3 * i + 1] > 1.5) {
            if(_world->soft_world->x[3 * i] < -0.249)
                _world->assign_node_to_body(0, i);
            if(_world->soft_world->x[3 * i] > 0.249)
                _world->assign_node_to_body(1, i);
        }
    }
}



int main(int argc, char **argv) {
    // PyMesh::MshLoader mshloader("../data/cube_.msh");
    PyMesh::MeshLoader mshloader("../data/test_cube_long");
    Eigen::VectorXd nodes = mshloader.get_nodes();
    Eigen::VectorXi elems = mshloader.get_elements();
    Eigen::VectorXi faces = mshloader.get_faces();

    mw = new MixedWorld(nodes, elems, faces, 1./30., 5);
    init_rigid_world(mw->rigid_world);
    init_mixed_world(mw);
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
    glutInitWindowSize(1280, 720);
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

