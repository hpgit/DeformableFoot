//
// Created by trif on 05/11/2019.
//

#include "engine/rigid/RigidWorld.h"
#include "pymesh/MshLoader.h"
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

//#include <flann/flann.hpp>


RigidWorld world(1./30.);

Camera camera;
int last_x, last_y;
bool mouse_downed = false;

int mouse_button = 0;
 

void init() {
    Eigen::Affine3d body0_transform;
    body0_transform.translation() = Eigen::Vector3d(-1., 2., 0.);
    body0_transform.linear().setIdentity();
    Eigen::Affine3d body1_transform;
    body1_transform.translation() = Eigen::Vector3d(1., 2., 0.);
    body1_transform.linear().setIdentity();
    Eigen::Affine3d body2_transform;
    body2_transform.translation() = Eigen::Vector3d(3., 2., 0.);
    body2_transform.linear().setIdentity();
    Eigen::Affine3d body3_transform;
    body3_transform.translation() = Eigen::Vector3d(1., 0., 0.);
    body3_transform.linear().setIdentity();
    body3_transform.linear()(0, 0) = 0.;
    body3_transform.linear()(0, 1) = 1.;
    body3_transform.linear()(1, 0) = -1.;
    body3_transform.linear()(1, 1) = 0.;

//    std::cout << body3_transform.matrix() << std::endl;

//    body3_transform.rotate(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));

    Body *body0 = new Body("body0", body0_transform);
    Body *body1 = new Body("body1", body1_transform);
    Body *body2 = new Body("body2", body2_transform);
    Body *body3 = new Body("body3", body3_transform);

    world.AddBody(body0);
    world.AddBody(body1);
    world.AddBody(body2);
    world.AddBody(body3);

    Eigen::Affine3d joint0_body_p_transform, joint0_body_c_transform;
    joint0_body_p_transform.translation() = Eigen::Vector3d(1., 0., 0.);
    joint0_body_p_transform.linear().setIdentity();
    joint0_body_c_transform.translation() = Eigen::Vector3d(-1., 0., 0.);
    joint0_body_c_transform.linear().setIdentity();

    Eigen::Affine3d joint1_body_p_transform, joint1_body_c_transform;
    joint1_body_p_transform.translation() = Eigen::Vector3d(0., -.5, 0.);
    joint1_body_p_transform.linear().setIdentity();
    joint1_body_c_transform.translation() = Eigen::Vector3d(-1., 0., 0.);
    joint1_body_c_transform.linear().setIdentity();


    BJoint *joint0 = new BJoint(body0, joint0_body_p_transform, body1, joint0_body_c_transform);
    BJoint *joint1 = new BJoint(body1, joint0_body_p_transform, body2, joint0_body_c_transform);
    BJoint *joint2 = new BJoint(body1, joint1_body_p_transform, body3, joint1_body_c_transform);

    world.AddJoint(joint0);
    world.AddJoint(joint1);
    world.AddJoint(joint2);

    body1->v_g(1) = -1.;
    body2->v_g(2) = -10.;

    world.SetGravity(Eigen::Vector3d::Zero());
}


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
//    glEnable(GL_DEPTH_TEST);
//    gluLookAt(0., 0., -1., 0., 0., 0., 0., 1., 0.);
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
    camera.transform();
//    glScaled(20., 20., 20.);
    world.Draw();
    glutSwapBuffers();
}

void keyboard(unsigned char c, int x, int y) {
    if (c == 27) {
        // escape
        exit(EXIT_SUCCESS);
    }
    else if ( c == 32) {
        // space
        world.Step();
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
    world.Step();
//    std::cout << world.GetKineticEnergy()<< std::endl;
//    std::cout << world.GetPotentialEnergy() << std::endl;
//    std::cout << world.GetKineticEnergy() + world.GetPotentialEnergy() << std::endl;

    glutPostRedisplay();
    glutTimerFunc(33, timer, 0);
}

int main(int argc, char **argv) {
    init();
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
    glutInitWindowSize(1024, 768);
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

