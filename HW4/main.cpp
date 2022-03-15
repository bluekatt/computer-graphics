//
//  main.cpp
//  Computer Graphics
//
//  Created by 박종석 on 2021/09/01.
//

#include <fstream>
#include <sstream>
#include <iostream>
#include <cmath>
#include <vector>
#include "vec3.hpp"
#include "BSPTree.hpp"
#ifdef __APPLE__
/* Defined before OpenGL and GLUT includes to avoid deprecation messages */
#define GL_SILENCE_DEPRECATION
#include <OpenGL/gl.h>
#include <GLUT/glut.h>
#else
#include <GL/gl.h>
#include <GL/glut.h>
#endif

using namespace std;

static const int MAX_OBJS = 10;

static int tResolution = 10; // Resolution of trajectory
static int sResolution = 10; // Resolution of cross section

static bool useBSpline = false;

static int ncs = 0; // the number of cross sections
static int ncp = 0; // the number of control points

static unsigned int width = 1000;
static unsigned int height = 1000;

static bool mouseRotatePressed = false;
static bool mouseMovePressed = false;

static bool keyboardSPressed = false;

static float trackBallRadius = 750;
static float quat[4] = { 1, 0, 0, 0 };
static float lastQuat[4] = { 1, 0, 0, 0 };
static float trans[3] = { 0, 0, 0 };
static int lastX = 0;
static int lastY = 0;
static float viewAngle = 45;

static bool fullScreen = false;

float maxCamDistance = 500.0;
float minCamDistacne = 10.0;
float maxViewAngle = 170.0;
float minViewAngle = 1.0;
bool leftButton = false;
GLfloat mousePosX, mousePosY;

float timeStep = 1000.0 / 30.0;

float cam[3] = { 0, 0.0, 180.0 };
float ori[3] = { 0.0, 0.0, 0.0 };
float rot[3] = { 0.0, 1.0, 0.0 };

float* controlPoints;
float* scalingFactors;
float* rotations;
float* positions;

int numElement;
GLuint* element;
int numSurface;
float* surface;
float* normal;

int numObjs = 0;
vector<vector<float> > objVertices[MAX_OBJS];
vector<vector<float> > objNormals[MAX_OBJS];

int numControlCurve;
float* controlCurve;
int numScalingCurve;
float* scalingCurve;
int numRotationCurve;
float* rotationCurve;
int numPositionCurve;
float* positionCurve;

BSPTree bspTree;
vector<vector<float> > traversed;

float length(float a[], int n) {
    float ret = 0;
    for(int i=0;i<n;i++) {
        ret += a[i] * a[i];
    }
    return sqrt(ret);
}

void crossProduct(float a[3], float b[3], float c[3]) {
    c[0] = a[1] * b[2] - a[2] * b[1];
    c[1] = a[2] * b[0] - a[0] * b[2];
    c[2] = a[0] * b[1] - a[1] * b[0];
}

float dotProduct(float a[3], float b[3]) {
    float ret = 0;
    for(int i=0;i<3;i++) {
        ret += a[i] * b[i];
    }
    return ret;
}

void resize(float a[], int n, float newLen) {
    float len = length(cam, n);
    for(int i=0;i<n;i++) {
        a[i] = a[i] / len * newLen;
    }
}

void neg(float a[], int n) {
    for(int i=0;i<n;i++) {
        a[i] = -a[i];
    }
}

void normalize(float a[], int n) {
    float len = length(a, n);
    if(len<0.00001) return;
    for(int i=0;i<n;i++) {
        a[i] = a[i] / len;
    }
}

void clear(float a[], int n) {
    for(int i=0;i<n;i++) {
        a[i] = 0;
    }
}

//(w1w2 −v1 ·v2,w1v2 +w2v1 +v1 ×v2)
void quatMult(float a[4], float b[4], float c[4]) {
    float w1 = a[0], w2 = b[0];
    float v1[3] = { a[1], a[2], a[3] };
    float v2[3] = { b[1], b[2], b[3] };
    float cross[3];
    crossProduct(v1, v2, cross);
    
    cross[0] += w1 * v2[0] + w2 * v1[0];
    cross[1] += w1 * v2[1] + w2 * v1[1];
    cross[2] += w1 * v2[2] + w2 * v1[2];
    
    c[0] = w1 * w2 - dotProduct(v1, v2);
    c[1] = cross[0];
    c[2] = cross[1];
    c[3] = cross[2];
}

void rotate(float q[4], float v[3]) {
    float p[4] = { 0, v[0], v[1], v[2] };
    float invQ[4] = { q[0], -q[1], -q[2], -q[3] };
    float qp[4];
    
    float result[4];
    
    quatMult(q, p, qp);
    quatMult(qp, invQ, result);
    v[0] = result[1]; v[1] = result[2]; v[2] = result[3];
}

void projectPointToSphere(float x, float y, float v[]) {
    v[0] = x;
    v[1] = y;
    v[2] = sqrt(trackBallRadius * trackBallRadius - x*x - y*y);
}

void copyQuat(float a[], float b[]) {
    for(int i=0;i<4;i++) {
        a[i] = b[i];
    }
}

void printV(float a[], int n) {
    for(int i=0;i<n;i++) {
        std::cout << a[i] << " ";
    }
    std::cout << "\n";
}

void expMap(float v[3], float q[4]) {
    float norm = length(v, 3);
    if(norm==0) {
        q[0] = 1;
        q[1] = 0; q[2] = 0; q[3] = 0;
    } else {
        float s = sin(norm) / norm;
        q[0] = cos(norm);
        q[1] = s * v[0];
        q[2] = s * v[1];
        q[3] = s * v[2];
    }
}

void logMap(float q[4], float v[3]) {
    float w = q[0];
    float norm = length(q+1, 3);
    if(w>=1 || norm == 0) {
        clear(v, 3);
        return;
    }
    
    float s = M_PI/2.0;
    if(w==0) {
        v[0] = s * q[1]; v[1] = s * q[2]; v[2] = s * q[3];
        return;
    }
    
    s = atan2(norm, w) / norm;
    v[0] = s * q[1]; v[1] = s * q[2]; v[2] = s * q[3];
}

void quatPow(float q[4], float t, float c[4]) {
    float v[3];
    logMap(q, v);
    v[0] *= t; v[1] *= t; v[2] *= t;
    expMap(v, c);
}

void slerp(float q1[4], float q2[4], float t, float c[4]) {
    float invQ1[4] = { q1[0], -q1[1], -q1[2], -q1[3] };
    
    float mq[4];
    quatMult(invQ1, q2, mq);
    float rq[4];
    quatPow(mq, t, rq);
    quatMult(q1, rq, c);
}

void drawModel() {
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_NORMAL_ARRAY);
    
    GLUquadricObj* qobj;
    float defaultDif[4] = { 0.8, 0.8, 0.8, 1.0 };
    float defaultAmb[4] = { 0.2, 0.2, 0.2, 1.0 };
    float defaultEmm[4] = { 0.0, 0.0, 0.0, 1.0 };
    float defaultSpec[4] = { 0.0, 0.0, 0.0, 1.0 };
    
    float color0[] = { 111.0/255.0, 79.0/255.0, 40.0/255.0, 1.0 };
    float spec0[4] = { 0.5, 0.5, 0.5, 1.0 };
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color0);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spec0);
    glMateriali(GL_FRONT_AND_BACK, GL_SHININESS, 8);
    glPushName(0); // Used for picking
    glPushMatrix();
    {
        for(int i=0;i<objVertices[0].size();i++) {
            int size = (int)objVertices[0][i].size();
            float* vertices = &objVertices[0][i][0];
            float* normals = &objNormals[0][i][0];
            glVertexPointer(3, GL_FLOAT, 0, vertices);
            glNormalPointer(GL_FLOAT, 0, normals);
            glDrawArrays(GL_POLYGON, 0, size/3);
        }
    }
    glPopMatrix();
    glPopName();
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, defaultSpec);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, defaultAmb);
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, defaultDif);
    glMateriali(GL_FRONT_AND_BACK, GL_SHININESS, 0);
    
    float color2[] = { 0.7, 0.7, 0.7, 1.0 };
    float spec2[4] = { 1.0, 1.0, 1.0, 1.0 };
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, color2);
    glMaterialfv(GL_FRONT, GL_SPECULAR, spec2);
    glMateriali(GL_FRONT, GL_SHININESS, 30);
    glPushName(2);
    glPushMatrix();
    {
        for(int i=0;i<objVertices[2].size();i++) {
            int size = (int)objVertices[2][i].size();
            float* vertices = &objVertices[2][i][0];
            float* normals = &objNormals[2][i][0];
            glVertexPointer(3, GL_FLOAT, 0, vertices);
            glNormalPointer(GL_FLOAT, 0, normals);
            glDrawArrays(GL_POLYGON, 0, size/3);
        }
    }
    glPopMatrix();
    glPopName();
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, defaultSpec);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, defaultAmb);
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, defaultDif);
    glMateriali(GL_FRONT_AND_BACK, GL_SHININESS, 0);
    
    float color10[] = { 1.0, 1.0, 1.0, 1.0 };
    float spec10[4] = { 1.0, 1.0, 1.0, 1.0 };
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, color10);
    glMaterialfv(GL_FRONT, GL_SPECULAR, spec10);
    glMateriali(GL_FRONT, GL_SHININESS, 100);
    glPushName(2);
    glPushMatrix();
    {
        for(int i=0;i<objVertices[5].size();i++) {
            int size = (int)objVertices[5][i].size();
            float* vertices = &objVertices[5][i][0];
            float* normals = &objNormals[5][i][0];
            glVertexPointer(3, GL_FLOAT, 0, vertices);
            glNormalPointer(GL_FLOAT, 0, normals);
            glDrawArrays(GL_POLYGON, 0, size/3);
        }
    }
    glPopMatrix();
    glPopName();
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, defaultSpec);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, defaultAmb);
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, defaultDif);
    glMateriali(GL_FRONT_AND_BACK, GL_SHININESS, 0);
    
    float color4[] = { 232.0/255.0, 169.0/255.0, 53.0/255.0, 1.0 };
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color4);
    glPushName(4);
    glPushMatrix();
    {
        glTranslated(35, 31, 15);
        qobj = gluNewQuadric();
        gluQuadricDrawStyle(qobj, GLU_FILL);
        gluSphere(qobj, 8, 100, 100);
    }
    glPopMatrix();
    glPopName();
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, defaultSpec);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, defaultAmb);
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, defaultDif);
    glMateriali(GL_FRONT_AND_BACK, GL_SHININESS, 0);
    
    float color5[] = { 12.0/255.0, 116.0/255.0, 232.0/255.0, 1.0 };
    glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, color5);
    glPushName(5);
    glPushMatrix();
    {
        glTranslated(25, 31, -5);
        qobj = gluNewQuadric();
        gluQuadricDrawStyle(qobj, GLU_FILL);
        gluSphere(qobj, 8, 100, 100);
    }
    glPopMatrix();
    glPopName();
    glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, defaultEmm);
    
    float amb6[] = { 232.0/255.0*0.53, 70.0/255.0*0.53, 53.0/255.0*0.53, 1.0 };
    float color6[] = { 232.0/255.0, 70.0/255.0, 53.0/255.0, 1.0 };
    float spec6[4] = { 1.0, 1.0, 1.0, 1.0 };
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb6);
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, color6);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spec6);
    glMateriali(GL_FRONT_AND_BACK, GL_SHININESS, 80);
    glPushName(6);
    glPushMatrix();
    {
        glTranslated(15, 31, 15);
        qobj = gluNewQuadric();
        gluQuadricDrawStyle(qobj, GLU_FILL);
        gluSphere(qobj, 8, 100, 100);
    }
    glPopMatrix();
    glPopName();
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, defaultSpec);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, defaultAmb);
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, defaultDif);
    glMateriali(GL_FRONT_AND_BACK, GL_SHININESS, 0);
    
    glColor3f(1, 1, 1);
    float color7[] = { 1.0, 1.0, 1.0, 1.0 };
    float spec7[] = { 0.7, 0.7, 0.7, 1.0 };
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color7);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spec7);
    glMateriali(GL_FRONT_AND_BACK, GL_SHININESS, 100);
    glPushName(7);
    glPushMatrix();
    {
        glTranslated(25, 31, 35);
        qobj = gluNewQuadric();
        gluQuadricDrawStyle(qobj, GLU_FILL);
        gluSphere(qobj, 8, 100, 100);
    }
    glPopMatrix();
    glPopName();
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, defaultSpec);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, defaultAmb);
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, defaultDif);
    glMateriali(GL_FRONT_AND_BACK, GL_SHININESS, 0);
    
    
    float color3[] = { 1.0, 1.0, 1.0, 1.0 };
    float spec3[4] = { 0.5, 0.5, 0.5, 1.0 };
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, color3);
    glMaterialfv(GL_FRONT, GL_SPECULAR, spec3);
    glMateriali(GL_FRONT, GL_SHININESS, 1);
    glPushName(3);
    glPushMatrix();
    {
        glVertexPointer(3, GL_FLOAT, 0, surface);
        glNormalPointer(GL_FLOAT, 0, normal);
        glDrawElements(GL_TRIANGLES, numElement, GL_UNSIGNED_INT, element);
    }
    glPopMatrix();
    glPopName();
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, defaultSpec);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, defaultAmb);
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, defaultDif);
    glMateriali(GL_FRONT_AND_BACK, GL_SHININESS, 0);
    
    glDisableClientState(GL_NORMAL_ARRAY);
    float color8[] = { 1.0, 1.0, 1.0, 0.2 };
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color8);
    
    for(int i=0;i<traversed.size();i++) {
        int size = (int)traversed[i].size();
        float* vertices = &traversed[i][0];
        glVertexPointer(3, GL_FLOAT, 0, vertices);
        glDrawArrays(GL_POLYGON, 0, size/3);
    }
    
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, defaultSpec);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, defaultAmb);
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, defaultDif);
    glMateriali(GL_FRONT_AND_BACK, GL_SHININESS, 0);
    
    glColor3f(1, 1, 1);
    float color9[] = { 1.0, 1.0, 1.0, 0.2 };
    float spec9[] = { 0.7, 0.7, 0.7, 1.0 };
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color9);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spec9);
    glMateriali(GL_FRONT_AND_BACK, GL_SHININESS, 100);
    glPushName(7);
    glPushMatrix();
    {
        glTranslated(-25, 32, 35);
        qobj = gluNewQuadric();
        gluQuadricDrawStyle(qobj, GLU_FILL);
        gluSphere(qobj, 10, 100, 100);
    }
    glPopMatrix();
    glPopName();
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, defaultSpec);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, defaultAmb);
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, defaultDif);
    glMateriali(GL_FRONT_AND_BACK, GL_SHININESS, 0);
    

    glDisableClientState(GL_VERTEX_ARRAY);
}

void setLight(float* rcam) {
    glEnable(GL_LIGHTING);
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE );

    GLfloat light_ambient[] = { 0.2, 0.2, 0.2, 0.0 };
    GLfloat light_diffuse[] = { 0.2, 0.2, 0.2, 0.0 };
    GLfloat light_diffuse2[] = { 0.4, 0.4, 0.4, 0.0 };
    GLfloat light_diffuse3[] = { 12.0/255.0*0.6, 116.0/255.0*0.6, 232.0/255.0*0.6, 1.0 };
    GLfloat light_specular0[] = { 0.0, 0.0, 0.0, 0.0 };
    GLfloat light_specular[] = { 0.6, 0.6, 0.6, 0.0 };
    GLfloat light_position[] = { 1.0, 1.0, 0.0, 0.0 };
    GLfloat light_position2[] = { -1.0, 1.0, 0.0, 0.0 };
    GLfloat light_position3[] = { rcam[0]+trans[0], rcam[1]+trans[1], rcam[2]+trans[2], 1.0 };
    GLfloat light_position4[] = { 25.0, 31.0, -5.0, 1.0 };
    
    glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular0);
    glLightfv(GL_LIGHT0, GL_POSITION, light_position);
    glEnable(GL_LIGHT0);
    
    glLightfv(GL_LIGHT1, GL_DIFFUSE, light_diffuse);
    glLightfv(GL_LIGHT1, GL_SPECULAR, light_specular0);
    glLightfv(GL_LIGHT1, GL_POSITION, light_position2);
    glEnable(GL_LIGHT1);

    glLightfv(GL_LIGHT2, GL_AMBIENT, light_ambient);
    glLightfv(GL_LIGHT2, GL_DIFFUSE, light_diffuse2);
    glLightfv(GL_LIGHT2, GL_SPECULAR, light_specular);
    glLightfv(GL_LIGHT2, GL_POSITION, light_position3);
    glLightf(GL_LIGHT2, GL_LINEAR_ATTENUATION, 0.001);
    glEnable(GL_LIGHT2);
    
    glLightfv(GL_LIGHT3, GL_DIFFUSE, light_diffuse3);
    glLightfv(GL_LIGHT3, GL_POSITION, light_position4);
//    glLightf(GL_LIGHT3, GL_LINEAR_ATTENUATION, 0.001);
    glLightf(GL_LIGHT3, GL_QUADRATIC_ATTENUATION, 0.001);
    glEnable(GL_LIGHT3);
    
    glEnable(GL_NORMALIZE);
}

void reshape(int w, int h) {
    width = glutGet(GLUT_WINDOW_WIDTH);
    height = glutGet(GLUT_WINDOW_HEIGHT);
    
    trackBallRadius = fmax(w, h) / 2.0 * 1.5;

    glViewport(0, 0, w, h );
    
    float rcam[3] = { cam[0], cam[1], cam[2] };
    float rrot[4] = { rot[0], rot[1], rot[2] };
    
    rotate(quat, rcam);
    rotate(quat, rrot);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    float aspectRatio = (float)w/(float)h;
    gluPerspective( viewAngle /*field of view angle*/,
                    aspectRatio,
                    1.0 /*near clipping plane*/,
                    5000.0 /* far clipping plane */ );
    gluLookAt(rcam[0]+trans[0],
              rcam[1]+trans[1],
              rcam[2]+trans[2],
              trans[0],
              trans[1],
              trans[2],
              rrot[0],
              rrot[1],
              rrot[2] );
    
    traversed.clear();
    
    vec3 viewPoint = vec3(rcam[0]+trans[0], rcam[1]+trans[1], rcam[2]+trans[2]);
    bspTree.traverse(viewPoint, traversed);
    
    glMatrixMode( GL_MODELVIEW );
    setLight(rcam);
}

void display() {
    glLoadIdentity();
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    drawModel();
    glFlush();
    glutSwapBuffers();
}

void keyboardCB(unsigned char keyPressed, int x, int y) {
    switch (keyPressed) {
        case 'f':
            if (fullScreen == true) {
                glutReshapeWindow(width, height);
                fullScreen = false;
            } else {
                glutFullScreen();
                fullScreen = true;
            }
            break;
        case 'a':
            clear(trans, 3);
            viewAngle = 45;
            resize(cam, 3, 200);
            reshape(width, height);
            break;
        case 'o':
            clear(trans, 3);
            reshape(width, height);
            break;
        case 's':
            keyboardSPressed = !keyboardSPressed;
            break;
        case 'q':
            exit(0);
            break;
    }
    glutPostRedisplay();
}

void specialCB(int key, int x, int y) {
    switch (key) {
        case GLUT_KEY_UP:
            if(GLUT_ACTIVE_SHIFT==glutGetModifiers()) {
                float len = length(cam, 3);
                float newLen = len - 20;
                newLen = fmin(newLen, maxCamDistance);
                newLen = fmax(newLen, minCamDistacne);
                
                resize(cam, 3, newLen);
            } else {
                viewAngle -= 5;
                viewAngle = fmin(viewAngle, maxViewAngle);
                viewAngle = fmax(viewAngle, minViewAngle);
            }

            
            reshape(width, height);
            break;
        case GLUT_KEY_DOWN:
            if(GLUT_ACTIVE_SHIFT==glutGetModifiers()) {
                float len = length(cam, 3);
                float newLen = len + 20;
                newLen = fmin(newLen, maxCamDistance);
                newLen = fmax(newLen, minCamDistacne);
                
                resize(cam, 3, newLen);
            } else {
                viewAngle += 5;
                viewAngle = fmin(viewAngle, maxViewAngle);
                viewAngle = fmax(viewAngle, minViewAngle);
            }
            
            reshape(width, height);
            break;
    }
    glutPostRedisplay();
}

void seek(int x, int y) {
    GLuint selectBuf[64];
    GLint hits;
    GLint viewport[4];
    
    glGetIntegerv(GL_VIEWPORT, viewport);
    glSelectBuffer(64, selectBuf);
    glRenderMode(GL_SELECT);
    glInitNames();
    
    glViewport(0, 0, width, height );
    
    float rcam[3] = { cam[0], cam[1], cam[2] };
    float rrot[4] = { rot[0], rot[1], rot[2] };
    
    rotate(quat, rcam);
    rotate(quat, rrot);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    float aspectRatio = (float)width/(float)height;
    gluPickMatrix(x, viewport[3]-y, 2, 2, viewport);
    gluPerspective( viewAngle /*field of view angle*/,
                    aspectRatio,
                    1.0 /*near clipping plane*/,
                    5000.0 /* far clipping plane */ );
    gluLookAt(rcam[0]+trans[0],
              rcam[1]+trans[1],
              rcam[2]+trans[2],
              trans[0],
              trans[1],
              trans[2],
              rrot[0],
              rrot[1],
              rrot[2] );
    drawModel();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    hits = glRenderMode(GL_RENDER);
    if(hits>0) {
        GLdouble projection[16];
        GLdouble modelView[16];
        glGetDoublev(GL_PROJECTION_MATRIX, projection);
        glGetDoublev(GL_MODELVIEW_MATRIX, modelView);
        float zB = (float) selectBuf[1]/0xffffffff;
        
        double rx, ry, rz;
        gluUnProject((float)x,
                     (float)viewport[3]-(float)y,
                     zB,
                     modelView,
                     projection,
                     viewport,
                     &rx, &ry, &rz);
        
        trans[0] =rx; trans[1] = ry; trans[2] = rz;
        viewAngle = 45;
        resize(cam, 3, 60);
        reshape(width, height);
    }
}

void mouseCB(int button, int state, int x, int y) {
    if (keyboardSPressed) {
        if (button != GLUT_LEFT_BUTTON || state != GLUT_DOWN) return;
        seek(x, y);
        keyboardSPressed = false;
    }
    if (state == GLUT_UP) {
        mouseMovePressed = false;
        mouseRotatePressed = false;
    } else {
        if (button==GLUT_LEFT_BUTTON && GLUT_ACTIVE_SHIFT==glutGetModifiers()) {
            lastX = x;
            lastY = y;
            mouseMovePressed = true;
            mouseRotatePressed = false;
        } else if (button==GLUT_LEFT_BUTTON) {
            copyQuat(lastQuat, quat);
            lastX = x;
            lastY = y;
            mouseMovePressed = false;
            mouseRotatePressed = true;
        }
    }
    glutPostRedisplay();
}

void motionCB(int x, int y) {
    if (mouseRotatePressed) {
        float v1[3], v2[3];
        float halfScreenWidth = width * 0.5;
        float halfScreenHeight = height * 0.5;
        projectPointToSphere((float)lastX - halfScreenWidth, (float)halfScreenHeight - lastY, v1);
        projectPointToSphere((float)x - halfScreenWidth, (float)halfScreenHeight - y, v2);
        
        rotate(lastQuat, v1);
        rotate(lastQuat, v2);
        
        float cross[3];
        crossProduct(v1, v2, cross);
        
        float s = length(cross, 3);
        float c = dotProduct(v1, v2);
        
        float theta = -1.5 * atan2(s, c);
        if (isnan(theta)) return;
        float cosHalf = cos(theta/2);
        float sinHalf = sin(theta/2);
        
        normalize(cross, 3);
        float q[4] = { cosHalf, sinHalf * cross[0], sinHalf * cross[1], sinHalf * cross[2] };
        
        quatMult(q, lastQuat, quat);
        reshape(width, height);
    } else if (mouseMovePressed) {
        float px[3];
        float py[3] = { rot[0], rot[1], rot[2] };
        float rcam[3] = { cam[0], cam[1], cam[2] };
        rotate(quat, rcam);
        rotate(quat, py);
        
        crossProduct(rcam, py, px);
        neg(px, 3);
        normalize(px, 3);
        normalize(py, 3);
        
        float deltaX = (lastX-x)*0.05;
        float deltaY = (y-lastY)*0.05;
        
        trans[0] = trans[0] + px[0] * deltaX + py[0] * deltaY;
        trans[1] = trans[1] + px[1] * deltaX + py[1] * deltaY;
        trans[2] = trans[2] + px[2] * deltaX + py[2] * deltaY;
        lastX = x;
        lastY = y;
        reshape(width, height);
    }
    glutPostRedisplay();
}

void idle() {
    glutPostRedisplay();
}


void readTxtFile(ifstream& dataFile) {
    string line;
    stringstream ss;
    string word;
    getline(dataFile, line);
    while(line.empty() || line.at(0) == 13 || line.at(0) == '#') getline(dataFile, line);
    ss.str(line);
    ss >> word;
    if(word == "BSPLINE") {
        useBSpline = true;
    } else {
        useBSpline = false;
    }
    
    getline(dataFile, line);
    while(line.empty() || line.at(0) == 13 || line.at(0) == '#') getline(dataFile, line);
    ss.str(line);
    ss >> word;
    ncs = atoi(word.c_str());
    
    getline(dataFile, line);
    while(line.empty() || line.at(0) == 13 || line.at(0) == '#') getline(dataFile, line);
    ss.str(line);
    ss >> word;
    ncp = atoi(word.c_str());
    
    controlPoints = new float[ncs * ncp * 3];
    scalingFactors = new float[ncs];
    rotations = new float[ncs*4];
    positions = new float[ncs*3];
    
    for(int i=0;i<ncs;i++) {
        for(int j=0;j<ncp;j++) {
            getline(dataFile, line);
            while(line.empty() || line.at(0) == 13 || line.at(0) == '#') getline(dataFile, line);
            ss.str(line);
            ss >> word;
            controlPoints[i*ncp*3+j*3] = atof(word.c_str());
            ss >> word;
            controlPoints[i*ncp*3+j*3+2] = atof(word.c_str());
            controlPoints[i*ncp*3+j*3+1] = 0;
        }
        getline(dataFile, line);
        while(line.empty() || line.at(0) == 13 || line.at(0) == '#') getline(dataFile, line);
        ss.str(line);
        ss >> word;
        scalingFactors[i] = atof(word.c_str());
        
        float theta, rotAxis[3];
        getline(dataFile, line);
        while(line.empty() || line.at(0) == 13 || line.at(0) == '#') getline(dataFile, line);
        ss.str(line);
        ss >> word; theta = atof(word.c_str());
        ss >> word; rotAxis[0] = atof(word.c_str());
        ss >> word; rotAxis[1] = atof(word.c_str());
        ss >> word; rotAxis[2] = atof(word.c_str());
        
        normalize(rotAxis, 3);
        
        float cosHalf = cos(theta/2);
        float sinHalf = sin(theta/2);
        rotations[i*4] = cosHalf;
        rotations[i*4+1] = sinHalf * rotAxis[0];
        rotations[i*4+2] = sinHalf * rotAxis[1];
        rotations[i*4+3] = sinHalf * rotAxis[2];
        
        getline(dataFile, line);
        while(line.empty() || line.at(0) == 13 || line.at(0) == '#') getline(dataFile, line);
        ss.str(line);
        ss >> word; positions[i*3] = atof(word.c_str());
        ss >> word; positions[i*3+1] = atof(word.c_str());
        ss >> word; positions[i*3+2] = atof(word.c_str());
    }
}

// Split s with delimiter
vector<string> split(const string& s, const string& delimiter) {
    size_t pos_start = 0, pos_end, delim_len = delimiter.length();
    string token;
    vector<string> res;

    while ((pos_end = s.find (delimiter, pos_start)) != string::npos) {
        token = s.substr (pos_start, pos_end - pos_start);
        pos_start = pos_end + delim_len;
        res.push_back (token);
    }

    res.push_back (s.substr (pos_start));
    return res;
}

void readObjFile(ifstream& dataFile, float scale, float translation[]) {
    string line;
    stringstream ss;
    string word;
    
    objVertices[numObjs] = vector<vector<float> >();
    objNormals[numObjs] = vector<vector<float> >();
    
    vector<float> vertices = vector<float>();
    vector<float> normals = vector<float>();
    vector<GLuint> elements = vector<GLuint>();
    vector<float> realNormals = vector<float>();
    
    while(!dataFile.eof()) {
        getline(dataFile, line);
        while(line.empty() || line.at(0) == 13 || line.at(0) == '#') {
            if (dataFile.eof()) break;
            getline(dataFile, line);
        }
        ss.clear();
        ss.str(line);
        ss >> word;
        if(word == "v") {
            float x, y, z;
            ss >> x >> y >> z;
            x *= scale; y *= scale; z *= scale;
            x += translation[0]; y += translation[1]; z+=translation[2];
            vertices.push_back(x);
            vertices.push_back(y);
            vertices.push_back(z);
        } else if(word == "vn") {
            float x, y, z;
            ss >> x >> y >> z;
            normals.push_back(x);
            normals.push_back(y);
            normals.push_back(z);
        } else if(word == "f") {
            vector<string> faces = vector<string>();
            ss >> word;
            while(ss) {
                faces.push_back(word);
                ss >> word;
            }
            
            vector<int> faceVIndices = vector<int>();
            vector<int> faceNIndices = vector<int>();
            
            for(int i=0;i<faces.size();i++) {
                vector<string> splitted = split(faces[i], "/");
                int vIndex = atoi(splitted[0].c_str()) - 1;
                faceVIndices.push_back(vIndex);
                if(splitted.size()>=3) {
                    int nIndex = atoi(splitted[2].c_str()) - 1;
                    faceNIndices.push_back(nIndex);
                }
            }
            
            vector<float> newFace = vector<float>();
            vector<float> newNormal = vector<float>();
            
            for(int i=0;i<faceVIndices.size();i++) {
                int vIndex = faceVIndices[i] * 3;
                newFace.push_back(vertices[vIndex]);
                newFace.push_back(vertices[vIndex+1]);
                newFace.push_back(vertices[vIndex+2]);
                
                if(i<faceNIndices.size()) {
                    int nIndex = faceNIndices[i] * 3;
                    newNormal.push_back(normals[nIndex]);
                    newNormal.push_back(normals[nIndex+1]);
                    newNormal.push_back(normals[nIndex+2]);
                }
            }
            
            objVertices[numObjs].push_back(newFace);
            objNormals[numObjs].push_back(newNormal);
        }
    }
    numObjs++;
}

void bSpline(int num, int& numCurve, float* cPoints, float* &curve, bool closed, int res, int dim) {
    if(num<4) return;
    if(closed) {
        numCurve = num * res * dim;
    } else {
        numCurve = (num-3) * res * dim;
    }
    
    curve = new float[numCurve];
    
    int index = 0;
    for(int i=0;i<num-3;i++) {
        for(int r=0;r<res;r++) {
            float t = (1.0/((float)res-1))*(float)r;
            float b0 = (1.0-t)*(1.0-t)*(1.0-t)/6.0;
            float b1 = (3.0*t*t*t - 6.0*t*t + 4.0)/6.0;
            float b2 = (-3.0*t*t*t + 3*t*t + 3*t + 1)/6.0;
            float b3 = t*t*t/6.0;
            for(int j=0;j<dim;j++)
                curve[index++] = b0*cPoints[i*dim+j] + b1*cPoints[(i+1)*dim+j] + b2*cPoints[(i+2)*dim+j] + b3*cPoints[(i+3)*dim+j];
        }
    }
    if(closed) {
        for(int r=0;r<res;r++) {
            float t = (1.0/((float)res-1))*(float)r;
            float b0 = (1.0-t)*(1.0-t)*(1.0-t)/6.0;
            float b1 = (3.0*t*t*t - 6.0*t*t + 4.0)/6.0;
            float b2 = (-3.0*t*t*t + 3*t*t + 3*t + 1)/6.0;
            float b3 = t*t*t/6.0;
            for(int j=0;j<dim;j++)
                curve[index++] = b0*cPoints[(num-3)*dim+j] + b1*cPoints[(num-2)*dim+j] + b2*cPoints[(num-1)*dim+j] + b3*cPoints[0*dim+j];
        }
        for(int r=0;r<res;r++) {
            float t = (1.0/((float)res-1))*(float)r;
            float b0 = (1.0-t)*(1.0-t)*(1.0-t)/6.0;
            float b1 = (3.0*t*t*t - 6.0*t*t + 4.0)/6.0;
            float b2 = (-3.0*t*t*t + 3*t*t + 3*t + 1)/6.0;
            float b3 = t*t*t/6.0;
            for(int j=0;j<dim;j++)
                curve[index++] = b0*cPoints[(num-2)*dim+j] + b1*cPoints[(num-1)*dim+j] + b2*cPoints[0*dim+j] + b3*cPoints[1*dim+j];
        }
        for(int r=0;r<res;r++) {
            float t = (1.0/((float)res-1))*(float)r;
            float b0 = (1.0-t)*(1.0-t)*(1.0-t)/6.0;
            float b1 = (3.0*t*t*t - 6.0*t*t + 4.0)/6.0;
            float b2 = (-3.0*t*t*t + 3*t*t + 3*t + 1)/6.0;
            float b3 = t*t*t/6.0;
            for(int j=0;j<dim;j++)
                curve[index++] = b0*cPoints[(num-1)*dim+j] + b1*cPoints[0*dim+j] + b2*cPoints[1*dim+j] + b3*cPoints[2*dim+j];
        }
    }
}

// Construct Catmull-Rom Spline of dimension (dim), resolution (res)
void catmullRomSpline(int num, int& numCurve, float* cPoints, float* &curve, bool closed, int res, int dim) {
    if(num<4) return;
    
    int numBPoints = (num*3-2);
    if(closed) numBPoints = (num*3+1);
    numCurve = (numBPoints-1)/3 * res * dim;
    float* bPoints = new float[numBPoints * dim];
    curve = new float[numCurve];
    
    for(int i=0;i<num;i++) {
        for(int j=0;j<dim;j++)
            bPoints[3*i*dim+j] = cPoints[i*dim+j];
    }
    
    if(closed) {
        for(int j=0;j<dim;j++)
            bPoints[3*num*dim+j] = cPoints[0*dim+j];
    }
    
    for(int i=0;i<num-1;i++) {
        if(i==0) {
            if(closed) {
                for(int j=0;j<dim;j++)
                    bPoints[1*dim+j] = cPoints[0*dim+j] + (cPoints[1*dim+j] - cPoints[(num-1)*dim+j])/6.0;
            } else {
                for(int j=0;j<dim;j++)
                    bPoints[1*dim+j] = cPoints[0*dim+j];
            }
        } else {
            for(int j=0;j<dim;j++)
                bPoints[(3*i+1)*dim+j] = cPoints[i*dim+j] + (cPoints[(i+1)*dim+j] - cPoints[(i-1)*dim+j])/6.0;
        }
        
        if(i==num-2) {
            if(closed) {
                for(int j=0;j<dim;j++) {
                    bPoints[(3*i+2)*dim+j] = cPoints[(num-1)*dim+j] - (cPoints[0*dim+j] - cPoints[(num-2)*dim+j])/6.0;
                    bPoints[(3*i+4)*dim+j] = cPoints[(num-1)*dim+j] + (cPoints[0*dim+j] - cPoints[(num-2)*dim+j])/6.0;
                    bPoints[(3*i+5)*dim+j] = cPoints[0*dim+j] - (cPoints[1*dim+j] - cPoints[(num-1)*dim+j])/6.0;
                }
            } else {
                for(int j=0;j<dim;j++)
                    bPoints[(3*i+2)*dim+j] = cPoints[(num-1)*dim+j];
            }
        } else {
            for(int j=0;j<dim;j++)
                bPoints[(3*i+2)*dim+j] = cPoints[(i+1)*dim+j] - (cPoints[(i+2)*dim+j] - cPoints[i*dim+j])/6.0;
        }
    }
    
    int index = 0;
    for(int i=0;i<numBPoints-3;i+=3) {
        for(int r=0;r<res;r++) {
            float t = (1.0/((float)res-1))*(float)r;
            float b0 = (1.0-t)*(1.0-t)*(1.0-t);
            float b1 = 3.0*(1.0-t)*(1.0-t)*t;
            float b2 = 3.0*(1.0-t)*t*t;
            float b3 = t*t*t;
            for(int j=0;j<dim;j++)
                curve[index++] = b0*bPoints[i*dim+j] + b1*bPoints[(i+1)*dim+j] + b2*bPoints[(i+2)*dim+j] + b3*bPoints[(i+3)*dim+j];
        }
    }
}

void catmullRomQuatSpline(int num, int& numCurve, float* cQuat, float* &curve, int res) {
    if(num<4) return;
    int numBPoints = (num*3-2);
    numCurve = (numBPoints-1)/3 * res * 4;
    float* bPoints = new float[numBPoints * 4];
    curve = new float[numCurve];
    
    int dim = 4;
    
    for(int i=0;i<num;i++) {
        for(int j=0;j<dim;j++)
            bPoints[3*i*dim+j] = cQuat[i*dim+j];
    }
    
    for(int i=0;i<num-1;i++) {
        if(i==0) {
            for(int j=0;j<dim;j++)
                bPoints[1*dim+j] = cQuat[0*dim+j]; // Copy quaternion
        } else {
            float qi[4] = { cQuat[i*dim], cQuat[i*dim+1], cQuat[i*dim+2], cQuat[i*dim+3] };
            float invQim1[4] = { cQuat[(i-1)*dim], -cQuat[(i-1)*dim+1], -cQuat[(i-1)*dim+2], -cQuat[(i-1)*dim+3] };
            float qi1[4] = { cQuat[(i+1)*dim], cQuat[(i+1)*dim+1], cQuat[(i+1)*dim+2], cQuat[(i+1)*dim+3] };
            
            float mq[4];
            quatMult(invQim1, qi1, mq);
            
            float rq[4];
            quatPow(mq, 1.0/6.0, rq);
            quatMult(qi, rq, bPoints+(3*i+1)*dim); // result copied to bPoints[(3*i+1)*dim]
        }
        
        if(i==num-2) {
            for(int j=0;j<dim;j++)
                bPoints[(3*i+2)*dim+j] = cQuat[(num-1)*dim+j]; // Copy quaternion
        } else {
            float qi1[4] = { cQuat[(i+1)*dim], cQuat[(i+1)*dim+1], cQuat[(i+1)*dim+2], cQuat[(i+1)*dim+3] };
            float qi[4] = { cQuat[i*dim], cQuat[i*dim+1], cQuat[i*dim+2], cQuat[i*dim+3] };
            float invQi2[4] = { cQuat[(i+2)*dim], -cQuat[(i+2)*dim+1], -cQuat[(i+2)*dim+2], -cQuat[(i+2)*dim+3] };
            
            float mq[4];
            quatMult(invQi2, qi, mq);
            
            float rq[4];
            quatPow(mq, 1.0/6.0, rq);
            quatMult(qi1, rq, bPoints+(3*i+2)*dim);
        }
    }
    
    int index = 0;
    for(int i=0;i<numBPoints-3;i+=3) {
        for(int r=0;r<res;r++) {
            float t = (1.0/((float)res-1))*(float)r;
            float slerp01[4];
            float slerp12[4];
            float slerp23[4];
            
            slerp(bPoints+i*dim, bPoints+(i+1)*dim, t, slerp01);
            slerp(bPoints+(i+1)*dim, bPoints+(i+2)*dim, t, slerp12);
            slerp(bPoints+(i+2)*dim, bPoints+(i+3)*dim, t, slerp23);
            
            float slerp0112[4];
            float slerp1223[4];
            
            slerp(slerp01, slerp12, t, slerp0112);
            slerp(slerp12, slerp23, t, slerp1223);
            
            slerp(slerp0112, slerp1223, t, curve+index*dim);
            index++;
        }
    }
}

void constructSplines(float translation[]) {
    catmullRomSpline(ncs, numScalingCurve, scalingFactors, scalingCurve, false, tResolution, 1);
    catmullRomSpline(ncs, numPositionCurve, positions, positionCurve, false, tResolution, 3);
    catmullRomQuatSpline(ncs, numRotationCurve, rotations, rotationCurve, tResolution);
    
    // construct catmull spline on (ncp) control points
    catmullRomSpline(ncs, numControlCurve, controlPoints, controlCurve, false, tResolution, 3*ncp);
    
    numSurface = (ncs-1)*tResolution*ncp*sResolution*3;
    surface = new float[numSurface];
    normal = new float[numSurface];
    
    int index = 0;
    
    for(int i=0;i<ncs-1;i++) {
        for(int r=0;r<tResolution;r++) {
            int numCrossSectionCurve;
            float* crossSection;
            float* crossControlPoints = new float[ncp*3];
            
            for(int j=0;j<ncp;j++) {
                float p[3];
                // copy control point
                p[0] = controlCurve[(i*tResolution*ncp+r*ncp+j)*3];
                p[1] = controlCurve[(i*tResolution*ncp+r*ncp+j)*3+1];
                p[2] = controlCurve[(i*tResolution*ncp+r*ncp+j)*3+2];
                
                // scale the point
                p[0] *= scalingCurve[i*tResolution+r];
                p[1] *= scalingCurve[i*tResolution+r];
                p[2] *= scalingCurve[i*tResolution+r];
                
                // rotate the point
                rotate(rotationCurve+(i*tResolution+r)*4, p);
                
                // translate the point
                p[0] += positionCurve[(i*tResolution+r)*3];
                p[1] += positionCurve[(i*tResolution+r)*3+1];
                p[2] += positionCurve[(i*tResolution+r)*3+2];
                
                crossControlPoints[j*3+0] = p[0];
                crossControlPoints[j*3+1] = p[1];
                crossControlPoints[j*3+2] = p[2];
            }
            
            if(useBSpline) {
                bSpline(ncp, numCrossSectionCurve, crossControlPoints, crossSection, true, sResolution, 3);
            } else {
                catmullRomSpline(ncp, numCrossSectionCurve, crossControlPoints, crossSection, true, sResolution, 3);
            }
            
            for(int j=0;j<numCrossSectionCurve;j+=3) {
                surface[index] = crossSection[j+0] + translation[0];
                normal[index++] = crossSection[j+0] - positionCurve[(i*tResolution+r)*3];
                surface[index] = crossSection[j+1] + translation[1];
                normal[index++] = crossSection[j+1] - positionCurve[(i*tResolution+r)*3+1];
                surface[index] = crossSection[j+2] + translation[2];
                normal[index++] = crossSection[j+2] - positionCurve[(i*tResolution+r)*3+2];
                resize(normal+index-3, 3, 1);
            }
        }
    }
    numElement = numSurface*6/3 - ncp*sResolution*2*3;
    element = new GLuint[numElement];
    
    int eIdx = 0;
    for(int i=0;i<(ncs-1)*tResolution-1;i++) {
        for(int j=0;j<ncp*sResolution;j++) {
            element[eIdx++] = i*ncp*sResolution+j;
            element[eIdx++] = (i+1)*ncp*sResolution+j;
            element[eIdx++] = i*ncp*sResolution+j+1;
            element[eIdx++] = (i+1)*ncp*sResolution+j;
            element[eIdx++] = (i+1)*ncp*sResolution+j+1;
            element[eIdx++] = i*ncp*sResolution+j+1;
        }
    }
}

void timer(int unused) {
    /* call the display callback and forces the current window to be displayed */
    glutPostRedisplay();
    glutTimerFunc(timeStep, timer, 0);
}

inline bool ends_with(std::string const & value, std::string const & ending) {
    if (ending.size() > value.size()) return false;
    return std::equal(ending.rbegin(), ending.rend(), value.rbegin());
}

int main(int argc, char * argv[]) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(width, height);
    glutCreateWindow("HW4_2017-11522");
    
    float trans1[] = { -67, -70, -115 };
    string fname = "table.obj";
    ifstream dataFile(fname);
    if (!dataFile.is_open()) return 0;
    readObjFile(dataFile, 1.2, trans1);
    dataFile.close();
    
    float trans2[] = { 40, -40, 0 };
    fname = "mug.obj";
    dataFile.open(fname);
    if (!dataFile.is_open()) return 0;
    readObjFile(dataFile, 180, trans2);
    dataFile.close();
    
    float trans3[] = { 0, 23, -25 };
    fname = "can.obj";
    dataFile.open(fname);
    if (!dataFile.is_open()) return 0;
    readObjFile(dataFile, 250, trans3);
    dataFile.close();
    
    float trans4[] = { 0, 25, 40 };
    fname = "ico_sphere.obj";
    dataFile.open(fname);
    if (!dataFile.is_open()) return 0;
    readObjFile(dataFile, 10, trans4);
    dataFile.close();
    
    float trans5[] = { -10, 25, 10 };
    fname = "ico_sphere.obj";
    dataFile.open(fname);
    if (!dataFile.is_open()) return 0;
    readObjFile(dataFile, 10, trans5);
    dataFile.close();
    
    float trans6[] = { -20, 23, -25 };
    fname = "can.obj";
    dataFile.open(fname);
    if (!dataFile.is_open()) return 0;
    readObjFile(dataFile, 250, trans6);
    dataFile.close();
    
    vector<vector<vec3> > polygons;
    
    for(int i=0;i<objVertices[3].size();i++) {
        int size = (int)objVertices[3][i].size();
        
        vector<vec3> polygon;
        for(int j=0;j<size;j+=3) {
            vec3 v = vec3(objVertices[3][i][j], objVertices[3][i][j+1], objVertices[3][i][j+2]);
            polygon.push_back(v);
        }
        polygons.push_back(polygon);
    }
    
    for(int i=0;i<objVertices[4].size();i++) {
        int size = (int)objVertices[4][i].size();
        
        vector<vec3> polygon;
        for(int j=0;j<size;j+=3) {
            vec3 v = vec3(objVertices[4][i][j], objVertices[4][i][j+1], objVertices[4][i][j+2]);
            polygon.push_back(v);
        }
        polygons.push_back(polygon);
    }
    
    bspTree.construct(polygons);
    
    reshape(width, height);
    glClearColor(1, 1, 1, 0);
    
    glClearDepth(1.0f);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_BLEND);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    glFrontFace(GL_CCW);
    
    glutIdleFunc(idle);
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboardCB);
    glutSpecialFunc(specialCB);
    glutMouseFunc(mouseCB);
    glutTimerFunc(timeStep, timer, 0);
    glutMotionFunc(motionCB);
    
    glutMainLoop();
    return 0;
}
