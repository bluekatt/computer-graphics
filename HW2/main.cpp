//
//  main.cpp
//  Computer Graphics
//
//  Created by 박종석 on 2021/09/01.
//

#include <iostream>
#include <cmath>
#ifdef __APPLE__
/* Defined before OpenGL and GLUT includes to avoid deprecation messages */
#define GL_SILENCE_DEPRECATION
#include <OpenGL/gl.h>
#include <GLUT/glut.h>
#else
#include <GL/gl.h>
#include <GL/glut.h>
#endif

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

float maxCamDistance = 2000.0;
float minCamDistacne = 200.0;
float maxViewAngle = 170.0;
float minViewAngle = 1.0;
bool leftButton = false;
GLfloat mousePosX, mousePosY;

float timeStep = 1000.0 / 30.0;

GLUquadricObj* qobj;

float cam[3] = { 0, 0, 700.0 };
float ori[3] = { 0.0, 0.0, 0.0 };
float rot[3] = { 0.0, 1.0, 0.0 };

float bodyColor[3] = { 0.93, 0.55, 0.45 };
float pantsColor[3] = { 0.73, 0.78, 0.28 };
float patternColor[3] = { 0.42, 0.34, 0.61 };

int flexing = 1;

GLfloat shoulderMaxAngle = 40.0;
GLfloat shoulderMinAngle = 10.0;
GLfloat elbowMaxAngle = 20.0;
GLfloat elbowMinAngle = 0.0;
GLfloat hipJointMaxAngle = 40.0;
GLfloat hipJointMinAngle = 0.0;
GLfloat kneeMaxAngle = 40.0;
GLfloat kneeMinAngle = 0.0;

GLfloat leftShoulderAngle = -shoulderMaxAngle;
GLfloat rightShoulderAngle = shoulderMaxAngle;
GLfloat leftElbowAngle = -elbowMaxAngle;
GLfloat rightElbowAngle = elbowMaxAngle;

GLfloat leftHipJointAngle = 0;
GLfloat rightHipJointAngle = 0;
GLfloat leftKneeAngle = 0;
GLfloat rightKneeAngle = 0;

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

void drawModel() {
    glPushName(0); // Used for picking
    // Upper Body
    glPushMatrix();
    {
        glRotatef(-90, 1.0, 0.0, 0.0);
        glTranslated(0, 0, 25);
        glColor3f(bodyColor[0], bodyColor[1], bodyColor[2]);
        qobj = gluNewQuadric();
        gluQuadricDrawStyle(qobj, GLU_FILL);
        gluCylinder(qobj, 100, 25, 200, 100, 100);
        
        glPushMatrix();
        {
            glLineWidth(3);
            glBegin(GL_LINE_STRIP);
            glColor3f(0, 0, 0);
            for(float d=240;d<300;d+=0.1) {
                glVertex3d(cos(d/180*M_PI)*70.5, sin(d/180*M_PI)*70.5, 80);
            }
            glEnd();
        }
        glPopMatrix();
        
        glTranslated(0, 0, 190.625);
        glColor3f(bodyColor[0], bodyColor[1], bodyColor[2]);
        qobj = gluNewQuadric();
        gluQuadricDrawStyle(qobj, GLU_FILL);
        gluSphere(qobj, 26.7, 100, 100);
        glTranslated(0, 0, -215.625);
        glRotatef(90, 1, 0, 0);
        
        // Eyes
        glPushMatrix();
        {
            glTranslated(0, 150, 45);
            GLfloat m[16] = {
                1, 0, 0, 0,
                0, 1.5, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1
            };
            glMultMatrixf(m);
            glTranslated(-16, 0, 0);
            glColor3f(1, 1, 1);
            qobj = gluNewQuadric();
            gluQuadricDrawStyle(qobj, GLU_FILL);
            gluSphere(qobj, 20, 100, 100);
            glPushMatrix();
            {
                glColor3f(0, 0, 0);
                glTranslated(0, 5, 20);
                qobj = gluNewQuadric();
                gluQuadricDrawStyle(qobj, GLU_FILL);
                gluSphere(qobj, 5, 20, 20);
            }
            glPopMatrix();
            glPushMatrix();
            {
                glTranslated(0, 30, -5);
                glRotatef(10.0, 0, 0, 1);
                glBegin(GL_POLYGON);
                glColor3f(0, 0, 0);
                glVertex3d(-10, -3, 0);
                glVertex3d(-10, 3, 0);
                glVertex3d(10, 3, 0);
                glVertex3d(10, -3, 0);
                glEnd();
            }
            glPopMatrix();
            
            glColor3f(1, 1, 1);
            glTranslated(32, 0, 0);
            qobj = gluNewQuadric();
            gluQuadricDrawStyle(qobj, GLU_FILL);
            gluSphere(qobj, 20, 100, 100);
            glPushMatrix();
            {
                glColor3f(0, 0, 0);
                glTranslated(0, 5, 20);
                qobj = gluNewQuadric();
                gluQuadricDrawStyle(qobj, GLU_FILL);
                gluSphere(qobj, 5, 20, 20);
            }
            glPopMatrix();
            glPushMatrix();
            {
                glTranslated(0, 30, -5);
                glRotatef(-10.0, 0, 0, 1);
                glBegin(GL_POLYGON);
                glColor3f(0, 0, 0);
                glVertex3d(-10, -3, 0);
                glVertex3d(-10, 3, 0);
                glVertex3d(10, 3, 0);
                glVertex3d(10, -3, 0);
                glEnd();
            }
            glPopMatrix();

        }
        glPopMatrix();
        
        // Right Arm
        glPushMatrix();
        {
            glColor3f(bodyColor[0], bodyColor[1], bodyColor[2]);
            glTranslated(60, 80, 0);
            
            glRotatef(rightShoulderAngle, 0, 0, 1);
            glPushMatrix();
            {
                glRotatef(90, 0, 1, 0);
                qobj = gluNewQuadric();
                gluQuadricDrawStyle(qobj, GLU_FILL);
                gluCylinder(qobj, 40, 28, 76, 100, 100);
            }
            glPopMatrix();
            
            glTranslated(70, 0, 0);
            qobj = gluNewQuadric();
            gluQuadricDrawStyle(qobj, GLU_FILL);
            gluSphere(qobj, 28.6, 100, 100);
            
            glRotatef(rightElbowAngle, 0, 0, 1);
            glPushMatrix();
            {
                glTranslated(6, 0, 0);
                glPushMatrix();
                {
                    glRotatef(90, 0, 1, 0);
                    qobj = gluNewQuadric();
                    gluQuadricDrawStyle(qobj, GLU_FILL);
                    gluCylinder(qobj, 28, 18, 50, 100, 100);
                }
                glPopMatrix();
                
                glTranslated(46.4, 0, 0);
                qobj = gluNewQuadric();
                gluQuadricDrawStyle(qobj, GLU_FILL);
                gluSphere(qobj, 18.4, 100, 100);
            }
            glPopMatrix();
        }
        glPopMatrix();
        
        // Left Arm
        glPushMatrix();
        {
            glColor3f(bodyColor[0], bodyColor[1], bodyColor[2]);
            glTranslated(-60, 80, 0);
            glRotatef(180, 0, 0, 1);
            
            glRotatef(leftShoulderAngle, 0, 0, 1);
            glPushMatrix();
            {
                glRotatef(90, 0, 1, 0);
                qobj = gluNewQuadric();
                gluQuadricDrawStyle(qobj, GLU_FILL);
                gluCylinder(qobj, 40, 28, 76, 100, 100);
            }
            glPopMatrix();
            
            glTranslated(70, 0, 0);
            qobj = gluNewQuadric();
            gluQuadricDrawStyle(qobj, GLU_FILL);
            gluSphere(qobj, 28.6, 100, 100);
            
            glRotatef(leftElbowAngle, 0, 0, 1);
            glPushMatrix();
            {
                glTranslated(6, 0, 0);
                glPushMatrix();
                {
                    glRotatef(90, 0, 1, 0);
                    qobj = gluNewQuadric();
                    gluQuadricDrawStyle(qobj, GLU_FILL);
                    gluCylinder(qobj, 28, 18, 50, 100, 100);
                }
                glPopMatrix();
                
                glTranslated(46.4, 0, 0);
                qobj = gluNewQuadric();
                gluQuadricDrawStyle(qobj, GLU_FILL);
                gluSphere(qobj, 18.4, 100, 100);
            }
            glPopMatrix();
        }
        glPopMatrix();
    }
    glPopMatrix();
    
    // Lower Body
    glPushMatrix();
    {
        glColor3f(pantsColor[0], pantsColor[1], pantsColor[2]);
        qobj = gluNewQuadric();
        gluQuadricDrawStyle(qobj, GLU_FILL);
        gluSphere(qobj, 102, 100, 100);
        
        // Right Leg
        glPushMatrix();
        {
            glTranslatef(55, -30, 0);
            
            glRotatef(rightHipJointAngle, 1, 0, 0);
            
            glPushMatrix();
            {
                glRotatef(90, 1, 0, 0);
                glColor3f(pantsColor[0], pantsColor[1], pantsColor[2]);
                qobj = gluNewQuadric();
                gluQuadricDrawStyle(qobj, GLU_FILL);
                gluCylinder(qobj, 42, 42, 80, 100, 100);
                
                glColor3f(bodyColor[0], bodyColor[1], bodyColor[2]);
                qobj = gluNewQuadric();
                gluQuadricDrawStyle(qobj, GLU_FILL);
                gluCylinder(qobj, 35, 35, 80, 100, 100);
            }
            glPopMatrix();
            
            glTranslated(0, -80, 0);
            qobj = gluNewQuadric();
            gluSphere(qobj, 35, 100, 100);
            
            glRotatef(rightKneeAngle, 1, 0, 0);
            glPushMatrix();
            {
                glTranslated(0, -18, 0);
                
                glPushMatrix();
                {
                    glRotatef(90, 1, 0, 0);
                    qobj = gluNewQuadric();
                    gluQuadricDrawStyle(qobj, GLU_FILL);
                    gluCylinder(qobj, 30.5, 20, 20, 100, 100);
                }
                glPopMatrix();
                
                glTranslated(0, -11.3, 0);
                qobj = gluNewQuadric();
                gluQuadricDrawStyle(qobj, GLU_FILL);
                gluSphere(qobj, 21.8, 100, 100);
            }
            glPopMatrix();
        }
        glPopMatrix();
        
        // Left Leg
        glPushMatrix();
        {
            glTranslatef(-55, -30, 0);
            
            glRotatef(leftHipJointAngle, 1, 0, 0);
            
            glPushMatrix();
            {
                glRotatef(90, 1, 0, 0);
                glColor3f(pantsColor[0], pantsColor[1], pantsColor[2]);
                qobj = gluNewQuadric();
                gluQuadricDrawStyle(qobj, GLU_FILL);
                gluCylinder(qobj, 42, 42, 80, 100, 100);
                
                glColor3f(bodyColor[0], bodyColor[1], bodyColor[2]);
                qobj = gluNewQuadric();
                gluQuadricDrawStyle(qobj, GLU_FILL);
                gluCylinder(qobj, 35, 35, 80, 100, 100);
            }
            glPopMatrix();
            
            glTranslated(0, -80, 0);
            qobj = gluNewQuadric();
            gluSphere(qobj, 35, 100, 100);
            
            glRotatef(leftKneeAngle, 1, 0, 0);
            glPushMatrix();
            {
                glTranslated(0, -18, 0);
                
                glPushMatrix();
                {
                    glRotatef(90, 1, 0, 0);
                    qobj = gluNewQuadric();
                    gluQuadricDrawStyle(qobj, GLU_FILL);
                    gluCylinder(qobj, 30.5, 20, 20, 100, 100);
                }
                glPopMatrix();
                
                glTranslated(0, -11.3, 0);
                qobj = gluNewQuadric();
                gluQuadricDrawStyle(qobj, GLU_FILL);
                gluSphere(qobj, 21.8, 100, 100);
            }
            glPopMatrix();
        }
        glPopMatrix();
    }
    glPopMatrix();
    glPopName();
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
    glMatrixMode( GL_MODELVIEW );
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
            resize(cam, 3, 700);
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
        resize(cam, 3, 300);
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
        
        float deltaX = (lastX-x)*0.5;
        float deltaY = (y-lastY)*0.5;
        
        trans[0] = trans[0] + px[0] * deltaX + py[0] * deltaY;
        trans[1] = trans[1] + px[1] * deltaX + py[1] * deltaY;
        trans[2] = trans[2] + px[2] * deltaX + py[2] * deltaY;
        lastX = x;
        lastY = y;
        reshape(width, height);
    }
    glutPostRedisplay();
}

void animate() {
    leftElbowAngle += ((elbowMaxAngle-elbowMinAngle) / 60.0) * (float)flexing;
    leftShoulderAngle += ((shoulderMaxAngle-shoulderMinAngle) / 60.0) * (float)flexing;
    rightElbowAngle -= ((elbowMaxAngle-elbowMinAngle) / 60.0) * (float)flexing;
    rightShoulderAngle -= ((shoulderMaxAngle-shoulderMinAngle) / 60.0) * (float)flexing;
    
    leftKneeAngle += ((kneeMaxAngle-kneeMinAngle) / 60.0) * (float)flexing;
    rightKneeAngle += ((kneeMaxAngle-kneeMinAngle) / 60.0) * (float)flexing;
    leftHipJointAngle -= ((hipJointMaxAngle-hipJointMinAngle) / 60.0) * (float)flexing;
    rightHipJointAngle -= ((hipJointMaxAngle-hipJointMinAngle) / 60.0) * (float)flexing;
    
    if (leftElbowAngle >= 0 || leftElbowAngle <= -elbowMaxAngle ||
        leftShoulderAngle >= 0 || leftShoulderAngle <= -shoulderMaxAngle ||
        rightElbowAngle <= 0 || rightElbowAngle >= elbowMaxAngle ||
        rightShoulderAngle <= 0 || rightShoulderAngle >= shoulderMaxAngle ||
        leftKneeAngle <= 0 || leftKneeAngle >= kneeMaxAngle ||
        leftHipJointAngle >= 0 || leftHipJointAngle <= -hipJointMaxAngle ||
        rightKneeAngle <= 0 || rightKneeAngle >= kneeMaxAngle ||
        rightHipJointAngle >= 0 || rightHipJointAngle <= -hipJointMaxAngle
        ) {
        flexing = -flexing;
    }
}

void timer(int unused) {
    /* call the display callback and forces the current window to be displayed */
    animate();
    
    glutPostRedisplay();
    glutTimerFunc(timeStep, timer, 0);
}

void idle() {
    glutPostRedisplay();
}

void setLight() {
    glShadeModel (GL_SMOOTH);
    GLfloat light_ambient[] = { 0.0, 0.0, 0.0, 1.0 };
    GLfloat light_diffuse[] = { 1.0, 1.0, 1.0, 1.0 };
    GLfloat light_specular[] = { 1.0, 1.0, 1.0, 1.0 };
    GLfloat light_position[] = { 0.0, 0.0, 1.0, 0.0 };
    GLfloat light_position2[] = { 0.0, 0.0, -1.0, 0.0 };
    
    glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
    glLightfv(GL_LIGHT0, GL_POSITION, light_position);
    glLightfv(GL_LIGHT1, GL_AMBIENT, light_ambient);
    glLightfv(GL_LIGHT1, GL_DIFFUSE, light_diffuse);
    glLightfv(GL_LIGHT1, GL_SPECULAR, light_specular);
    glLightfv(GL_LIGHT1, GL_POSITION, light_position2);
    
    glEnable(GL_COLOR_MATERIAL);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHT1);
}

int main(int argc, char * argv[]) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(width, height);
    glutCreateWindow("HW2_2017-11522");
    
    reshape(width, height);
    glClearColor(0.9, 0.9, 0.9, 0);
    
    glClearDepth(1.0f);
    glDepthFunc(GL_LESS);
    glEnable(GL_DEPTH_TEST);
    
    setLight();
    
    glutIdleFunc(idle);
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboardCB);
    glutSpecialFunc(specialCB);
    glutMouseFunc(mouseCB);
    glutMotionFunc(motionCB);
    glutTimerFunc(timeStep, timer, 0);
    
    glutMainLoop();
    return 0;
}
