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

const GLdouble ORIGINAL_WIDTH = 500;
const GLdouble ORIGINAL_HEIGHT = 500;

bool leftButton = false;
GLfloat mousePosX, mousePosY;

float timeStep = 1000.0 / 30.0;

GLUquadricObj* qobj;

float eye[3] = { 600.0, 350.0, 600.0 };
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


void loadGlobalCoord() {
    glLoadIdentity();
    gluLookAt(eye[0], eye[1], eye[2], ori[0], ori[1], ori[2], 0, 1, 0);
}

void glutMotion(int x, int y) {
    if ( leftButton ) {
        float dx = x - mousePosX;

        mousePosX = x;
        
        float theta = atan2(eye[2], eye[0]);
        float length = sqrt(eye[0]*eye[0]+eye[2]*eye[2]);
        
        theta += dx*0.005;
        
        eye[0] = length * cos(theta);
        eye[2] = length * sin(theta);

        loadGlobalCoord();
    }
}

void glutMouse(int button, int state, int x, int y) {
    switch ( button ) {
        case GLUT_LEFT_BUTTON:
            if ( state == GLUT_DOWN )
            {
                mousePosX = x;
                mousePosY = y;
                leftButton = true;
            }
            else if ( state == GLUT_UP )
            {
                leftButton = false;
            }
            break;
        case GLUT_RIGHT_BUTTON:break;
        case 3:break;
        default:break;
    }
}

void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(0.9, 0.9, 0.9, 0);
    loadGlobalCoord();
         
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
                glTranslated(0, -20, 0);
                
                glPushMatrix();
                {
                    glRotatef(90, 1, 0, 0);
                    qobj = gluNewQuadric();
                    gluQuadricDrawStyle(qobj, GLU_FILL);
                    gluCylinder(qobj, 28.7, 20, 20, 100, 100);
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
                glTranslated(0, -20, 0);
                
                glPushMatrix();
                {
                    glRotatef(90, 1, 0, 0);
                    qobj = gluNewQuadric();
                    gluQuadricDrawStyle(qobj, GLU_FILL);
                    gluCylinder(qobj, 28.7, 20, 20, 100, 100);
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
    
    glutSwapBuffers();
}

void reshape(int width, int height) {
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45, (GLdouble)width/(GLdouble)height, 1.0, 5000.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void keyboard(unsigned char key, int x, int y) {
    switch (key) {
    case 27:
        exit(0);
        break;
    default:
        break;
    }
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

void init() {
    glEnable(GL_DEPTH_TEST);
    
}

int main(int argc, char * argv[]) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(ORIGINAL_WIDTH, ORIGINAL_HEIGHT);
    glutCreateWindow("HW1_2017-11522");
    init();
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);
    glutMouseFunc(glutMouse);
    glutMotionFunc(glutMotion);
    glutTimerFunc(timeStep, timer, 0);
    glutMainLoop();
    return 0;
}
