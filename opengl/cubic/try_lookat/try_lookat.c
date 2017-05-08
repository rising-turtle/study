#include <GL/glut.h>
#include <GL/glu.h>
#include <GL/gl.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>

void drawaxes(); 
void DrawCube(float, float, float);

void display()
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glColor3f(0.0, 1.0, 0.0);
  
  glMatrixMode(GL_PROJECTION); 
  glLoadIdentity();
  // gluLookAt(0, 0, 0.2, 0, 0, -1, 0, 1,0);
  // glLoadIdentity();
  glOrtho(-1.5, 1.5, -1.5, 1.5, -2, 2); // glOrtho project objects from [-x, +x, -y, y, -z, z] forwarding the camera direction
  // gluPerspective(45.0, 1, 0.1, 10.0); // project models into camera screen 

  glMatrixMode(GL_MODELVIEW); 

  glLoadIdentity();
  // gluLookAt(0, 0, 1.2, 0, 0, -10, 0, 1,0); // gluLookAt is just a combination of translation and rotation for the objects
  
  glTranslatef(0, 0, -1);

  glPushMatrix();
  glScalef(0.6, 0.5, 0.5);

  glTranslatef(0.5, 0.5, 0.5);
  DrawCube(0 ,0 ,0);
  glPopMatrix();
  
  // glLoadIdentity();
  drawaxes();

  glutSwapBuffers();
}


int main(int argc, char* argv[])
{
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH); // double: buffer, used to swap, depth: z-buffer 
    glutInitWindowSize(800, 800);
    glutInitWindowPosition(200, 200);
    glutCreateWindow("3D Object in OpenGL");
    glClearColor(0.0, 0.0, 0.0, 0.0);
    glutDisplayFunc(display);
    // glutReshapeFunc(Resize);
    
    glEnable(GL_DEPTH_TEST);   // try to enable z-buffer 
    glDepthFunc(GL_TRUE);

    glutMainLoop();
    return 0; 
}


void drawaxes(void)
{
    glColor3ub(255, 0, 0);
    glBegin(GL_LINE_STRIP);
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(1.0, 0.0, 0.0);
    glVertex3f(0.75, 0.25, 0.0);
    glVertex3f(0.75, -0.25, 0.0);
    glVertex3f(1.0, 0.0, 0.0);
    glVertex3f(0.75, 0.0, 0.25);
    glVertex3f(0.75, 0.0, -0.25);
    glVertex3f(1.0, 0.0, 0.0);
    glEnd();
    glColor3ub(0, 255, 0);
    glBegin(GL_LINE_STRIP);
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(0.0, 1.0, 0.0);
    glVertex3f(0.0, 0.75, 0.25);
    glVertex3f(0.0, 0.75, -0.25);
    glVertex3f(0.0, 1.0, 0.0);
    glVertex3f(0.25, 0.75, 0.0);
    glVertex3f(-0.25, 0.75, 0.0);
    glVertex3f(0.0, 1.0, 0.0);
    glEnd();
    glColor3ub(0, 0, 255);
    glBegin(GL_LINE_STRIP);
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(0.0, 0.0, 1.0);
    glVertex3f(0.25, 0.0, 0.75);
    glVertex3f(-0.25, 0.0, 0.75);
    glVertex3f(0.0, 0.0, 1.0);
    glVertex3f(0.0, 0.25, 0.75);
    glVertex3f(0.0, -0.25, 0.75);
    glVertex3f(0.0, 0.0, 1.0);
    glEnd();
 
    glColor3ub(255, 255, 0);
    glRasterPos3f(1.1, 0.0, 0.0);
    glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, 'x');
    glRasterPos3f(0.0, 1.1, 0.0);
    glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, 'y');
    glRasterPos3f(0.0, 0.0, 1.1);
    glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, 'z');
}


void DrawCube(float xPos, float yPos, float zPos)
{
        glPushMatrix();
        // glTranslatef(xPos, yPos, zPos);
        // glBegin(GL_POLYGON);
        glBegin(GL_QUADS); // has to be GL_QUADS, to draw a rectangle in 3D space, not GL_POLYGON

                /*      This is the top face*/
                glColor3f(1.0f, 1.0f, 0.0f);     // Yellow

                glVertex3f(0.0f, 0.0f, 0.0f);
                glVertex3f(0.0f, 0.0f, -1.0f);
                glVertex3f(-1.0f, 0.0f, -1.0f);
                glVertex3f(-1.0f, 0.0f, 0.0f);

                /*      This is the front face*/
                glColor3f(1.0f, 0.5f, 0.0f);     // Orange
                glVertex3f(0.0f, 0.0f, 0.0f);
                glVertex3f(-1.0f, 0.0f, 0.0f);
                glVertex3f(-1.0f, -1.0f, 0.0f);
                glVertex3f(0.0f, -1.0f, 0.0f);

                /*      This is the right face*/
                glColor3f(0.0f, 1.0f, 0.0f);     // Green

                glVertex3f(0.0f, 0.0f, 0.0f);
                glVertex3f(0.0f, -1.0f, 0.0f);
                glVertex3f(0.0f, -1.0f, -1.0f);
                glVertex3f(0.0f, 0.0f, -1.0f);

          // glEnd(); 

          //      glBegin(GL_QUADS); 
                /*      This is the left face*/
                glColor3f(1.0f, 0.0f, 0.0f);     // Red

                // glVertex3f(0, 0,0);
                glVertex3f(-1.0f, 0.0f, 0.0f);
                glVertex3f(-1.0f, 0.0f, -1.0f);
                glVertex3f(-1.0f, -1.0f, -1.0f);
                glVertex3f(-1.0f, -1.0f, 0.0f);

                /*      This is the bottom face*/
                glColor3f(1.0f, 0.0f, 1.0f);     // Magenta
                // glVertex3f(0, 0,0);
                glVertex3f(0.0f, -1.0f, 0.0f);
                glVertex3f(0.0f, -1.0f, -1.0f);
                glVertex3f(-1.0f, -1.0f, -1.0f);
                glVertex3f(-1.0f, -1.0f, 0.0f);

                /*      This is the back face*/
                glColor3f(0.0f, 0.0f, 1.0f);     // Blue

                // glVertex3f(0, 0,0);
                glVertex3f(0.0f, 0.0f, -1.0f);
                glVertex3f(-1.0f, 0.0f, -1.0f);
                glVertex3f(-1.0f, -1.0f, -1.0f);
                glVertex3f(0.0f, -1.0f, -1.0f);

        glEnd();
        glPopMatrix();
}

