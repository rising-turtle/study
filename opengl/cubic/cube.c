#include <GL/glut.h>
#include <GL/glu.h>
#include <GL/gl.h>
#include <stdbool.h>
#include <stdio.h>

/*      Here we find a few global variables. While
        i don't really like to use global variables,
        i found them very handy for this particular
        program. These variables will control angles,
        fullscreen, and the global device context.
*/

float angle = 0.0f;
float legAngle[2] = {0.0f, 0.0f};
float armAngle[2] = {0.0f, 0.0f};
bool fullScreen = false;

/*      Function:       DrawCube
        Purpose:        As the name would suggest, this is
                                the function for drawing the cubes.
*/

void DrawCube(float xPos, float yPos, float zPos)
{
        glPushMatrix();
        glBegin(GL_POLYGON);

                /*      This is the top face*/
                glVertex3f(0.0f, 0.0f, 0.0f);
                glVertex3f(0.0f, 0.0f, -1.0f);
                glVertex3f(-1.0f, 0.0f, -1.0f);
                glVertex3f(-1.0f, 0.0f, 0.0f);

                /*      This is the front face*/
                glVertex3f(0.0f, 0.0f, 0.0f);
                glVertex3f(-1.0f, 0.0f, 0.0f);
                glVertex3f(-1.0f, -1.0f, 0.0f);
                glVertex3f(0.0f, -1.0f, 0.0f);

                /*      This is the right face*/
                glVertex3f(0.0f, 0.0f, 0.0f);
                glVertex3f(0.0f, -1.0f, 0.0f);
                glVertex3f(0.0f, -1.0f, -1.0f);
                glVertex3f(0.0f, 0.0f, -1.0f);

                /*      This is the left face*/
                glVertex3f(-1.0f, 0.0f, 0.0f);
                glVertex3f(-1.0f, 0.0f, -1.0f);
                glVertex3f(-1.0f, -1.0f, -1.0f);
                glVertex3f(-1.0f, -1.0f, 0.0f);

                /*      This is the bottom face*/
                glVertex3f(0.0f, 0.0f, 0.0f);
                glVertex3f(0.0f, -1.0f, -1.0f);
                glVertex3f(-1.0f, -1.0f, -1.0f);
                glVertex3f(-1.0f, -1.0f, 0.0f);

                /*      This is the back face*/
                glVertex3f(0.0f, 0.0f, 0.0f);
                glVertex3f(-1.0f, 0.0f, -1.0f);
                glVertex3f(-1.0f, -1.0f, -1.0f);
                glVertex3f(0.0f, -1.0f, -1.0f);

        glEnd();
        glPopMatrix();
}

/*      Function:       DrawArm
        Purpose:        This function draws the arm
                                for the robot.
*/

void DrawArm(float xPos, float yPos, float zPos)
{
        glPushMatrix();

                /*      Sets color to red*/
                glColor3f(1.0f, 0.0f, 0.0f);
                glTranslatef(xPos, yPos, zPos);

                /*      Creates 1 x 4 x 1 cube*/
                glScalef(1.0f, 4.0f, 1.0f);
                DrawCube(0.0f, 0.0f, 0.0f);

        glPopMatrix();
}

/*      Function:       DrawHead
        Purpose:        This function will create the
                                head for the robot.
*/

void DrawHead(float xPos, float yPos, float zPos)
{
        glPushMatrix();

                /*      Sets color to white*/
                glColor3f(1.0f, 1.0f, 1.0f);
                glTranslatef(xPos, yPos, zPos);

                /*      Creates 2 x 2 x 2 cube*/
                glScalef(2.0f, 2.0f, 2.0f);
                DrawCube(0.0f, 0.0f, 0.0f);

        glPopMatrix();
}

/*      Function:       DrawTorso
        Purpose:        Function will do as suggested
                                and draw a torso for our robot.
*/

void DrawTorso(float xPos, float yPos, float zPos)
{
        glPushMatrix();

                /*      Sets color to blue*/
                glColor3f(0.0f, 0.0f, 1.0f);
                glTranslatef(xPos, yPos, zPos);

                /*      Creates 3 x 5 x 1 cube*/
                glScalef(3.0f, 5.0f, 1.0f);
                DrawCube(0.0f, 0.0f, 0.0f);

        glPopMatrix();
}

/*      Function:       DrawLeg
        Purpose:        Not to sound repetitve, but as suggested
                                this function will draw our robots legs.
*/

void DrawLeg(float xPos, float yPos, float zPos)
{
        glPushMatrix();

                /*      Sets color to yellow*/
                glColor3f(1.0f, 1.0f, 0.0f);
                glTranslatef(xPos, yPos, zPos);

                /*      Creates 1 x 5 x 1 cube*/
                glScalef(1.0f, 5.0f, 1.0f);
                DrawCube(0.0f, 0.0f, 0.0f);

        glPopMatrix();
}

/*      Function:       DrawRobot
        Purpose:        Function to draw our entire robot
*/

void DrawRobot(float xPos, float yPos, float zPos)
{
        /*      Variables for state of robots legs. True
                means the leg is forward, and False means
                the leg is back. The same applies to the
                robots arm states.
        */
        static bool leg1 = true;
        static bool leg2 = false;
        static bool arm1 = true;
        static bool arm2 = false;

        glPushMatrix();

                /*      This will draw our robot at the
                        desired coordinates.
                */
                glTranslatef(xPos, yPos, zPos);

                /*      These three lines will draw the
                        various components of our robot.
                */
                DrawHead(1.0f, 2.0f, 0.0f);
                DrawTorso(1.5f, 0.0f, 0.0f);
                glPushMatrix();


                /*      If the arm is moving forward we will increase
                        the angle; otherwise, we will decrease the
                        angle.
                */
                if (arm1)
                {
                        armAngle[0] = armAngle[0] + 1.0f;
                }
                else
                {
                        armAngle[0] = armAngle[0] - 1.0f;
                }

                /*      Once the arm has reached its max angle
                        in one direction, we want it to reverse
                        and change direction.
                */
                if (armAngle[0] >= 15.0f)
                {
                        arm1 = false;
                }
                if (armAngle[0] <= 15.0f)
                {
                        arm1 = true;
                }


                /*      Here we are going to move the arm away
                        from the torso and rotate. This will
                        create a walking effect.
                */
                glTranslatef(0.0f, -0.5f, 0.0f);
                glRotatef(armAngle[0], 1.0f, 0.0f, 0.0f);
                DrawArm(2.5f, 0.0f, -0.5f);

        glPopMatrix();

        glPushMatrix();


                /*      If the arm is moving forward we will increase
                        the angle, otherwise we will decrease the
                        angle
                */
                if (arm2)
                {
                        armAngle[1] = armAngle[1] + 1.0f;
                }
                else
                {
                        armAngle[1] = armAngle[1] - 1.0f;
                }

                /*      Here we are going to move the arm away
                        from the torso and rotate. This will
                        create a walking effect.
                */
                glTranslatef(0.0f, -0.5f, 0.0f);
                glRotatef(armAngle[1], 1.0f, 0.0f, 0.0f);
                DrawArm(-1.5f, 0.0f, -0.5f);

        glPopMatrix();

        /*      Now its time to rotate the legs relative to the
                robots position in the world, this is the first
                leg, ie the right one.
        */
        glPushMatrix();

                /*      If the leg is moving forward we will increase
                        the angle; otherwise, we will decrease the
                        angle.
                */
                if (leg1)
                {
                        legAngle[0] = legAngle[0] + 1.0f;
                }
                else
                {
                        legAngle[0] = legAngle[0] - 1.0f;
                }

                /*      Once the leg has reached its max angle
                        in one direction, we want it to reverse
                        and change direction.
                */
                if (legAngle[0] >= 15.0f)
                {
                        leg1 = false;
                }
                if (legAngle[0] <= -15.0f)
                {
                        leg1 = true;
                }


                /*      Here we are going to move the leg away
                        from the torso and rotate. This will
                        create a walking effect.
                */
                glTranslatef(0.0f, -0.5f, 0.0f);
                glRotatef(legAngle[0], 1.0f, 0.0f, 0.0f);


                /*      Time to draw the leg.
                */
                DrawLeg(-0.5f, -5.0f, -0.5f);

        glPopMatrix();

        /*      Same as above, for the left leg.
        */
        glPushMatrix();

                /*      If the leg is moving forward we will increase
                        the angle, otherwise we will decrease the
                        angle
                */
                if (leg2)
                {
                        legAngle[1] = legAngle[1] + 1.0f;
                }
                else
                {
                        legAngle[1] = legAngle[1] - 1.0f;
                }

                /*      Once the leg has reached its max angle
                        in one direction, we want it to reverse
                        and change direction.
                */
                if (legAngle[1] >= 15.0f)
                {
                        leg2 = false;
                }
                if (legAngle[1] <= -15.0f)
                {
                        leg2 = true;
                }

                /*      Here we are going to move the leg away
                        from the torso and rotate. This will
                        create a walking effect.
                */
                glTranslatef(0.0f, -0.5f, 0.0f);
                glRotatef(legAngle[1], 1.0f, 0.0f, 0.0f);
                DrawLeg(1.5f, -5.0f, -0.5f);

        glPopMatrix();

        glPopMatrix();

}

/*      Function:       Render
        Purpose:        This function will be responsible
                                for the rendering, got to love my
                                descriptive function names : )
*/
void Render()
{
  printf("in render! angle = %f\n", angle);
        /*      Enable depth testing
        */
        glEnable(GL_DEPTH_TEST);

        /*      Heres our rendering. Clears the screen
                to black, clear the color and depth
                buffers, and reset our modelview matrix.
        */
        glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glLoadIdentity();

        /*      Increase rotation angle counter
        */
        angle = angle + 1.0f;

        /*      Reset after we have completed a circle
        */
        if (angle >= 360.0f)
        {
                angle = 0.0f;
        }

        glPushMatrix();
                glLoadIdentity();

                /*      Move to 0,0,-30 , rotate the robot on
                        its y axis, draw the robot, and dispose
                        of the current matrix.
                */
                glTranslatef(0.0f, 0.0f, -30.0f);
                glRotatef(angle, 0.0f, 1.0f, 0.0f);
                DrawRobot(0.0f, 0.0f, 0.0f);
        glPopMatrix();

        glFlush();

        /*      Bring back buffer to foreground
        */
       // SwapBuffers(g_HDC);
       glutSwapBuffers();
}
 
void Resize(int width, int height)
{
  printf("in Resize()\n");
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    //gluPerspective(60.0, width/height, 0.1, 10.0);

    // glRotatef(rotationAngleZ, 0, 0, 1); // rotate around z axis 
    // glRotatef(rotationAngleY, 0, 1, 0); // rotate around y axis 
    // glRotatef(rotationAngleX, 1, 0, 0); // rotate around x axis

    // glOrtho(-3, 3, -1.5, 1.5, -1, 1000.0);
    // gluLookAt(-1.0, -1.0, 3.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0);
    glOrtho(-30, 30, -15, 15, -1, 1000.0);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}
 
int main(int argc, char **argv)
{
    
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
    glutInitWindowSize(400, 400);
    glutInitWindowPosition(200, 200);
    glutCreateWindow("3D Object in OpenGL");
    // Init();
    glClearColor(0.0, 0.0, 0.0, 0.0);
    // glMatrixMode(GL_PROJECTION);
    // glLoadIdentity();
    // gluLookAt(0, 0, 30.0, 0.0, 0.0, 0.0, 0, 1.0, 0.0);
    // glOrtho(-30, 30, -15, 15, -1, 1000.0);
    // gluPerspective(60.0, 1, 0.1, 10.0);

    glutDisplayFunc(Render);
    glutReshapeFunc(Resize);
    glutMainLoop();
    return 0;
}

