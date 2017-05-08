#include "Viewer.h"

using namespace std;
MapBuilder mapbuild;



static int year=0,day=0;

GLdouble rotationX = 0;
GLdouble rotationY = 0;

GLdouble translationX = 0;
GLdouble translationY = 0;
GLdouble translationZ = 0;

/*
* Variables for keeping track of dragging of the mouse.
*/
enum MouseDragMode {None, Zoom, Translate, Rotate};

MouseDragMode mouseDrag = None;
GLdouble startX;
GLdouble startY;

/*
* Width and height of the window.
*/
int height=480;
int width=640;

#define WIDTH 640
#define HEIGHT 480
#define N_FRAME WIDTH*HEIGHT
#define TRI_N_FRAME 3*N_FRAME

void testMap()
{
	string pcdf("D:\\tmpfiles\\1.pcd");
	unsigned char* pimg = new unsigned char[TRI_N_FRAME];
	float* pver = new float[TRI_N_FRAME];
	
	mapbuild.loadpcfromfile(pcdf);
	mapbuild.fromPCtoCells(mapbuild.global_map);
	
	MapBuilder* pMap=&mapbuild;
	mapbuild.FromCell2Triangles(pimg,pver);
	//mapbuild.FromCell2TrianglesForCache(pimg,pver);
	mapbuild.Push2OpenGL(pimg,pver);

	delete []pimg;
	delete []pver;
	//if(mapbuild.CreateRenderList()==-1)
		//cout<<"failed to create RenderList!"<<endl;
}

void init()
{
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
//	glOrtho(0.0,1.0,0.0,1.0,-1.0,1.0);
	testMap();

	glClearColor(0,0,0,0);
	//glShadeModel(GL_FLAT);
	glShadeModel(GL_SMOOTH);
}

void reshape(int w,int h)
{
	glViewport (0, 0, (GLsizei) w, (GLsizei) h);
	glMatrixMode (GL_PROJECTION);
	glLoadIdentity ();
	gluPerspective(45.0,(GLfloat)w/(GLfloat)h,1.0,100.0);
	glMatrixMode (GL_MODELVIEW);
	glLoadIdentity ();
	gluLookAt(0.0,0.0,-5.0,
		0.0,0.0,0.0,
		0.0,-1.0,0.0);
}

void display()
{
	//glColor3f(1.0,1.0,1.0);
	/*glBegin(GL_POLYGON);
	glVertex3f(0.25,0.25,0.0);
	glVertex3f(0.75,0.25,0.0);
	glVertex3f(0.75,0.75,0.0);
	glVertex3f(0.25,0.25,0.0);
	glEnd();*/
	
	glClear(GL_COLOR_BUFFER_BIT);
	glColor3f(1.0,1.0,1.0);
	glPushMatrix();

	glTranslated(translationX, translationY, translationZ);
	glRotated(rotationX, 1, 0, 0);
	glRotated(rotationY, 0, 1, 0);


	glBegin(GL_LINES); // x axis
	glColor3f(1.0,0.0,0.0);
	glVertex3f(0.0,0,0);
	glVertex3f(1.0,0,0);

	glColor3f(0.0,1.0,0.0); // y axis
	glVertex3f(0.0,0,0);
	glVertex3f(0.0,1,0);

	glColor3f(0.0,0.0,1.0); // z axis
	glVertex3f(0.0,0,0);
	glVertex3f(0.0,0,1);
	glEnd();

	//mapbuild.RenderPoints();
	//mapbuild.RenderTriangle();
	mapbuild.RenderTriangle2();

	glPopMatrix();
	glutSwapBuffers();
	glFlush();
}


void keyboard(unsigned char key,int x,int y)
{
	switch (key)
	{
	case 'd':
		day=(day+10)%360;
		glutPostRedisplay();
		break;
	case 'D':
		day=(day-10)%360;
		glutPostRedisplay();
		break;
	case 'y':
		//OpenNI_cb(NULL,NULL,NULL);
		year=(year+5)%360;
		glutPostRedisplay();
		break;
	case 'Y':
		day=(year-5)%360;
		glutPostRedisplay();
		break;
	default:
		break;

	}
}


void mouse(int button, int state, int x, int y)
{
	startX = x;
	startY = y;

	// Select the mouse drag mode.
	int mods = glutGetModifiers();
	mouseDrag =
		button != GLUT_LEFT_BUTTON || state == GLUT_UP ? None :
		GLUT_ACTIVE_CTRL & mods ? Zoom :
		GLUT_ACTIVE_SHIFT & mods ? Translate :
		Rotate;
}

/*
* Mouse movement.
*/
void motion(int x, int y)
{
	GLdouble dx = x - startX;
	GLdouble dy = y - startY;

	// Update the camera transformation.
	switch (mouseDrag) {
		case Rotate:
			rotationY += 360 * dx / width;
			rotationX += 360 * dy / height;
			break;
		case Zoom:
			translationZ += 100 * dy / height;
			break;
		case Translate:
			translationX += 100 * dx / width;
			translationY += - 100 * dy / height;
			break;
		case None:
			break;
	}

	startX = x;
	startY = y;

	glutPostRedisplay();
}



int main(int argc, char** argv)
{
	glutInit(&argc,argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
	glutInitWindowSize(640, 480); 
	glutInitWindowPosition(100, 100);
	glutCreateWindow("hello");
	init();

	
	glutDisplayFunc(display);
	glutReshapeFunc(reshape);
	glutKeyboardFunc(keyboard);
	glutMouseFunc(mouse);
	glutMotionFunc(motion);

	glutMainLoop();
	return 0;
}

//void testINndex()
//{
//	MapBuilder test;
//	float x,y,z;
//	int index=0;
//	int debug_t =1000;
//	for(x=-5.96;x<6;x+=0.04)
//		for(y=-1.96;y<2;y+=0.04)
//			for(z=-5.96;z<6;z+=0.04)
//			{
//				int getindex = test.getIndexCell(x,y,z);
//				if(z>0)
//				{
//					z=z;
//				}
//				if(index!=getindex)
//				{
//					cout<<"error at (x,y,z): "<<x<<", "<<y<<", "<<z<<endl;
//					cout<<"index= "<<index<<" "<<"getindex= "<<getindex<<endl;
//					index++;
//				}
//				if(index>debug_t)
//				{
//					debug_t *=2;
//				}
//				index++;
//			}
//		cout<<"index= "<<index<<endl;
//		cout<<"out loop at (x,y,z): "<<x<<", "<<y<<", "<<z<<endl;
//}
//
//void main()
//{
//	testINndex();
//	getchar();
//	return;
//}