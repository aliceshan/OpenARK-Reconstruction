#include <iostream>
#include <algorithm>
#include <string>
#include <thread>
#include <cstring>


#include <GL/glew.h>
#include <GL/glut.h>

#include <opencv2/opencv.hpp>

#include <ORBSLAMSystem.h>
#include <PointCloudGenerator.h>

#include <libfreenect/libfreenect.h>
#include <libfreenect/libfreenect_sync.h>

//OpenGL global variable
float window_width = 800;
float window_height = 800;
float xRot = 15.0f;
float yRot = 0.0f;
float xTrans = 0.0;
float yTrans = 0;
float zTrans = -35.0;
int ox;
int oy;
int buttonState;
float xRotLength = 0.0f;
float yRotLength = 0.0f;
bool wireframe = false;
bool stop = false;

ark::PointCloudGenerator *pointCloudGenerator;
ark::ORBSLAMSystem *slam;
thread *app;

using namespace cv;
using namespace std;

int frame_id = 0;

void freenect_sync_get_depth_cv(Mat &imD, int index)
{
    imD = Mat::zeros(480,640,CV_16UC1);
    static char *data = nullptr;
    unsigned int timestamp;
    if (freenect_sync_get_depth((void**)&data, &timestamp, index, FREENECT_DEPTH_REGISTERED))
        return;
    memcpy(imD.data,data,640*480*2);
    return;
}

void freenect_sync_get_rgb_cv(Mat &imRGB, int index)
{
    imRGB = Mat::zeros(480,640,CV_8UC3);
    static char *data = nullptr;
    unsigned int timestamp;
    if (freenect_sync_get_video((void**)&data, &timestamp, index, FREENECT_VIDEO_RGB))
        return;
    memcpy(imRGB.data,data,640*480*3);
    return;
}

void draw_box(float ox, float oy, float oz, float width, float height, float length) {
    glLineWidth(1.0f);
    glColor3f(1.0f, 1.0f, 1.0f);

    glBegin(GL_LINES);

    glVertex3f(ox, oy, oz);
    glVertex3f(ox + width, oy, oz);

    glVertex3f(ox, oy, oz);
    glVertex3f(ox, oy + height, oz);

    glVertex3f(ox, oy, oz);
    glVertex3f(ox, oy, oz + length);

    glVertex3f(ox + width, oy, oz);
    glVertex3f(ox + width, oy + height, oz);

    glVertex3f(ox + width, oy + height, oz);
    glVertex3f(ox, oy + height, oz);

    glVertex3f(ox, oy + height, oz + length);
    glVertex3f(ox, oy, oz + length);

    glVertex3f(ox, oy + height, oz + length);
    glVertex3f(ox, oy + height, oz);

    glVertex3f(ox + width, oy, oz);
    glVertex3f(ox + width, oy, oz + length);

    glVertex3f(ox, oy, oz + length);
    glVertex3f(ox + width, oy, oz + length);

    glVertex3f(ox + width, oy + height, oz);
    glVertex3f(ox + width, oy + height, oz + length);

    glVertex3f(ox + width, oy + height, oz + length);
    glVertex3f(ox + width, oy, oz + length);

    glVertex3f(ox, oy + height, oz + length);
    glVertex3f(ox + width, oy + height, oz + length);

    glEnd();
}

void draw_origin(float length) {
    glLineWidth(1.0f);
    glColor3f(1.0f, 1.0f, 1.0f);

    glBegin(GL_LINES);

    glVertex3f(0.f,0.f,0.f);
    glVertex3f(length,0.f,0.f);

    glVertex3f(0.f,0.f,0.f);
    glVertex3f(0.f,length,0.f);

    glVertex3f(0.f,0.f,0.f);
    glVertex3f(0.f,0.f,length);

    glEnd();
}


void init() {
    glewInit();

    glViewport(0, 0, window_width, window_height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    gluPerspective(45.0, (float) window_width / window_height, 0.1f, 300.0);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glTranslatef(0.0f, 0.0f, -3.0f);
}

void display_func() {
    glEnable(GL_POINT_SMOOTH);
    glEnable(GL_DEPTH_TEST);
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (wireframe)
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    else
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    glPushMatrix();

    if (buttonState == 1) {
        xRot += (xRotLength - xRot) * 0.1f;
        yRot += (yRotLength - yRot) * 0.1f;
    }

    glTranslatef(xTrans, yTrans, zTrans);
    glRotatef(xRot, 1.0f, 0.0f, 0.0f);
    glRotatef(yRot, 0.0f, 1.0f, 0.0f);

    pointCloudGenerator->Render();

    draw_origin(4.f);

    glPopMatrix();
    glutSwapBuffers();

}

void idle_func() {
    glutPostRedisplay();
}

void reshape_func(GLint width, GLint height) {
    window_width = width;
    window_height = height;

    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    gluPerspective(45.0, (float) width / height, 0.001, 100.0);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glTranslatef(0.0f, 0.0f, -3.0f);
}

void application_thread() {
    slam->Start();
    pointCloudGenerator->Start();

    // Main loop
    int tframe = 1;
    while (!slam->IsRunning()) {
        Mat imRGB;
        freenect_sync_get_rgb_cv(imRGB, 0);
        if (imRGB.cols==0) {
            printf("Error: Kinect not connected?\n");
            return;
        }
        Mat imD;
        freenect_sync_get_depth_cv(imD, 0);
        if (imD.cols==0) {
            printf("Error: Kinect not connected?\n");
            return;
        }
	//std::cout << "TEST1" << std::endl << std::flush;
        //imshow("Depth", imD*5);
	//cv::waitKey(1);
	//std::cout << "TEST2" << std::endl << std::flush;
        // Pass the image to the SLAM system
        
        slam->PushFrame(imRGB, imD, tframe);
    }
}

void keyboard_func(unsigned char key, int x, int y) {
    if (key == ' ') {
        if (!stop) {
            app = new thread(application_thread);
            stop = !stop;
        } else {
            slam->RequestStop();
            pointCloudGenerator->RequestStop();
        }
    }

    if (key == 'w') {
        zTrans += 0.3f;
    }

    if (key == 's') {
        zTrans -= 0.3f;
    }

    if (key == 'a') {
        xTrans += 0.3f;
    }

    if (key == 'd') {
        xTrans -= 0.3f;
    }

    if (key == 'q') {
        yTrans -= 0.3f;
    }

    if (key == 'e') {
        yTrans += 0.3f;
    }

    if (key == 'p') {
        stringstream outputString;
        //pointCloudGenerator->SavePly("model_"+to_string(frame_id)+".ply");
        //pointCloudGenerator->SavePly("fi#le_file.ply", outputString);
        cout << "YAY" << endl;
        pointCloudGenerator->SavePly("file_file.ply", outputString);
        cout << outputString.str() << endl;

        /*string cmd = "./colorizer model_"+to_string(frame_id)+".ply 2048";
        system(cmd.c_str());
        frame_id++;*/
    }

    if (key == 'v') 
        wireframe = !wireframe;


    glutPostRedisplay();
}

void mouse_func(int button, int state, int x, int y) {
    if (state == GLUT_DOWN) {
        buttonState = 1;
    } else if (state == GLUT_UP) {
        buttonState = 0;
    }

    ox = x;
    oy = y;

    glutPostRedisplay();
}

void motion_func(int x, int y) {
    float dx, dy;
    dx = (float) (x - ox);
    dy = (float) (y - oy);

    if (buttonState == 1) {
        xRotLength += dy / 5.0f;
        yRotLength += dx / 5.0f;
    }

    ox = x;
    oy = y;

    glutPostRedisplay();
}

int main(int argc, char **argv) {
    if (argc != 3) {
        cerr << endl << "Usage: ./rgbd_realsense path_to_vocabulary path_to_settings" << endl;
        return 1;
    }

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
    glutInitWindowSize(window_width, window_height);
    (void) glutCreateWindow("GLUT Program");

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    pointCloudGenerator = new ark::PointCloudGenerator(argv[2]);
    slam = new ark::ORBSLAMSystem(argv[1], argv[2], ark::ORBSLAMSystem::RGBD, true);

    slam->AddKeyFrameAvailableHandler([pointCloudGenerator](const ark::RGBDFrame &keyFrame) {
        return pointCloudGenerator->OnKeyFrameAvailable(keyFrame);
    }, "PointCloudFusion");

    init();
    glutSetWindowTitle("OpenARK 3D Reconstruction");
    glutDisplayFunc(display_func);
    glutReshapeFunc(reshape_func);
    glutIdleFunc(idle_func);
    glutMouseFunc(mouse_func);
    glutMotionFunc(motion_func);
    glutKeyboardFunc(keyboard_func);
    glutMainLoop();

    delete pointCloudGenerator;
    delete slam;
    delete app;

    return EXIT_SUCCESS;
}
