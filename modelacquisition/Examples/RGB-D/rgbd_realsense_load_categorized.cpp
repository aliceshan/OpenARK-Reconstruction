/*
Offline version of 3D reconstruction with categorization
- Loads categorized key frames
- 3D dense model reconstruction through TSDF 
- Supports much larger and accurate reconstruction space than the uncatergorizaed offline version
- Camera calibration file required
*/

#include <iostream>
#include <algorithm>
#include <thread>
#include <set>
#include <vector>


#include <stdio.h>
#include <sys/types.h>
#include <dirent.h>


#include <GL/glew.h>
#include <GL/glut.h>

#include <opencv2/opencv.hpp>

#include <SaveFrame.h>
#include <PointCloudGenerator.h>

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

std::thread *app;
std::string settingsFile;
std::string directoryName;

using namespace std;

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

    gluPerspective(45.0, (float) window_width / window_height, 10.0f, 100.0);

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

set<string> getFiles(string filename){
    cout << "getting names from: " << filename << endl;
    set<string> files;
    DIR *dp;
    int i = 0;
    struct dirent *ep;     
    dp = opendir (filename.c_str());

    if (dp != NULL) {
        while ((ep = readdir (dp))) {
            string name = ep -> d_name;
            if (name.length() > 2) {
                files.insert(name);
                i++;
            }
        }
        (void) closedir (dp);
    }
    else {
        perror ("Couldn't open the directory");
    }
    
    printf("There's %d files in the current directory.\n", i);
    return files;
}



vector<float> getOrigin(string origin) {
    cout << "THIS ORIGIN: " << origin << endl;
    size_t pos = 0;
    string delimiter = "_";
    vector<float> originF;
    int i = 0;
    while ((pos = origin.find(delimiter)) != string::npos) {
        originF.push_back(atof(origin.substr(0, pos).c_str()));
        origin.erase(0, pos + delimiter.length());
        i++;
    }
    originF.push_back(atof(origin.c_str()));
    return originF;
    
}


void application_thread() {

    set<string> blocks = getFiles(directoryName);

    for (string origin: blocks) {  

        vector<float> originF = getOrigin(origin);
        
        // Create saveFrame. It loads from timestamp, RGB image, depth image folders to retrieve key frames in the current block
        ark::SaveFrame *saveFrame = new ark::SaveFrame(directoryName + origin + "/");
        // Create pointCloudGenerator (TSDF). It initializes all system threads and gets ready to process frames (RBG and Depth).
        ark::PointCloudGenerator *pointCloudGenerator = new ark::PointCloudGenerator(settingsFile, 
            originF.at(0), originF.at(1), originF.at(2));

        pointCloudGenerator->Start();

        set<string> frames = getFiles(directoryName + origin + "/RGB/");

        if (frames.size() < 3) {
            continue;
        }

        set<int> tframes;
        for (string frameC: frames) {
            int tframe_ = atoi(frameC.substr(0, frameC.find(".")).c_str());
            tframes.insert(tframe_);
        }


        for (int tframe: tframes) {


            ark::RGBDFrame frame = saveFrame->frameLoad(tframe);

            cv::cvtColor(frame.imRGB, frame.imRGB, cv::COLOR_BGR2RGB);

            pointCloudGenerator->OnKeyFrameAvailable(frame);

        }

        pointCloudGenerator->RequestStop();
        // Save the model in the current clock in a ply file
        pointCloudGenerator->SavePly();
        pointCloudGenerator->ClearTSDF();
        delete pointCloudGenerator;
        delete saveFrame;
    }
}

void keyboard_func(unsigned char key, int x, int y) {
    if (key == ' ') {
        if (!stop) {
            app = new thread(application_thread);
            stop = !stop;
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
        cerr << endl << "Usage: ./rgbd_realsense path_to_frames path_to_settings" << endl;
        return 1;
    }

    settingsFile = argv[2];

    directoryName = argv[1];
    directoryName += "/frames_categorized/";

    // Main loop
    application_thread();

    delete app;
    return EXIT_SUCCESS;
}