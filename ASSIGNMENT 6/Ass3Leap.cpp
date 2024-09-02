#include <iostream>
#include <LeapC.h>
#include <GL/freeglut.h>
#include <Windows.h>

LEAP_CONNECTION connection;
const LEAP_TRACKING_EVENT* lastFrame = nullptr;
float rotationAngle = 0.0f; // Angle to rotate the cube

// Function to draw a line between two points
void drawLine(float x1, float y1, float z1, float x2, float y2, float z2) {
    glBegin(GL_LINES);
    glVertex3f(x1 * 0.001f, y1 * 0.001f, z1 * 0.001f);
    glVertex3f(x2 * 0.001f, y2 * 0.001f, z2 * 0.001f);
    glEnd();
}

// Function to draw a sphere at a given position
void drawSphere(float x, float y, float z, float radius) {
    glPushMatrix();
    glTranslatef(x * 0.001f, y * 0.001f, z * 0.001f);
    glutSolidSphere(radius, 20, 20);
    glPopMatrix();
}

// Function to draw an arrow at a given position with direction
void drawArrow(float x, float y, float z, float dx, float dy, float dz) {
    glPushMatrix();
    glTranslatef(x, y, z);
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(dx, dy, dz);
    glEnd();
    glTranslatef(dx, dy, dz);
    glutSolidCone(0.01, 0.02, 10, 10);
    glPopMatrix();
}

// Function to draw the hand based on tracking data
void drawHand(const LEAP_HAND* hand) {
    glColor3f(0.0f, 1.0f, 0.0f); // Green color for the hand

    // Draw the palm
    drawSphere(hand->palm.position.x, hand->palm.position.y, hand->palm.position.z, 0.02f);

    // Draw fingers
    for (int i = 0; i < 5; ++i) {
        const LEAP_DIGIT& digit = hand->digits[i];
        const LEAP_BONE* bones = digit.bones;
        for (int j = 0; j < 4; ++j) {
            drawLine(bones[j].prev_joint.x, bones[j].prev_joint.y, bones[j].prev_joint.z,
                bones[j].next_joint.x, bones[j].next_joint.y, bones[j].next_joint.z);
            drawSphere(bones[j].next_joint.x, bones[j].next_joint.y, bones[j].next_joint.z, 0.01f);
        }
    }
}

// Function to draw a 3D cube with different colors for each face
void drawCube() {
    glBegin(GL_QUADS);

    // Front face
    glColor3f(1.0f, 0.0f, 0.0f); // Red
    glVertex3f(-0.05f, -0.05f, 0.05f);
    glVertex3f(0.05f, -0.05f, 0.05f);
    glVertex3f(0.05f, 0.05f, 0.05f);
    glVertex3f(-0.05f, 0.05f, 0.05f);

    // Back face
    glColor3f(0.0f, 1.0f, 0.0f); // Green
    glVertex3f(-0.05f, -0.05f, -0.05f);
    glVertex3f(-0.05f, 0.05f, -0.05f);
    glVertex3f(0.05f, 0.05f, -0.05f);
    glVertex3f(0.05f, -0.05f, -0.05f);

    // Top face
    glColor3f(0.0f, 0.0f, 1.0f); // Blue
    glVertex3f(-0.05f, 0.05f, -0.05f);
    glVertex3f(-0.05f, 0.05f, 0.05f);
    glVertex3f(0.05f, 0.05f, 0.05f);
    glVertex3f(0.05f, 0.05f, -0.05f);

    // Bottom face
    glColor3f(1.0f, 1.0f, 0.0f); // Yellow
    glVertex3f(-0.05f, -0.05f, -0.05f);
    glVertex3f(0.05f, -0.05f, -0.05f);
    glVertex3f(0.05f, -0.05f, 0.05f);
    glVertex3f(-0.05f, -0.05f, 0.05f);

    // Right face
    glColor3f(1.0f, 0.0f, 1.0f); // Magenta
    glVertex3f(0.05f, -0.05f, -0.05f);
    glVertex3f(0.05f, 0.05f, -0.05f);
    glVertex3f(0.05f, 0.05f, 0.05f);
    glVertex3f(0.05f, -0.05f, 0.05f);

    // Left face
    glColor3f(0.0f, 1.0f, 1.0f); // Cyan
    glVertex3f(-0.05f, -0.05f, -0.05f);
    glVertex3f(-0.05f, -0.05f, 0.05f);
    glVertex3f(-0.05f, 0.05f, 0.05f);
    glVertex3f(-0.05f, 0.05f, -0.05f);

    glEnd();
}

// OpenGL display function
void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    gluLookAt(0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);

    // Apply rotation to the cube
    glPushMatrix();
    glRotatef(rotationAngle, 0.0f, 1.0f, 0.0f);
    drawCube();
    glPopMatrix();

    // Draw rotation arrows
    glColor3f(1.0f, 1.0f, 1.0f); // White color for arrows
    drawArrow(-0.2f, 0.0f, 0.0f, -0.1f, 0.0f, 0.0f); // Left arrow
    drawArrow(0.2f, 0.0f, 0.0f, 0.1f, 0.0f, 0.0f); // Right arrow

    if (lastFrame && lastFrame->nHands > 0) {
        for (uint32_t h = 0; h < lastFrame->nHands; ++h) {
            drawHand(&lastFrame->pHands[h]);
        }
    }

    glutSwapBuffers();
}

// OpenGL idle function
void idle() {
    LEAP_CONNECTION_MESSAGE msg;
    eLeapRS result = LeapPollConnection(connection, 1000, &msg);
    if (result == eLeapRS_Success && msg.type == eLeapEventType_Tracking) {
        lastFrame = msg.tracking_event;

        // Check for hand rotation to update rotationAngle
        if (lastFrame->nHands > 0) {
            const LEAP_HAND* hand = &lastFrame->pHands[0];
            float yaw = hand->palm.orientation.z; // Using the z component of palm orientation for yaw
            rotationAngle += yaw * 180.0f / 3.14159f; // Convert radians to degrees and accumulate
        }
    }
    glutPostRedisplay();
}

// Initialize Leap Motion connection
void initLeapMotion() {
    eLeapRS result = LeapCreateConnection(nullptr, &connection);
    if (result != eLeapRS_Success) {
        std::cerr << "Failed to create connection: " << result << std::endl;
        exit(1);
    }

    result = LeapOpenConnection(connection);
    if (result != eLeapRS_Success) {
        std::cerr << "Failed to open connection: " << result << std::endl;
        exit(1);
    }

    std::cout << "Connection to Ultraleap sensor established successfully!" << std::endl;
}

// OpenGL initialization
void initOpenGL(int argc, char** argv) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutCreateWindow("Ultraleap Hand Tracking Visualization");
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING); // Ensure lighting is disabled
    glutDisplayFunc(display);
    glutIdleFunc(idle);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0, 1.0, 0.1, 100.0);
    glMatrixMode(GL_MODELVIEW);
}

int main(int argc, char** argv) {
    initLeapMotion();
    initOpenGL(argc, argv);
    glutMainLoop();
    return 0;
}
