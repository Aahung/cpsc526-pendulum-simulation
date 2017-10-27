#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
import sys
import math

#  from pyquaternion import Quaternion    ## would be useful for 3D simulation

import numpy as np

window = 0  # number of the glut window
theta = 0.0
simTime = 0
dT = 0.01
simRun = True
RAD_TO_DEG = 180.0 / 3.1416
G = -9.8
initial_energy = 0
potential_energy_ground_height = -0.5


#####################################################
#### Link class, i.e., for a rigid body
#####################################################

class Link:

    color = [0, 0, 0]  # # draw color
    size = [1, 1, 1]  # # dimensions
    mass = 1.0  # # mass in kg
    izz = 1.0  # # moment of inertia about z-axis
    theta = 0  # # 2D orientation  (will need to change for 3D)
    omega = 0  # # 2D angular velocity
    posn = np.array([0.0, 0.0, 0.0])  # # 3D position (keep z=0 for 2D)
    vel = np.array([0.0, 0.0, 0.0])

    def kinetic_energy(self):
        return 0.5 * (self.mass * np.linalg.norm(self.vel) ** 2 + self.inertia() * np.linalg.norm(self.omega) ** 2)
    def potential_energy(self):
        return self.mass * -G * (self.posn[1] - potential_energy_ground_height)
    def calibrate_energy(self, factor):
        self.vel *= math.sqrt(factor)
        self.omega *= math.sqrt(factor)
    def radius(self):
        return self.size[1] / 2
    def inertia(self):
        return self.mass * self.radius() * self.radius()
    def draw(self):  # ## steps to draw a link
        glPushMatrix()  # # save copy of coord frame
        glTranslatef(self.posn[0], self.posn[1], self.posn[2])  # # move
        glRotatef(self.theta * RAD_TO_DEG, 0, 0, 1)  # # rotate
        glScale(self.size[0], self.size[1], self.size[2])  # # set size
        glColor3f(self.color[0], self.color[1], self.color[2])  # # set colour
        DrawCube()  # # draw a scaled cube
        glPopMatrix()  # # restore old coord frame


#####################################################
#### main():   launches app
#####################################################

def main():
    global window
    global link1, link2
    glutInit(sys.argv)
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH)  # display mode
    glutInitWindowSize(640, 480)  # window size
    glutInitWindowPosition(0, 0)  # window coords for mouse start at top-left
    window = glutCreateWindow('CPSC 526 Simulation Template')
    glutDisplayFunc(DrawWorld)  # register the function to draw the world

        # glutFullScreen()               # full screen

    glutIdleFunc(SimWorld)  # when doing nothing, redraw the scene
    glutReshapeFunc(ReSizeGLScene)  # register the function to call when window is resized
    glutKeyboardFunc(keyPressed)  # register the function to call when keyboard is pressed
    InitGL(640, 480)  # initialize window

    link1 = Link()
    link2 = Link()

    global kinetic_link, potential_link
    kinetic_link = Link()
    potential_link = Link()

    resetSim()

    glutMainLoop()  # start event processing loop


#####################################################
#### keyPressed():  called whenever a key is pressed
#####################################################

def resetSim():
    global link1, link2
    global simTime, simRun
    global initial_energy

    printf('Simulation reset\n')
    simRun = True
    simTime = 0

    link1.size = [0.04, 1.0, 0.12]
    link1.color = [1, 0.9, 0.9]
    link1.posn = np.array([0.0, 0.0, 0.0])
    link1.vel = np.array([0.0, 0.0, 0.0])
    link1.theta = math.pi / 4
    link1.omega = 0  # # radians per second

    link2.size = [0.04, 1.0, 0.12]
    link2.color = [0.9, 0.9, 1.0]
    link2.posn = np.array([1.0, 0.0, 0.0])
    link2.vel = np.array([0.0, 4.0, 0.0])
    link2.theta = -0.2
    link2.omega = 0  # # radians per second

    initial_energy = link1.kinetic_energy() + link1.potential_energy()

def calEnergy():
    global kinetic_link, potential_link
    kinetic_link.color = [1, 0.0, 0.0]
    potential_link.color = [0, 1, 0.0]
    kinetic_energy = link1.kinetic_energy()
    potential_energy = link1.potential_energy()
    kinetic_link.size = [0.04, kinetic_energy * 0.1, 0.12]
    potential_link.size = [0.04, potential_energy * 0.1, 0.12]
    bar_link_origin = np.array([0.0, 0.0, 0.0])
    potential_link.posn = np.array([1.0, 0.0, 0.0]) + np.array([0, potential_link.size[1] / 2, 0])
    kinetic_link.posn = np.array([1.0, 0.0, 0.0]) + np.array([0, potential_link.size[1] + kinetic_link.size[1] / 2, 0])

    total_energy = potential_energy + kinetic_energy
    link1.calibrate_energy(initial_energy / total_energy)

#####################################################
#### keyPressed():  called whenever a key is pressed
#####################################################

def keyPressed(key, x, y):
    global simRun
    ch = key.decode('utf-8')
    if ch == ' ':  # ### toggle the simulation
        if simRun == True:
            simRun = False
        else:
            simRun = True
    elif ch == chr(27):

                                 # ### ESC key

        sys.exit()
    elif ch == 'q':

                                 # ### quit

        sys.exit()
    elif ch == 'r':

                                 # ### reset simulation

        resetSim()


#####################################################
#### SimWorld():  simulates a time step
#####################################################

def SimWorld():
    global simTime, dT, simRun
    global link1, link2

    deltaTheta = 2.4
    if simRun == False:  # # is simulation stopped?
        return

            # ### solve for the equations of motion (simple in this case!)

    acc1 = np.array([0, -10, 0])  # ## linear acceleration = [0, -G, 0]
    acc2 = np.array([0, -10, 0])  # ## linear acceleration = [0, -G, 0]
    omega_dot1 = 0.0  # ## assume no angular acceleration
    omega_dot2 = 0.0  # ## assume no angular acceleration

            # ###  for the constrained one-link pendulum, and the 4-link pendulum,
            # ###  you will want to build the equations of motion as a linear system, and then solve that.
            # ###  Here is a simple example of using numpy to solve a linear system.

    rx = np.cos(link1.theta + math.pi / 2) * link1.radius()
    ry = np.sin(link1.theta + math.pi / 2) * link1.radius()
    a = np.array(
        [[link1.mass, 0, 0, -1, 0],
         [0, link1.mass, 0, 0, -1],
         [0, 0, link1.inertia(), ry, -rx],
         [-1, 0, ry, 0, 0],
         [0, -1, -rx, 0, 0]])
    b = np.array(
        [0,
         link1.mass * G,
         0,
         -link1.omega * link1.omega * rx,
         -link1.omega * link1.omega * ry])
    x = np.linalg.solve(a, b)
    acc1 = np.array([x[0], x[1], 0])
    omega_dot1 = x[2]  # ## assume no angular acceleration
    # print(x)   # [ -2.17647059  53.54411765  56.63235294]
            # ### explicit Euler integration to update the state

    link1.posn += link1.vel * dT
    link1.vel += acc1 * dT
    link1.theta += link1.omega * dT
    link1.omega += omega_dot1 * dT

    # correction for constrained point (fix point)
    posn_expected = np.array([link1.radius() * np.cos(math.pi * 0.75), link1.radius() * np.sin(math.pi * 0.75), 0])
    vel_expected = np.array([0, 0, 0])
    acc_expected = np.array([0, 0, 0])
    posn_real = np.array([link1.posn[0] + link1.radius() * np.cos(link1.theta + math.pi / 2), link1.posn[1] + link1.radius() * np.sin(link1.theta + math.pi / 2), 0])
    vel_real = np.array([link1.vel[0] - link1.radius() * link1.omega * np.sin(link1.theta + math.pi / 2), link1.vel[1] + link1.radius() * link1.omega * np.cos(link1.theta + math.pi / 2), 0])
    acc_real = np.array([acc1[0] - link1.radius() * omega_dot1 * np.sin(link1.theta + math.pi / 2), acc1[1] + link1.radius() * omega_dot1 * np.cos(link1.theta + math.pi / 2), 0])
    
    acc_correct = 3 * (posn_expected - posn_real) + 5 * (vel_expected - vel_real)
    link1.vel += acc_correct * dT
    print(posn_expected - posn_real)
    print(vel_expected - vel_real)


    # link2.posn += link2.vel * dT
    # link2.vel += acc2 * dT
    link2.theta = 3.1415926 / 6
    link2.omega += omega_dot2 * dT

    simTime += dT

            # ### draw the updated state
    calEnergy()
    DrawWorld()
    printf('simTime=%.2f\n', simTime)


#####################################################
#### DrawWorld():  draw the world
#####################################################

def DrawWorld():
    global link1, link2, kinetic_link, potential_link

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)  # Clear The Screen And The Depth Buffer
    glLoadIdentity()
    gluLookAt(
        1, 1, 5, # eyeX, eyeY, eyeZ
        0, 0, 0, # centerX, centerY, centerZ
        0, 1, 0,) # upX, upY, upZ

    DrawOrigin()
    link1.draw()
    # link2.draw()
    kinetic_link.draw()
    potential_link.draw()

    glutSwapBuffers()  # swap the buffers to display what was just drawn


#####################################################
#### initGL():  does standard OpenGL initialization work
#####################################################

def InitGL(Width, Height):  # We call this right after our OpenGL window is created.
    glClearColor(1.0, 1.0, 0.9, 0.0)  # This Will Clear The Background Color To Black
    glClearDepth(1.0)  # Enables Clearing Of The Depth Buffer
    glDepthFunc(GL_LESS)  # The Type Of Depth Test To Do
    glEnable(GL_DEPTH_TEST)  # Enables Depth Testing
    glEnable(GL_BLEND)
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
    glEnable(GL_LINE_SMOOTH)
    glShadeModel(GL_SMOOTH)  # Enables Smooth Color Shading
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()  # Reset The Projection Matrix
    gluPerspective(45.0, float(Width) / float(Height), 0.1, 100.0)
    glMatrixMode(GL_MODELVIEW)


#####################################################
#### ReSizeGLScene():    called when window is resized
#####################################################

def ReSizeGLScene(Width, Height):
    if Height == 0:  # Prevent A Divide By Zero If The Window Is Too Small
        Height = 1
    glViewport(0, 0, Width, Height)  # Reset The Current Viewport And Perspective Transformation
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45.0, float(Width) / float(Height), 0.1, 100.0)  # # 45 deg horizontal field of view, aspect ratio, near, far
    glMatrixMode(GL_MODELVIEW)


#####################################################
#### DrawOrigin():  draws RGB lines for XYZ origin of coordinate system
#####################################################

def DrawOrigin():
    glLineWidth(3.0)

    glColor3f(1, 0.5, 0.5)  # # light red x-axis
    glBegin(GL_LINES)
    glVertex3f(0, 0, 0)
    glVertex3f(1, 0, 0)
    glEnd()

    glColor3f(0.5, 1, 0.5)  # # light green y-axis
    glBegin(GL_LINES)
    glVertex3f(0, 0, 0)
    glVertex3f(0, 1, 0)
    glEnd()

    glColor3f(0.5, 0.5, 1)  # # light blue z-axis
    glBegin(GL_LINES)
    glVertex3f(0, 0, 0)
    glVertex3f(0, 0, 1)
    glEnd()


#####################################################
#### DrawCube():  draws a cube that spans from (-1,-1,-1) to (1,1,1)
#####################################################

def DrawCube():

    glScalef(0.5, 0.5, 0.5)  # dimensions below are for a 2x2x2 cube, so scale it down by a half first
    glBegin(GL_QUADS)  # Start Drawing The Cube

    glVertex3f(1.0, 1.0, -1.0)  # Top Right Of The Quad (Top)
    glVertex3f(-1.0, 1.0, -1.0)  # Top Left Of The Quad (Top)
    glVertex3f(-1.0, 1.0, 1.0)  # Bottom Left Of The Quad (Top)
    glVertex3f(1.0, 1.0, 1.0)  # Bottom Right Of The Quad (Top)

    glVertex3f(1.0, -1.0, 1.0)  # Top Right Of The Quad (Bottom)
    glVertex3f(-1.0, -1.0, 1.0)  # Top Left Of The Quad (Bottom)
    glVertex3f(-1.0, -1.0, -1.0)  # Bottom Left Of The Quad (Bottom)
    glVertex3f(1.0, -1.0, -1.0)  # Bottom Right Of The Quad (Bottom)

    glVertex3f(1.0, 1.0, 1.0)  # Top Right Of The Quad (Front)
    glVertex3f(-1.0, 1.0, 1.0)  # Top Left Of The Quad (Front)
    glVertex3f(-1.0, -1.0, 1.0)  # Bottom Left Of The Quad (Front)
    glVertex3f(1.0, -1.0, 1.0)  # Bottom Right Of The Quad (Front)

    glVertex3f(1.0, -1.0, -1.0)  # Bottom Left Of The Quad (Back)
    glVertex3f(-1.0, -1.0, -1.0)  # Bottom Right Of The Quad (Back)
    glVertex3f(-1.0, 1.0, -1.0)  # Top Right Of The Quad (Back)
    glVertex3f(1.0, 1.0, -1.0)  # Top Left Of The Quad (Back)

    glVertex3f(-1.0, 1.0, 1.0)  # Top Right Of The Quad (Left)
    glVertex3f(-1.0, 1.0, -1.0)  # Top Left Of The Quad (Left)
    glVertex3f(-1.0, -1.0, -1.0)  # Bottom Left Of The Quad (Left)
    glVertex3f(-1.0, -1.0, 1.0)  # Bottom Right Of The Quad (Left)

    glVertex3f(1.0, 1.0, -1.0)  # Top Right Of The Quad (Right)
    glVertex3f(1.0, 1.0, 1.0)  # Top Left Of The Quad (Right)
    glVertex3f(1.0, -1.0, 1.0)  # Bottom Left Of The Quad (Right)
    glVertex3f(1.0, -1.0, -1.0)  # Bottom Right Of The Quad (Right)
    glEnd()  # Done Drawing The Quad

            # ## Draw the wireframe edges

    glColor3f(0.0, 0.0, 0.0)
    glLineWidth(1.0)

    glBegin(GL_LINE_LOOP)
    glVertex3f(1.0, 1.0, -1.0)  # Top Right Of The Quad (Top)
    glVertex3f(-1.0, 1.0, -1.0)  # Top Left Of The Quad (Top)
    glVertex3f(-1.0, 1.0, 1.0)  # Bottom Left Of The Quad (Top)
    glVertex3f(1.0, 1.0, 1.0)  # Bottom Right Of The Quad (Top)
    glEnd()  # Done Drawing The Quad

    glBegin(GL_LINE_LOOP)
    glVertex3f(1.0, -1.0, 1.0)  # Top Right Of The Quad (Bottom)
    glVertex3f(-1.0, -1.0, 1.0)  # Top Left Of The Quad (Bottom)
    glVertex3f(-1.0, -1.0, -1.0)  # Bottom Left Of The Quad (Bottom)
    glVertex3f(1.0, -1.0, -1.0)  # Bottom Right Of The Quad (Bottom)
    glEnd()  # Done Drawing The Quad

    glBegin(GL_LINE_LOOP)
    glVertex3f(1.0, 1.0, 1.0)  # Top Right Of The Quad (Front)
    glVertex3f(-1.0, 1.0, 1.0)  # Top Left Of The Quad (Front)
    glVertex3f(-1.0, -1.0, 1.0)  # Bottom Left Of The Quad (Front)
    glVertex3f(1.0, -1.0, 1.0)  # Bottom Right Of The Quad (Front)
    glEnd()  # Done Drawing The Quad

    glBegin(GL_LINE_LOOP)
    glVertex3f(1.0, -1.0, -1.0)  # Bottom Left Of The Quad (Back)
    glVertex3f(-1.0, -1.0, -1.0)  # Bottom Right Of The Quad (Back)
    glVertex3f(-1.0, 1.0, -1.0)  # Top Right Of The Quad (Back)
    glVertex3f(1.0, 1.0, -1.0)  # Top Left Of The Quad (Back)
    glEnd()  # Done Drawing The Quad

    glBegin(GL_LINE_LOOP)
    glVertex3f(-1.0, 1.0, 1.0)  # Top Right Of The Quad (Left)
    glVertex3f(-1.0, 1.0, -1.0)  # Top Left Of The Quad (Left)
    glVertex3f(-1.0, -1.0, -1.0)  # Bottom Left Of The Quad (Left)
    glVertex3f(-1.0, -1.0, 1.0)  # Bottom Right Of The Quad (Left)
    glEnd()  # Done Drawing The Quad

    glBegin(GL_LINE_LOOP)
    glVertex3f(1.0, 1.0, -1.0)  # Top Right Of The Quad (Right)
    glVertex3f(1.0, 1.0, 1.0)  # Top Left Of The Quad (Right)
    glVertex3f(1.0, -1.0, 1.0)  # Bottom Left Of The Quad (Right)
    glVertex3f(1.0, -1.0, -1.0)  # Bottom Right Of The Quad (Right)
    glEnd()  # Done Drawing The Quad


####################################################
# printf()
####################################################

def printf(format, *args):
    sys.stdout.write(format % args)


################################################################################
# start the app

print('Hit ESC key to quit.')
main()

            