#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
import sys
import math
from random import random

#  from pyquaternion import Quaternion    ## would be useful for 3D simulation

import numpy as np
np.set_printoptions(precision=3)

window = 0  # number of the glut window
theta = 0.0
simTime = 0
dT = 0.01
simRun = True
RAD_TO_DEG = 180.0 / math.pi
G = -9.8
initial_energy = 0

dampling = True
maintaining_energy = True
has_ground_force = True

dampling_energy = 0
pivot_posn = np.array([0, 0, 0])
potential_energy_ground_height = -1
number_of_links = 4


#####################################################
#### Link class, i.e., for a rigid body
#####################################################

class Link:

    index = 0
    color = [0, 0, 0]  # # draw color
    size = [1, 1, 1]  # # dimensions
    mass = 1.0  # # mass in kg
    izz = 1.0  # # moment of inertia about z-axis
    theta = 0  # # 2D orientation  (will need to change for 3D)
    omega = 0  # # 2D angular velocity
    posn = np.array([0.0, 0.0, 0.0])  # # 3D position (keep z=0 for 2D)
    vel = np.array([0.0, 0.0, 0.0])

    acc = np.array([0, 0, 0])
    omega_dot = 0

    def __init__(self, index=0):
        self.index = index
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
    def trail_posn(self):
        return self.posn + self.radius() * np.array([np.cos(self.theta + 1.5 * math.pi), np.sin(self.theta + 1.5 * math.pi), 0])
    def trail_vel(self):
        return np.array([self.vel[0] - self.radius() * self.omega * np.sin(self.theta + math.pi * 1.5), self.vel[1] + links[0].radius() * self.omega * np.cos(self.theta + math.pi * 1.5), 0])
    def trail_acc(self):
        return np.array([self.acc[0] - self.radius() * self.omega_dot * np.sin(self.theta + math.pi * 1.5), self.acc[1] + self.radius() * self.omega_dot * np.cos(self.theta + math.pi * 1.5), 0])
    def head_posn(self):
        return self.posn + self.radius() * np.array([np.cos(self.theta + 0.5 * math.pi), np.sin(self.theta + 0.5 * math.pi), 0])
    def correct_head(self, posn_expected, vel_expected, acc_expected):
        global dT
        # correction for constrained point (fix point)
        posn_real = np.array([self.posn[0] + self.radius() * np.cos(self.theta + math.pi / 2), self.posn[1] + self.radius() * np.sin(self.theta + math.pi / 2), 0])
        vel_real = np.array([self.vel[0] - self.radius() * self.omega * np.sin(self.theta + math.pi / 2), self.vel[1] + links[0].radius() * self.omega * np.cos(self.theta + math.pi / 2), 0])
        acc_real = np.array([self.acc[0] - self.radius() * self.omega_dot * np.sin(self.theta + math.pi / 2), self.acc[1] + self.radius() * self.omega_dot * np.cos(self.theta + math.pi / 2), 0])
        acc_correct = 50 * (posn_expected - posn_real) + 8 * (vel_expected - vel_real)# * abs(vel_expected - vel_real)
        self.vel += acc_correct * dT
    def r_head_x(self):
        return np.cos(self.theta + math.pi / 2) * self.radius()
    def r_head_y(self):
        return np.sin(self.theta + math.pi / 2) * self.radius()
    def r_trail_x(self):
        return np.cos(self.theta + math.pi * 1.5) * self.radius()
    def r_trail_y(self):
        return np.sin(self.theta + math.pi * 1.5) * self.radius()


def print_status():
    global dampling, maintaining_energy, has_ground_force
    translation = {True: 'On', False: 'Off'}
    print(
'''======================================================
Multiple Link Pendulum Simulator USAGE
======================================================
[R] Reset simulation    | [Q Esc] Exit          | [Space] Pause
[A] Add a link          | [S] Remove a link     | [D] Switch Dumpling (current: %s)
[E] Switch energy perserving (current %s)       | [G] Switch ground plane (current: %s)
''' % (translation[dampling], translation[maintaining_energy], translation[has_ground_force]))

#####################################################
#### main():   launches app
#####################################################

def main():
    global window
    glutInit(sys.argv)
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH)  # display mode
    glutInitWindowSize(640, 480)  # window size
    glutInitWindowPosition(0, 0)  # window coords for mouse start at top-left
    window = glutCreateWindow('CPSC 526 Simulation 2D (Sam)')
    glutDisplayFunc(DrawWorld)  # register the function to draw the world

    glutIdleFunc(SimWorld)  # when doing nothing, redraw the scene
    glutReshapeFunc(ReSizeGLScene)  # register the function to call when window is resized
    glutKeyboardFunc(keyPressed)  # register the function to call when keyboard is pressed
    InitGL(640, 480)  # initialize window

    createObjects()
    resetSim()
    print_status()

    glutMainLoop()  # start event processing loop

def createObjects():
    global links
    global number_of_links
    links = [Link(i) for i in range(number_of_links)]

    global kinetic_link, potential_link
    kinetic_link = Link()
    potential_link = Link()

    global ground_link
    ground_link = Link()

#####################################################
#### keyPressed():  called whenever a key is pressed
#####################################################

def resetSim():
    global links
    global simTime, simRun
    global initial_energy, dampling_energy
    global pivot_posn
    global potential_energy_ground_height

    printf('Simulation reset\n')
    simRun = True
    simTime = 0

    links[0].size = [0.04, 1.0, 0.12]
    links[0].color = [random(), random(), random()]
    links[0].posn = np.array([0.0, 0.5 * len(links) * links[0].size[1], 0.0])
    links[0].vel = np.array([0.0, 0.0, 0.0])
    links[0].theta = math.pi / 4
    links[0].omega = 0
    pivot_posn = links[0].head_posn()
    potential_energy_ground_height = links[0].head_posn()[1] - 0.55 * links[0].size[1]

    for i, list in enumerate(links):
        if i == 0:
            continue
        links[i].size = [0.04, 1.0, 0.12]
        links[i].color = [random(), random(), random()]
        links[i].vel = np.array([0.0, 0.0, 0.0])
        links[i].theta = math.pi / 4
        links[i].omega = 0
        links[i].posn = links[i - 1].trail_posn() + links[i - 1].trail_posn() - links[i - 1].posn
        potential_energy_ground_height = potential_energy_ground_height - 0.5 * links[i].size[1]

    initial_energy = sum([link.kinetic_energy() + link.potential_energy() for link in links])
    dampling_energy = 0

    ground_link.size = [10, 0.1, 10]
    ground_link.color = [1.0, 1.0, 1.0]
    ground_link.posn = np.array([0, potential_energy_ground_height - max(len(links[1:]), 2) * links[0].size[1] / 2, 0])

def calEnergy():
    global kinetic_link, potential_link, links, initial_energy, dampling_energy, maintaining_energy
    kinetic_link.color = [1, 0.0, 0.0]
    potential_link.color = [0, 1, 0.0]
    kinetic_energy = sum([link.kinetic_energy() for link in links])
    potential_energy = sum([link.potential_energy() for link in links])
    scale = 1 / initial_energy
    kinetic_link.size = [0.4, kinetic_energy * scale, 0.12]
    potential_link.size = [0.4, potential_energy * scale, 0.12]
    bar_link_origin = np.array([max(2, len(links)) - 1, 0.0, 0.0])
    potential_link.posn = bar_link_origin + np.array([0, potential_link.size[1] / 2, 0])
    kinetic_link.posn = bar_link_origin + np.array([0, potential_link.size[1] + kinetic_link.size[1] / 2, 0])
    kinetic_energy_expected = initial_energy - potential_energy - dampling_energy
    if maintaining_energy and kinetic_energy > 0.01:
        [link.calibrate_energy((max(kinetic_energy_expected, 0) + kinetic_energy) / kinetic_energy / 2) for link in links]
    # print('%f, %f;' % (kinetic_energy, potential_energy))

#####################################################
#### keyPressed():  called whenever a key is pressed
#####################################################

def keyPressed(key, x, y):
    global simRun
    global number_of_links
    global dampling, has_ground_force, maintaining_energy
    ch = key.decode('utf-8')
    if ch == ' ':  # ### toggle the simulation
        if simRun == True:
            simRun = False
        else:
            simRun = True
    elif ch == chr(27):
        sys.exit()
    elif ch == 'q':
        sys.exit()
    elif ch == 'r':
        createObjects()
        resetSim()
    elif ch == 'a':
        number_of_links += 1
        createObjects()
        resetSim()
        print_status()
    elif ch == 's':
        number_of_links = max(number_of_links - 1, 1)
        createObjects()
        resetSim()
        print_status()
    elif ch == 'd':
        dampling = not dampling
        print_status()
    elif ch == 'g':
        has_ground_force = not has_ground_force
        print_status()
    elif ch == 'e':
        maintaining_energy = not maintaining_energy
        print_status()



#####################################################
#### SimWorld():  simulates a time step
#####################################################

def SimWorld():
    global simTime, dT, simRun
    global pivot_posn
    global links
    global dampling_energy
    global dampling, has_ground_force
    global ground_link

    if simRun == False:  # # is simulation stopped?
        return

    # check collision with ground
    ground_force = 0
    if has_ground_force and links[-1].trail_posn()[1] < ground_link.posn[1]:
        k_p = 500
        k_d = 50
        ground_force = k_p * (ground_link.posn[1] - links[-1].trail_posn()[1]) - k_d * links[-1].trail_vel()[1]

    matrix_size = 5 * len(links)
    if ground_force != 0:
        matrix_size += 2
    a = np.zeros((matrix_size, matrix_size))
    b = np.zeros(matrix_size)
    for i, link in enumerate(links):
        a[i * 3, i * 3] = link.mass
        a[i * 3 + 1, i * 3 + 1] = link.mass
        a[i * 3 + 2, i * 3 + 2] = link.inertia()
        b[i * 3: i * 3  + 3] = np.array(
            [0,
             link.mass * G,
             0])
    a[len(links) * 3:len(links) * 3 + 2, 0:3] = np.array(
        [[-1, 0, links[0].r_head_y()],
         [0, -1, -links[0].r_head_x()]])
    b[len(links) * 3:len(links) * 3 + 2] = np.array(
        [-links[0].omega * links[0].omega * links[0].r_head_x(),
         -links[0].omega ** 2 * links[0].r_head_y()])
    for i, link in enumerate(links):
        if i == 0:
            continue
        a[len(links) * 3 + i * 2:len(links) * 3 + i * 2 + 2, i * 3 - 3: i * 3 + 3] = np.array(
            [[-1, 0, links[i - 1].r_trail_y(), 1, 0, -link.r_head_y()],
             [0, -1, -links[i - 1].r_trail_x(), 0, 1, link.r_head_x()]])
        b[len(links) * 3 + i * 2:len(links) * 3 + i * 2 + 2] = np.array(
            [-links[i - 1].omega * links[i - 1].omega * links[i - 1].r_trail_x() + link.omega * link.omega * link.r_head_x(),
             -links[i - 1].omega ** 2 * links[i - 1].r_trail_y() + link.omega ** 2 * link.r_head_y()])
        if i == len(links) - 1 and ground_force != 0:
            a[len(links) * 3 + i * 2 + 2:len(links) * 3 + i * 2 + 2 + 2, i * 3: i * 3 + 3] = np.array(
                [[-1, 0, links[i].r_trail_y()],
                 [0, -1, -links[i].r_trail_x()]])
            b[len(links) * 3 + i * 2 + 2:len(links) * 3 + i * 2 + 2 + 2] = np.array(
                [-links[i].omega * links[i].omega * links[i].r_trail_x(),
                 -links[i].omega ** 2 * links[i].r_trail_y() - ground_force / link.mass])
    for row in range(np.shape(a)[0]):
        for col in range(row + 1, np.shape(a)[1]):
            a[row, col] = a[col, row]
    
    x = np.linalg.solve(a, b)
    for i, link in enumerate(links):
        link.acc = np.array([x[i * 3], x[i * 3 + 1], 0])
        link.omega_dot = x[i * 3 + 2]

    
    for i, link in enumerate(links):
        links[i].color = [random(), random(), random()]
        links[i].posn += links[i].vel * dT
        links[i].vel += links[i].acc * dT
        links[i].theta += links[i].omega * dT
        omega_without_dampling = links[i].omega + links[i].omega_dot * dT
        if dampling:
            # dampling
            k_d = 0.08
            links[i].omega_dot -= (k_d * links[i].omega / links[i].inertia())
            omega_with_dampling = links[i].omega + links[i].omega_dot * dT
            dampling_energy += 0.5 * links[i].inertia() * (np.linalg.norm(omega_without_dampling) ** 2 - np.linalg.norm(omega_with_dampling) ** 2)
            links[i].omega = omega_with_dampling
        else:
            links[i].omega = omega_without_dampling
        if links[i].omega > 10:
            links[i].omega = 10
        if links[i].omega_dot > 10:
            links[i].omega_dot = 10
        if links[i].omega < -10:
            links[i].omega = -10
        if links[i].omega_dot < -10:
            links[i].omega_dot = -10
        if i == 0:
            links[0].correct_head(pivot_posn, np.array([0, 0, 0]), np.array([0, 0, 0]))
        else:
            links[i].correct_head(links[i - 1].trail_posn(), links[i - 1].trail_vel(), links[i - 1].trail_acc())
    

    calEnergy()
    simTime += dT

    # ### draw the updated state
    DrawWorld()
    # printf('simTime=%.2f\n', simTime)


#####################################################
#### DrawWorld():  draw the world
#####################################################

def DrawWorld():
    global links, kinetic_link, potential_link, has_ground_force

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)  # Clear The Screen And The Depth Buffer
    glLoadIdentity()
    gluLookAt(
        1, 3, round(len(links) * 2), # eyeX, eyeY, eyeZ
        0, 0, 0, # centerX, centerY, centerZ
        0, 1, 0,) # upX, upY, upZ

    DrawOrigin()
    [link.draw() for link in links]
    kinetic_link.draw()
    potential_link.draw()
    if has_ground_force:
        ground_link.draw()

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
    glVertex3f(4, 0, 0)
    glEnd()

    glColor3f(0.5, 1, 0.5)  # # light green y-axis
    glBegin(GL_LINES)
    glVertex3f(0, 0, 0)
    glVertex3f(0, 4, 0)
    glEnd()

    glColor3f(0.5, 0.5, 1)  # # light blue z-axis
    glBegin(GL_LINES)
    glVertex3f(0, 0, 0)
    glVertex3f(0, 0, 4)
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

            