#ifndef WGP
#define WGP

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <time.h>

// GLEW _MUST_ be included first
#ifdef USEGLEW
#include <GL/glew.h>
#endif
//  Get all GL prototypes
#define GL_GLEXT_PROTOTYPES
//  Select SDL, SDL2, GLFW or GLUT
#if defined(SDL2)
#include <SDL2/SDL.h>
#include <SDL2/SDL_mixer.h>
#elif defined(SDL)
#include <SDL/SDL.h>
#include <SDL/SDL_mixer.h>
#elif defined(GLFW)
#include <GLFW/glfw3.h>
#elif defined(__APPLE__)
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif
//  Make sure GLU and GL are included
#ifdef __APPLE__
#include <OpenGL/glu.h>
#include <OpenGL/gl.h>
// Tell Xcode IDE to not gripe about OpenGL deprecation
#pragma clang diagnostic ignored "-Wdeprecated-declarations"
#else
#include <GL/glu.h>
#include <GL/gl.h>
#endif
//  Default resolution
//  For Retina displays compile with -DRES=2
#ifndef RES
#define RES 1
#endif

//  cos and sin in degrees
#define Cos(th) cos(3.14159265/180*(th))
#define Sin(th) sin(3.14159265/180*(th))
#define revCos(th) acos(th)*180/3.14159265
#define revSin(th) asin(th)*180/3.14159265

#ifdef __cplusplus
extern "C" {
#endif
/*
 PUT METHODS HERE
 
*/


void Print(const char* format , ...);
void Fatal(const char* format , ...);
unsigned int LoadTexBMP(const char* file);
void Project(double asp,double dim);
void ErrCheck(const char* where);
void icosahedron(float x,float y,float z,float s, unsigned int t,int light);
void Vertex(double th, double ph);
void Normal(double* P, double* Q, double* R, double* cross);
void triangles(double x, double y, double z, double s, double rx, double ry, int colors,int tex,double ts,int texi);
// void draw_boids(int n,double zh);
// void boid(int x, int y, int z);
int CreateShaderProg(char* VertFile,char* FragFile);



#ifdef __cplusplus
}
#endif

#endif