/**************************************************
 *
 *                 Shaders.h
 *
 *  Utility functions that make constructing shaders
 *  a bit easier.
 *
 ***************************************************/

#ifndef SHADERS_H
#define SHADERS_H

int buildShader(int type, char *filename);
int buildProgram(int first, ...);
void dumpProgram(int program, char *description);

#endif
