//
// A header file for accessing Cory Bloyd's Marching Cubes implementation
// from an external program.  Created for CS77 - Experimental Haptics at
// Stanford University.  See the original MarchingCubes.cpp file for
// information and copyright.
//
// January 2010 - Sonny Chan
//

#ifndef MARCHINGSOURCE_H
#define MARCHINGSOURCE_H

// vMarchCubeCustom is a special entry point into Bloyd's source that
// will compute and return the triangle vertices for a single marching
// cubes cell on a custom function. f < 0 is inside, f > 0 is out.

GLvoid vMarchCubeCustom(GLfloat fX, GLfloat fY, GLfloat fZ, GLfloat fScale,
                        double (*f)(double, double, double),
                        GLint &iTriCount, GLfloat *afVertices);

#endif
