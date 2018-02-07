//===========================================================================
/*
    CPSC 599.86 / 601.86 - Computer Haptics
    Winter 2018, University of Calgary

    This class encapsulates the visual and haptic rendering for an implicit
    surface.  It inherits the regular CHAI3D cMesh class, and taps into
    an external implementation of the Marching Cubes algorithm to create
    a triangle mesh from the implicit surface function.

    Your job is to implement the method that tracks the position of the
    proxy as the tool moves and interacts with the object, as described by
    the implicit surface rendering algorithm in Salisbury & Tarr 1997.

    \author    Your Name
*/
//===========================================================================

#include "ImplicitMesh.h"
#include "MarchingSource.h"
#include <iostream>

using namespace chai3d;


ImplicitMesh::ImplicitMesh()
    : m_surfaceFunction(0), m_projectedSphere(0.05)
{
    // because we are haptically rendering this object as an implicit surface
    // rather than a set of polygons, we will not need a collision detector
    setCollisionDetector(0);
    
    // create a sphere that tracks the position of the proxy
    m_projectedSphere.m_material->setWhite();
    addChild(&m_projectedSphere);
}

ImplicitMesh::~ImplicitMesh()
{
    // remove the proxy tracking sphere so that the base class doesn't delete it
    removeChild(&m_projectedSphere);
}

void ImplicitMesh::createFromFunction(
							double (*f)(double, double, double),
							chai3d::cVector3d (*g)(double, double, double),
							cVector3d a_lowerBound, cVector3d a_upperBound,
                            double a_granularity)
{
    // variables to hold raw triangles returned from marching cubes algorithm
    GLint tcount;
    GLfloat vertices[5*3*3];

    // sample the implicit surface by stepping through each dimension
    for (GLfloat x = a_lowerBound.x(); x <= a_upperBound.x(); x += a_granularity)
        for (GLfloat y = a_lowerBound.y(); y <= a_upperBound.y(); y += a_granularity)
            for (GLfloat z = a_lowerBound.z(); z <= a_upperBound.z(); z += a_granularity)
            {
                // call marching cubes to get the triangular facets for this cell
                vMarchCubeCustom(x, y, z, a_granularity, f, tcount, vertices);

                // add resulting triangles (if any) to our mesh
                for (int i = 0; i < tcount; ++i) {
                    int ix = i*9;
                    this->newTriangle(
                        cVector3d(vertices[ix+0], vertices[ix+1], vertices[ix+2]),
                        cVector3d(vertices[ix+3], vertices[ix+4], vertices[ix+5]),
                        cVector3d(vertices[ix+6], vertices[ix+7], vertices[ix+8])
                    );
                }
            }

    // compute face normals for our mesh so that lighting works properly
    this->computeAllNormals();
    
    // remember the function used to create this object
    m_surfaceFunction = f;
	m_gradientFunction = g;
}

//! Contains code for graphically rendering this object in OpenGL.
void ImplicitMesh::render(cRenderOptions& a_options)
{
    // update the position and visibility of the proxy sphere
    m_projectedSphere.setShowEnabled(m_interactionInside);
    m_projectedSphere.setLocalPos(m_interactionPoint);
    
    // get the base class to render the mesh
    cMesh::render(a_options);
}


//===========================================================================
/*!
    This method should contain the core of the implicit surface rendering
    algorithm implementation.  The member variables m_interactionPoint
    and m_interactionInside should both be set by this method.

    \param  a_toolPos  Position of the tool.
    \param  a_toolVel  Velocity of the tool.
    \param  a_IDN  Identification number of the force algorithm.
*/
//===========================================================================
void ImplicitMesh::computeLocalInteraction(const cVector3d& a_toolPos,
                                           const cVector3d& a_toolVel,
                                           const unsigned int a_IDN)
{
    /////////////////////////////////////////////////////////////////////////
    // [CPSC.86] IMPLICIT SURFACE RENDERING ALGORITHM
    /////////////////////////////////////////////////////////////////////////
    
    // For Part II, the friction coefficients can be read from the object's
    // material as shown here.
    double mu_s = m_material->getStaticFriction();
    double mu_k = m_material->getDynamicFriction();

	chai3d::cVector3d planeNormal;
	chai3d::cVector3d seedPoint;
	chai3d::cVector3d fromProxyToHapticPoint;
	chai3d::cVector3d temp;
	double cosTheta;
	double frictionDist = 0.0;
	double epsilon = 0.00001;

	debugToolPos = a_toolPos;
	functionValue = m_surfaceFunction(a_toolPos.x(), a_toolPos.y(), a_toolPos.z());

	//// Get the gradient at the previously approximated proxy position.. use as plane normal.
	planeNormal = m_gradientFunction(m_interactionPoint.x(), m_interactionPoint.y(), m_interactionPoint.z());
	planeNormal.normalize();
	fromProxyToHapticPoint = a_toolPos - m_interactionPoint;


	if (functionValue < 0.0 || touched)
	{
		temp = fromProxyToHapticPoint; 
		temp.normalize();

		cosTheta = temp.dot(planeNormal);

		debugTempVec = temp;
		debugTempB = cosTheta;
		debugTempA = tan(acos(cosTheta));

		if (abs(tan(acos(cosTheta))) > mu_s)
			kinetic = true;

		if (kinetic)
		{
			if (abs(tan(acos(cosTheta))) > mu_k)
			{
				frictionDist = sin(atan(mu_k)) + epsilon;
			}
			else
				kinetic = false;
		}
		else
		{
			if (abs(tan(acos(cosTheta))) > mu_s)
			{
				frictionDist += sin(atan(mu_k)) + epsilon;
				kinetic = true;
			}
		}


		if (kinetic)
		{
			if (touched)
			{
				chai3d::cVector3d projToolVectorOntoPlaneNormal;

				// Project onto the plane normal and reverse it's direction since here, theta > 90.
				projToolVectorOntoPlaneNormal = fromProxyToHapticPoint.dot(planeNormal) * planeNormal;

				temp = fromProxyToHapticPoint - projToolVectorOntoPlaneNormal;
				temp.normalize();

				temp = temp * frictionDist;

				// New seed point will be on the tangent plane defined by the previously approximated
				// proxy position and the gradient vector at that point.
				seedPoint = m_interactionPoint + fromProxyToHapticPoint - projToolVectorOntoPlaneNormal;// -temp;

				debugSeedPoint = seedPoint;

				m_interactionPoint = findNearestSurfacePoint(seedPoint, epsilon);        

				if (fromProxyToHapticPoint.dot(planeNormal) > epsilon)
					touched = false;
			}	
			else
			{
				m_interactionPoint = findNearestSurfacePoint(a_toolPos, epsilon);
				touched = true;
			}
		}

		m_interactionInside = true;
	}
	else
	{
		if ((a_toolPos.x() !=  m_interactionPoint.x()) || (a_toolPos.y() != m_interactionPoint.y()) || (a_toolPos.z() != m_interactionPoint.z()))
			kinetic = true;
		else
			kinetic = false;

		// m_interactionProjectedPoint is the "proxy" point on the surface
		// that the rendering algorithm tracks.  It should be equal to the
		// tool position when the tool is not in contact with the object.
		m_interactionPoint = a_toolPos;

		// m_interactionInside should be set to true when the tool is in contact
		// with the object.
		m_interactionInside = false;
	}

	debugTouched = touched;
	debugKinetic = kinetic;
}



cVector3d ImplicitMesh::findNearestSurfacePoint(cVector3d seedPoint, double epsilon)
{
	chai3d::cVector3d p(seedPoint);
	chai3d::cVector3d deltaP;

	do
	{
		debugGradientVector = m_gradientFunction(p.x(), p.y(), p.z());

		deltaP = -1 * m_surfaceFunction(p.x(), p.y(), p.z()) * debugGradientVector / debugGradientVector.dot(debugGradientVector); 
		p += deltaP;

		deltaMovement = deltaP.length();




	} while (deltaP.length() > epsilon);

	return p;
}