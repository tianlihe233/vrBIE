#include <GL/glew.h>
#include <GL/freeglut.h>
#include <stdlib.h>
#include <iostream>
#include <stdio.h>
#include <fstream>
#include <boost/math/constants/constants.hpp> // pi
#include "bemDefines.h"
#include "vrBase/vrRotation.h"
#include "vrGlobalConf.h"
#include "vrBase/vrLog.h"
//#include "VR_Geometry_MeshDataStruct.h"
//#include "vrBEM3D.h"
using namespace std;
using namespace VR;


extern VR::Interactive::vrBallController g_trackball_1;



const MyFloat pi = boost::math::constants::pi<MyFloat>();
const MyFloat E = 1e5;
const MyFloat mu = 0.3;
const MyFloat shearMod = E / (2 * (1 + mu));
MyFloat const4 = (1 - 2 * mu);
MyFloat const3 = 1 / (4 * pi*(1 - mu));
MyFloat const2 = (3 - 4 * mu);
MyFloat const1 = 1 / (8 * pi*shearMod * (1 - mu));
const MyFloat kappa = 3 - 4 * mu;
//vrBEM3D g_BEM3D(E , mu ,shearMod, const4 ,const3 , const2 ,const1 , kappa );
//global p knotVec controlPts elRange bsFnConn dispConn tracConn
const MyInt p = 2; //degree of basis functions
const MyInt numMeshes = 1;
const MyInt inc = 5;

const MyInt infinitePlate = 0; //if this flag is set, we apply the infinite plate tractions.
// otherwise we apply uniform traction(tractionX)

//these parameters are for the plate with a hole problem
const MyInt tractionAtInfinity = 100;
MyVec3 exactTracInterval;

//MyFloat tractionX;
//MyFloat tractionY;
MyDenseVector tractionXYZ;

MyFloat g_scene_scale = 4.0;

void addSceneScale()
{
	g_scene_scale *= 1.1;
}

void subSceneScale()
{
	g_scene_scale *= 0.9;
}

namespace VR
{
	void showScene()
	{
		g_trackball_1.IssueGLrotation();
		glScalef(g_scene_scale, g_scene_scale, g_scene_scale);
		glutSolidCube(0.3);
		//g_BEM3D.renderScene();
	}

	void initPhysSystem()
	{
		

		//1.load 3d obj mesh
		//g_BEM3D.initPhysicalSystem(vrCStr(GlobalConf::g_str_Obj_meshName));
		
	}
}