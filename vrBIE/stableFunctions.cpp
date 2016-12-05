#include <GL/glew.h>
#include <GL/freeglut.h>

#include <stdlib.h>
#include <iostream>
#include <stdio.h>
#include <fstream>

#include "vrBase/vrRotation.h"
#include "vrGlobalConf.h"
#include "vrBase/vrLog.h"
using namespace std;
extern void display(void);
extern VR::Interactive::vrBallController g_trackball_1;

const GLfloat light_ambient[] = { 0.0f, 0.0f, 0.0f, 1.0f };
const GLfloat light_diffuse[] = { 1.0f, 1.0f, 1.0f, 1.0f };
const GLfloat light_specular[] = { 1.0f, 1.0f, 1.0f, 1.0f };
const GLfloat light_position[] = { 2.0f, 5.0f, 5.0f, 0.0f };

const GLfloat mat_ambient[] = { 0.7f, 0.7f, 0.7f, 1.0f };
const GLfloat mat_diffuse[] = { 0.8f, 0.8f, 0.8f, 1.0f };
const GLfloat mat_specular[] = { 1.0f, 1.0f, 1.0f, 1.0f };
const GLfloat high_shininess[] = { 100.0f };

extern void addSceneScale();
extern void subSceneScale();
 void key(unsigned char key, int x, int y)
{
	switch (key)
	{
	case '+':
		addSceneScale();
		break;

	case '-':
		subSceneScale();
		break;

	case 27:
	case 'q':
		exit(0);
		break;
	}
	glutPostRedisplay();
}

void idle(void)
{
	glutPostRedisplay();
}

void resize(int width, int height)
{
	glViewport(0, 0, width, height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45.0, width / (float)height, 2.0, 10.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glTranslatef(0.0, 0.0, -5.0);

	g_trackball_1.ClientAreaResize(VR::Interactive::ERect(0, 0, width, height));
	glutPostRedisplay();

	/*const float ar = (float)width / (float)height;

	glViewport(0, 0, width, height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glFrustum(-ar, ar, -1.0, 1.0, 2.0, 100.0);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	g_trackball_1.ClientAreaResize(VR::Interactive::ERect(0, 0, width, height));*/

}

/* GLUT callback Handlers */
void motion(int x, int y)
{
	if (((GetKeyState(VK_CONTROL) & 0x80) > 0)  /*|| GLUT_ACTIVE_ALT == glutGetModifiers()*/)
	{
		g_trackball_1.MouseMove(VR::Interactive::EPoint(x, y));
		//g_trackball_2.MouseMove(VR_FEM::EPoint(x, y));
		//printf("motion (%d,%d)\n",x,y);
	}

	glutPostRedisplay();
}

void mouse(int button, int state, int x, int y)
{
	if (state == GLUT_DOWN)
	{
		if (((GetKeyState(VK_CONTROL) & 0x80) > 0))
		{
			//g_trackball.setCuttingTrack(false);
			g_trackball_1.MouseDown(VR::Interactive::EPoint(x, y));
			//g_trackball_2.MouseDown(VR_FEM::EPoint(x, y));
			//printf("mouse down(%d,%d)\n",x,y);
		}
	}
	else if (state == GLUT_UP)
	{
		g_trackball_1.MouseUp(VR::Interactive::EPoint(x, y));
		//g_trackball_2.MouseUp(VR_FEM::EPoint(x, y));
		//printf("mouse up(%d,%d)\n",x,y);
	}
	glutPostRedisplay();
}

void Specialkeyboard(int key, int x, int y)
{
	switch (key) {
	case GLUT_KEY_LEFT:
	{
						  g_trackball_1.Key(39);
						  //g_trackball_2.Key(39);
						  break;
	}
	case GLUT_KEY_RIGHT:
	{
						   g_trackball_1.Key(37);
						   //g_trackball_2.Key(37);
						   break;
	}
	case GLUT_KEY_UP:
	{
						g_trackball_1.Key(38);
						//g_trackball_2.Key(38);
						break;
	}
	case GLUT_KEY_DOWN:
	{
						  g_trackball_1.Key(40);
						  //g_trackball_2.Key(40);
						  break;
	}
	default:
	{
			   printf("Special key %d\n", key);
	}
	}
	glutPostRedisplay();
}

namespace VR
{

	void init_global()
	{
		VR::ConfigureParser::vrPropertyMap propertyMap;
		vrString inifile = FileSystem::get_currentpath() + vrString("/conf/param.conf");

		propertyMap[GlobalConf::g_const_Obj_hasCoord];
		propertyMap[GlobalConf::g_const_Obj_hasVerticeNormal];
		propertyMap[GlobalConf::g_const_Obj_normalizeMesh];
		propertyMap[GlobalConf::g_const_Obj_meshName];
		propertyMap[GlobalConf::g_const_Obj_textureName];
		propertyMap[GlobalConf::g_const_Obj_meshId];
		propertyMap[GlobalConf::g_const_Obj_octreeFineLevel];
		propertyMap[GlobalConf::g_const_Simulation_doSimulation];
		propertyMap[GlobalConf::g_const_Simulation_MinCount];
		propertyMap[GlobalConf::g_const_Simulation_MaxCount];
		propertyMap[GlobalConf::g_const_Simulation_YoungModulus];
		propertyMap[GlobalConf::g_const_Simulation_externalForceFactor];
		propertyMap[GlobalConf::g_const_Simulation_scriptForceFactor];
		propertyMap[GlobalConf::g_const_Simulation_animation_max_fps];
		propertyMap[GlobalConf::g_const_Simulation_camera_zoom];

		propertyMap[GlobalConf::g_const_Scene_windowWidth];
		propertyMap[GlobalConf::g_const_Scene_windowHeight];
		propertyMap[GlobalConf::g_const_Scene_bkgColor];
		propertyMap[GlobalConf::g_const_Scene_planeXsize];
		propertyMap[GlobalConf::g_const_Scene_planeZsize];
		propertyMap[GlobalConf::g_const_Scene_planeXdivs];
		propertyMap[GlobalConf::g_const_Scene_planeZdivs];
		propertyMap[GlobalConf::g_const_Scene_planeColor];
		propertyMap[GlobalConf::g_const_Scene_modelColor];
		propertyMap[GlobalConf::g_const_Scene_modelColor4Ka];
		propertyMap[GlobalConf::g_const_Scene_CamerPos];
		propertyMap[GlobalConf::g_const_Scene_LightPos];
		propertyMap[GlobalConf::g_const_Scene_EyeCenter];
		propertyMap[GlobalConf::g_const_Scene_ZUp];
		propertyMap[GlobalConf::g_const_Scene_ShowFPS];
		propertyMap[GlobalConf::g_const_Scene_FOV];
		propertyMap[GlobalConf::g_const_Scene_Near];
		propertyMap[GlobalConf::g_const_Scene_Far];
		propertyMap[GlobalConf::g_const_Scene_ShadowMapWidth];
		propertyMap[GlobalConf::g_const_Scene_ShadowMapHeight];


		VR::ConfigureParser::parser_configurefile(inifile, propertyMap);


		VR::ConfigureParser::getConfPropertyValue(propertyMap, GlobalConf::g_const_Obj_hasCoord, GlobalConf::g_Obj_hasCoord);
		VR::ConfigureParser::getConfPropertyValue(propertyMap, GlobalConf::g_const_Obj_hasVerticeNormal, GlobalConf::g_Obj_hasVerticeNormal);
		VR::ConfigureParser::getConfPropertyValue(propertyMap, GlobalConf::g_const_Obj_normalizeMesh, GlobalConf::g_Obj_normalizeMesh);

		VR::ConfigureParser::getConfPropertyValueStr(propertyMap, GlobalConf::g_const_Obj_meshName, GlobalConf::g_str_Obj_meshName);
		VR::ConfigureParser::getConfPropertyValueStr(propertyMap, GlobalConf::g_const_Obj_textureName, GlobalConf::g_str_Obj_textureName);
		VR::ConfigureParser::getConfPropertyValueStr(propertyMap, GlobalConf::g_const_Obj_meshId, GlobalConf::g_str_Obj_meshId);

		VR::ConfigureParser::getConfPropertyValue(propertyMap, GlobalConf::g_const_Obj_octreeFineLevel, GlobalConf::g_n_Obj_octreeFineLevel);
		VR::ConfigureParser::getConfPropertyValue(propertyMap, GlobalConf::g_const_Simulation_doSimulation, GlobalConf::g_Simulation_doSimulation);
		VR::ConfigureParser::getConfPropertyValue(propertyMap, GlobalConf::g_const_Simulation_MinCount, GlobalConf::g_n_Simulation_MinCount);
		VR::ConfigureParser::getConfPropertyValue(propertyMap, GlobalConf::g_const_Simulation_MaxCount, GlobalConf::g_n_Simulation_MaxCount);
		VR::ConfigureParser::getConfPropertyValue(propertyMap, GlobalConf::g_const_Simulation_YoungModulus, GlobalConf::g_db_Simulation_YoungModulus);
		VR::ConfigureParser::getConfPropertyValue(propertyMap, GlobalConf::g_const_Simulation_externalForceFactor, GlobalConf::g_db_Simulation_externalForceFactor);
		VR::ConfigureParser::getConfPropertyValue(propertyMap, GlobalConf::g_const_Simulation_scriptForceFactor, GlobalConf::g_db_Simulation_scriptForceFactor);

		VR::ConfigureParser::getConfPropertyValue(propertyMap, GlobalConf::g_const_Simulation_animation_max_fps, GlobalConf::g_db_Simulation_animation_max_fps);
		VR::ConfigureParser::getConfPropertyValue(propertyMap, GlobalConf::g_const_Simulation_camera_zoom, GlobalConf::g_db_Simulation_camera_zoom);

		VR::ConfigureParser::getConfPropertyValue(propertyMap, GlobalConf::g_const_Scene_windowWidth, GlobalConf::g_n_Scene_windowWidth);
		VR::ConfigureParser::getConfPropertyValue(propertyMap, GlobalConf::g_const_Scene_windowHeight, GlobalConf::g_n_Scene_windowHeight);
		VR::ConfigureParser::getConfPropertyValue(propertyMap, GlobalConf::g_const_Scene_bkgColor, GlobalConf::g_vec4_Scene_bkgColor);

		VR::ConfigureParser::getConfPropertyValue(propertyMap, GlobalConf::g_const_Scene_planeXsize, GlobalConf::g_db_Scene_planeXsize);
		VR::ConfigureParser::getConfPropertyValue(propertyMap, GlobalConf::g_const_Scene_planeZsize, GlobalConf::g_db_Scene_planeZsize);
		VR::ConfigureParser::getConfPropertyValue(propertyMap, GlobalConf::g_const_Scene_planeXdivs, GlobalConf::g_n_Scene_planeXdivs);
		VR::ConfigureParser::getConfPropertyValue(propertyMap, GlobalConf::g_const_Scene_planeZdivs, GlobalConf::g_n_Scene_planeZdivs);

		VR::ConfigureParser::getConfPropertyValue(propertyMap, GlobalConf::g_const_Scene_planeColor, GlobalConf::g_vec3_Scene_planeColor);
		VR::ConfigureParser::getConfPropertyValue(propertyMap, GlobalConf::g_const_Scene_modelColor, GlobalConf::g_vec3_Scene_modelColor);
		VR::ConfigureParser::getConfPropertyValue(propertyMap, GlobalConf::g_const_Scene_modelColor4Ka, GlobalConf::g_vec3_Scene_modelColor4Ka);

		VR::ConfigureParser::getConfPropertyValue(propertyMap, GlobalConf::g_const_Scene_CamerPos, GlobalConf::g_vec3_Scene_CamerPos);
		VR::ConfigureParser::getConfPropertyValue(propertyMap, GlobalConf::g_const_Scene_LightPos, GlobalConf::g_vec3_Scene_LightPos);
		VR::ConfigureParser::getConfPropertyValue(propertyMap, GlobalConf::g_const_Scene_EyeCenter, GlobalConf::g_vec3_Scene_EyeCenter);
		VR::ConfigureParser::getConfPropertyValue(propertyMap, GlobalConf::g_const_Scene_ZUp, GlobalConf::g_vec3_Scene_ZUp);
		VR::ConfigureParser::getConfPropertyValue(propertyMap, GlobalConf::g_const_Scene_ShowFPS, GlobalConf::g_Scene_ShowFPS);
		VR::ConfigureParser::getConfPropertyValue(propertyMap, GlobalConf::g_const_Scene_FOV, GlobalConf::g_db_Scene_FOV);
		VR::ConfigureParser::getConfPropertyValue(propertyMap, GlobalConf::g_const_Scene_Near, GlobalConf::g_db_Scene_Near);
		VR::ConfigureParser::getConfPropertyValue(propertyMap, GlobalConf::g_const_Scene_Far, GlobalConf::g_db_Scene_Far);
		VR::ConfigureParser::getConfPropertyValue(propertyMap, GlobalConf::g_const_Scene_ShadowMapWidth, GlobalConf::g_n_Scene_ShadowMapWidth);
		VR::ConfigureParser::getConfPropertyValue(propertyMap, GlobalConf::g_const_Scene_ShadowMapHeight, GlobalConf::g_n_Scene_ShadowMapHeight);

		GlobalConf::printConf();
	}
}

const char* loadShaderAsString(const char* file)
{
	std::ifstream shader_file(file, std::ifstream::in);
	std::string str((std::istreambuf_iterator<char>(shader_file)), std::istreambuf_iterator<char>());
	return str.c_str();
}


void initializeMyShader(const char* lpszVtxShader, const char* lpszFrgShader)
{
	GLuint vertShader = glCreateShader(GL_VERTEX_SHADER);
	if (0 == vertShader)
	{
		fprintf(stderr, "Error creating vertex shader.\n");
		exit(1);
	}

	const GLchar * shaderCode = loadShaderAsString(lpszVtxShader);
	const GLchar* codeArray[] = { shaderCode };
	glShaderSource(vertShader, 1, codeArray, NULL);

	glCompileShader(vertShader);

	GLint result;
	glGetShaderiv(vertShader, GL_COMPILE_STATUS, &result);
	if (GL_FALSE == result)
	{
		fprintf(stderr, "Vertex shader compilation failed!\n");
		GLint logLen;
		glGetShaderiv(vertShader, GL_INFO_LOG_LENGTH, &logLen);
		if (logLen > 0)
		{
			char * log = (char *)malloc(logLen);
			GLsizei written;
			glGetShaderInfoLog(vertShader, logLen, &written, log);
			fprintf(stderr, "Shader log:\n%s", log);
			free(log);
		}
	}
}

void initOpenGL(int argc, char *argv[])
{
	using namespace VR;
	glutInit(&argc, argv);
	glutInitWindowSize(GlobalConf::g_n_Scene_windowWidth, GlobalConf::g_n_Scene_windowHeight);
	glutInitWindowPosition(10, 10);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);

	glutCreateWindow(vrAppTitle);

	GLenum err = glewInit();
	if (GLEW_OK != err)
	{
		printf("Error initializing GLEW: %s\n", glewGetErrorString(err));
	}
	else
	{
		const GLubyte *renderer = glGetString(GL_RENDERER);
		const GLubyte *vendor = glGetString(GL_VENDOR);
		const GLubyte *version = glGetString(GL_VERSION);
		const GLubyte *glslVersion =
			glGetString(GL_SHADING_LANGUAGE_VERSION);
		GLint major, minor;
		glGetIntegerv(GL_MAJOR_VERSION, &major);
		glGetIntegerv(GL_MINOR_VERSION, &minor);
		printf("GL Vendor : %s\n", vendor);
		printf("GL Renderer : %s\n", renderer);
		printf("GL Version (string) : %s\n", version);
		printf("GL Version (integer) : %d.%d\n", major, minor);
		printf("GLSL Version : %s\n", glslVersion);

		/*GLint nExtensions;
		glGetIntegerv(GL_NUM_EXTENSIONS, &nExtensions);
		for( int i = 0; i < nExtensions; i++ )
		printf("%s\n", glGetStringi( GL_EXTENSIONS, i ) );*/

		if (GL_FALSE == glewGetExtension("GL_SHADING_LANGUAGE_VERSION"))
		{
			printf("GL_SHADING_LANGUAGE unsupport!\n");
		}

		//initializeMyShader((FileSystem::get_currentpath() + vrString("/myshader/basic.vert")).c_str());
	}
	g_trackball_1.SetColor(RGB(130, 80, 30));
	g_trackball_1.SetDrawConstraints();

	glutReshapeFunc(resize);
	glutDisplayFunc(display);
	glutKeyboardFunc(key);
	glutIdleFunc(idle);
	glutSpecialFunc(Specialkeyboard);
	glutMouseFunc(mouse);
	glutMotionFunc(motion);

	glClearColor(1, 1, 1, 1);
	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);

	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);

	glEnable(GL_LIGHT0);
	glEnable(GL_NORMALIZE);
	glEnable(GL_COLOR_MATERIAL);
	glEnable(GL_LIGHTING);

	glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
	glLightfv(GL_LIGHT0, GL_POSITION, light_position);

	glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ambient);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_diffuse);
	glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
	glMaterialfv(GL_FRONT, GL_SHININESS, high_shininess);
}