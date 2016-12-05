#ifndef _vrGlobalConf_H_
#define _vrGlobalConf_H_

#include "vrBase/vrBase.h"
namespace VR
{
	namespace GlobalConf
	{
		extern bool g_Obj_hasCoord;
		extern vrString const g_const_Obj_hasCoord;// = vrString("Obj.hasCoord");

		extern bool g_Obj_hasVerticeNormal;
		extern vrString const g_const_Obj_hasVerticeNormal;// = vrString("Obj.hasVerticeNormal");

		extern bool g_Obj_normalizeMesh;
		extern vrString const g_const_Obj_normalizeMesh;// = vrString("Obj.normalizeMesh");

		extern vrString g_str_Obj_meshName;
		extern vrString const g_const_Obj_meshName;// = vrString("Obj.meshName");

		extern vrString g_str_Obj_textureName;
		extern vrString const g_const_Obj_textureName;// = vrString("Obj.textureName");

		extern vrString g_str_Obj_meshId;
		extern vrString const g_const_Obj_meshId;// = vrString("Obj.meshId");

		extern vrInt g_n_Obj_octreeFineLevel;
		extern vrString const g_const_Obj_octreeFineLevel;// = vrString("Obj.octreeFineLevel");

		extern bool g_Simulation_doSimulation;
		extern vrString const g_const_Simulation_doSimulation;// = vrString("Simulation.doSimulation");

		extern vrInt g_n_Simulation_MinCount;
		extern vrString const g_const_Simulation_MinCount;// = vrString("Simulation.MinCount");

		extern vrInt g_n_Simulation_MaxCount;
		extern vrString const g_const_Simulation_MaxCount;// = vrString("Simulation.MaxCount");

		extern vrFloat g_db_Simulation_YoungModulus;
		extern vrString const g_const_Simulation_YoungModulus;// = vrString("Simulation.YoungModulus");

		extern vrFloat g_db_Simulation_externalForceFactor;
		extern vrString const g_const_Simulation_externalForceFactor;// = vrString("Simulation.externalForceFactor");

		extern vrFloat g_db_Simulation_scriptForceFactor;
		extern vrString const g_const_Simulation_scriptForceFactor;// = vrString("Simulation.scriptForceFactor");

		extern vrInt g_n_Scene_windowWidth;
		extern vrString const g_const_Scene_windowWidth;// = vrString("Scene.windowWidth");

		extern vrInt g_n_Scene_windowHeight;
		extern vrString const g_const_Scene_windowHeight;// = vrString("Scene.windowHeight");

		extern vrVec4 g_vec4_Scene_bkgColor;
		extern vrString const g_const_Scene_bkgColor;// = vrString("Scene.bkgColor");

		extern vrFloat g_db_Scene_planeXsize;
		extern vrString const g_const_Scene_planeXsize;// = vrString("Scene.planeXsize");

		extern vrFloat g_db_Scene_planeZsize;
		extern vrString const g_const_Scene_planeZsize;// = vrString("Scene.planeZsize");

		extern vrInt g_n_Scene_planeXdivs;
		extern vrString const g_const_Scene_planeXdivs;// = vrString("Scene.planeXdivs");

		extern vrInt g_n_Scene_planeZdivs;
		extern vrString const g_const_Scene_planeZdivs;// = vrString("Scene.planeZdivs");	

		extern vrVec3 g_vec3_Scene_planeColor;
		extern vrString const g_const_Scene_planeColor;// = vrString("Scene.planeColor");

		extern vrVec3 g_vec3_Scene_modelColor;
		extern vrString const g_const_Scene_modelColor;// = vrString("Scene.modelColor");

		extern vrVec3 g_vec3_Scene_modelColor4Ka;
		extern vrString const g_const_Scene_modelColor4Ka;// = vrString("Scene.modelColor4Ka");

		extern vrVec3 g_vec3_Scene_CamerPos;
		extern vrString const g_const_Scene_CamerPos;// = vrString("Scene.CamerPos");

		extern vrVec3 g_vec3_Scene_LightPos;
		extern vrString const g_const_Scene_LightPos;// = vrString("Scene.LightPos");

		extern vrVec3 g_vec3_Scene_EyeCenter;
		extern vrString const g_const_Scene_EyeCenter;// = vrString("Scene.EyeCenter");

		extern vrVec3 g_vec3_Scene_ZUp;
		extern vrString const g_const_Scene_ZUp;// = vrString("Scene.ZUp");

		extern bool g_Scene_ShowFPS;
		extern vrString const g_const_Scene_ShowFPS;// = vrString("Scene.ShowFPS");

		extern vrFloat g_db_Scene_FOV;
		extern vrString const g_const_Scene_FOV;// = vrString("Scene.FOV");

		extern vrFloat g_db_Scene_Near;
		extern vrString const g_const_Scene_Near;// = vrString("Scene.Near");

		extern vrFloat g_db_Scene_Far;
		extern vrString const g_const_Scene_Far;// = vrString("Scene.Far");
			 
		extern vrInt g_n_Scene_ShadowMapWidth;
		extern vrString const g_const_Scene_ShadowMapWidth;// = vrString("Scene.ShadowMapWidth");

		extern vrInt g_n_Scene_ShadowMapHeight;
		extern vrString const g_const_Scene_ShadowMapHeight;// = vrString("Scene.ShadowMapHeight");

		extern vrFloat g_db_Simulation_animation_max_fps;
		extern vrString const g_const_Simulation_animation_max_fps;// = vrString("Simulation.animation_max_fps");

		extern vrFloat g_db_Simulation_camera_zoom;
		extern vrString const g_const_Simulation_camera_zoom;// = vrString("Simulation.camera_zoom");

		extern void printConf();
	}//GlobalConf
}//VR
#endif//_vrGlobalConf_H_