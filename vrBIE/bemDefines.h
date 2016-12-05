#ifndef _bemDefines_h_
#define _bemDefines_h_
#include "vrBase/vrBase.h"
#include "bemMarco.h"
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/SparseCore>
#include <Eigen/Dense>
#include <vector>
#include <assert.h>
#include <boost/smart_ptr.hpp>

namespace VR
{
#if DoublePrecision
	
	typedef double MyFloat;
	typedef int   MyInt;
	typedef Eigen::MatrixXd MyDenseMatrix;
	typedef Eigen::Vector3d MyDenseVector;
	typedef Eigen::Vector2d MyVec2ParamSpace;
	typedef Eigen::Vector2i MyVec2IParamSpace;
	typedef Eigen::Vector3d MyVec3;
	typedef Eigen::Vector3i MyVec3I;

	typedef MyDenseVector MyPoint;
	typedef Eigen::MatrixXi MyIntMatrix;
	typedef Eigen::VectorXi MyIntVector;
	typedef Eigen::Vector3i MyVectorI;
	
	typedef Eigen::MatrixXd MyMatrix;
	typedef Eigen::VectorXd MyVector;
	typedef Eigen::Matrix3d MyMatrix_3X3;
	typedef Eigen::Matrix2d MyMatrix_2X2;
	typedef Eigen::SparseMatrix<MyFloat, 1> MySpMat;
	typedef Eigen::Matrix<MyFloat, 3, 2> MyCoords_3X2;
	typedef MyMatrix_3X3 MyCoords_3X3;
	typedef MyCoords_3X2 MyMatrix_3X2;

	typedef Eigen::Matrix<MyInt, 9, 1> MyVector9I;
	typedef Eigen::Matrix<MyFloat, 9, 1> MyVector9;
#else
	typedef float MyFloat;
	typedef int   MyInt;
	typedef Eigen::MatrixXf MyDenseMatrix;
	typedef Eigen::Vector3f MyDenseVector;
	typedef Eigen::Vector2f MyVec2;
	typedef Eigen::Vector2i MyVec2I;

	typedef MyDenseVector MyPoint;
	typedef Eigen::MatrixXf MyMatrix;
	typedef Eigen::MatrixXi MyIntMatrix;
	typedef Eigen::VectorXf MyVector;
	typedef Eigen::VectorXi MyIntVector;
	typedef Eigen::Vector3i MyVectorI;
	typedef Eigen::Matrix3f MyMatrix_3X3;
	typedef Eigen::Matrix2f MyMatrix_2X2;
	typedef Eigen::SparseMatrix<MyFloat, 1> MySpMat;

	typedef Eigen::Matrix<MyInt, 9, 1> MyVector9I;
	typedef Eigen::Matrix<MyFloat, 9, 1> MyVector9;
#endif

	//namespace Colors
	//{
	//	typedef enum{ black = 0, blue = 1, green = 2, indigoBlue = 3, red = 4, pink = 5, yellow = 6, white = 7 } MyColors;
	//	static MyFloat colorTemplage[8][3] = { { 0.0f, 0.0f, 0.0f },//black
	//	{ 0.0f, 0.0f, 1.0f },//blue
	//	{ 0.0f, 1.0f, 0.0f },//green
	//	{ 0.0f, 1.0f, 1.0f },//dian blue
	//	{ 1.0f, 0.0f, 0.0f },//red
	//	{ 1.0f, 0.0f, 1.0f },//pink
	//	{ 1.0f, 1.0f, 0.0f },//yellow
	//	{ 1.0f, 1.0f, 1.0f } };//white

	//	static YC::MyDenseVector purple(80.0 / 255.0, 64.0 / 255.0, 255.0 / 255.0);
	//	static YC::MyDenseVector gold(255.0 / 255.0, 228.0 / 255.0, 58.0 / 255.0);
	//}

	namespace Geometry
	{
		const int shape_Function_Count_In_BEM = 3;
		const int gauss_Sample_Point = 3;
		const int n_tensor_pols = 8;
		const int dofs_per_cell = 9;
		const int vertexs_per_tri = 3;
		const int dimensions_per_vertex = 3;
		const int first_dof_idx = 0;
		const int max_dofs_per_face = 9;
	}
}
#endif//_bemDefines_h_