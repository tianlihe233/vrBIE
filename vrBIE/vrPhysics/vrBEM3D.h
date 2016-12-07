#ifndef _vrBEM3D_H_
#define _vrBEM3D_H_
#include "vrGeometry/VR_Geometry_MeshDataStruct.h"
#include "bemDefines.h"
#include "bemTriangleElem.h"
namespace VR
{
	class vrBEM3D
	{
	public:
		vrBEM3D(const MyFloat _E /*= 1e5*/, const MyFloat _mu /*= 0.3*/,
			const MyFloat _shearMod /*= E / (2 * (1 + mu))*/, const MyFloat _const4 /*= (1 - 2 * mu)*/,
			const MyFloat _const3 /*= 1 / (4 * pi*(1 - mu))*/, const MyFloat _const2 /*= (3 - 4 * mu)*/,
			const MyFloat _const1 /*= 1 / (8 * pi*shearMod * (1 - mu))*/, const MyFloat _kappa /*= 3 - 4 * mu*/);
		~vrBEM3D();
	public:
		int initPhysicalSystem(vrLpsz lpszObjName);
		void renderScene()const;
		MyInt getDofs()const{ return m_nGlobalDof; }
	private:
		void distributeDof3d();
		void createForceBoundaryCondition3d();
		bool isDCCondition(const MyVec3& pos);
		bool isForceCondition(const MyVec3& pos);
		void createGMatrixAndHMatrixBEM3d();
		void makeRigidH();
		MyVector GaussElimination(const MyMatrix& K, MyVector& b);

#if USE_Mantic_CMat
		vrFloat sgn(vrFloat val);
		vrMat3 tensor_product(const MyVec3 vec_a, const MyVec3& vec_b);
		void sortVertexConnectedVertexSurface();
		MyInt index2to1(const MyInt idx0, const MyInt idx1, const MyInt sum = Geometry::vertexs_per_tri)
		{
			MyInt nRet = sum - idx0 - idx1;
			Q_ASSERT(nRet >= 0 && nRet < Geometry::vertexs_per_tri);
			Q_ASSERT(idx0 != idx1);
			return sum - idx0 - idx1;
		}
#endif
	private:
		Geometry::MeshDataStructPtr m_ObjMesh_ptr;
		MyInt m_nGlobalDof;
		MyVec3 m_trace3d;

		std::vector< TriangleElemPtr > m_vec_triElem;
#if USE_Mantic_CMat
		std::vector< VertexPtr > m_vec_vertex_boundary;
#endif
		
		std::vector< VertexPtr > m_vec_vertex2d_trace;

		MyMatrix m_Hsubmatrix, m_Gsubmatrix, m_A_matrix;
		MyVector m_rhs, m_displacement;
	private:
		const MyFloat E;
		const MyFloat mu;
		const MyFloat shearMod;
		const MyFloat const4;
		const MyFloat const3;
		const MyFloat const2;
		const MyFloat const1;
		const MyFloat kappa;
	};
}
#endif//_vrBEM3D_H_