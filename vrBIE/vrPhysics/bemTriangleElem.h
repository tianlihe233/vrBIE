#ifndef _bemTriangleElem_h_
#define _bemTriangleElem_h_
#include <boost/smart_ptr.hpp>
#include "bemVertex.h"


namespace VR
{
	typedef enum{ Continuous/*, DisContinuous*//*use for dc condition */ } TriElemType;

	class TriangleElem;
	typedef boost::shared_ptr< TriangleElem > TriangleElemPtr;
	class TriangleElem
	{
	public:
		class TriangleElemCompare
		{
		public:
			TriangleElemCompare(const MyVec3& vtx0, const MyVec3& vtx1, const MyVec3& vtx2)
			{
				vertex[0] = vtx0;
				vertex[1] = vtx1;
				vertex[2] = vtx2;
			}

			bool operator()(TriangleElemPtr& line)
			{
				const MyVec3& line_vtx_0 = (*line).m_elem_vertex[0]->getPos();
				const MyVec3& line_vtx_1 = (*line).m_elem_vertex[1]->getPos();
				const MyVec3& line_vtx_2 = (*line).m_elem_vertex[2]->getPos();

				return numbers::isEqual(line_vtx_0, vertex[0]) && numbers::isEqual(line_vtx_1, vertex[1]) && numbers::isEqual(line_vtx_2, vertex[2]);
			}
		private:
			MyVec3 vertex[Geometry::vertexs_per_tri];
		};

	public:
		TriangleElem(const std::vector< VertexPtr >& vertexes);
		~TriangleElem();
	public:
		MyInt getID()const{ return m_nID; }
		void setId(MyInt id){ m_nID = id; }
		void get_dof_indices(MyVector9I &vecDofs);
		void get_postion(MyVector9& Pj_FEM);
		VertexPtr getVertex(unsigned idx){ return m_elem_vertex[idx]; }
		MyFloat getJxW(unsigned q)const{ return JxW_values[q]; }

		MyInt searchVtxIndexByVtxId(const MyInt nId);
		MyInt searchVtxIndexByVtxPos(const MyVec3& pos);

#if USE_Mantic_CMat
		bool isDiscontinuous()const{ return false; }
#endif
		/*void setTriElemDiscontinuous(){ m_TriElemType = DisContinuous; }
		bool isDiscontinuous()const{ return DisContinuous == m_TriElemType; }*/

		void setTriElemContinuous(){ m_TriElemType = Continuous; }		
		bool isContinuous()const{ return Continuous == m_TriElemType; }

		void setElemEndPoint(const MyVec3& endPt0, const MyVec3& endPt1, const MyVec3& endPt2)
		{
			m_elemEndPts.row(0) = endPt0.transpose(); 
			m_elemEndPts.row(1) = endPt1.transpose(); 
			m_elemEndPts.row(2) = endPt2.transpose();
		}
		void setElemVtxPoint(const MyVec3& vtxPt0, const MyVec3& vtxPt1, const MyVec3& vtxPt2)
		{ 
			m_elemVtxPts.row(0) = vtxPt0.transpose(); 
			m_elemVtxPts.row(1) = vtxPt1.transpose(); 
			m_elemVtxPts.row(2) = vtxPt2.transpose();
		}

		MyVec3 getElemEndPoint(MyInt idx)const{ return m_elemEndPts.row(idx).transpose(); }
		MyVec3 getElemVtxPoint(MyInt idx)const{ return m_elemVtxPts.row(idx).transpose(); }
		MyVec3 getElemNormals()const{ return m_TriElemNormal; }
		void setElemNormals(const MyVec3& _normals){ m_TriElemNormal = _normals; }

		void getKernelParameters(const MyInt gpt/*0,1,2*/, const MyVec3& collectPt, MyFloat& JxW, MyVec3& normals, MyFloat& r, MyVec3& dr, MyFloat& drdn);
		void calculateJumpTerm_smooth(MyMatrix_3X3& jumpTerm);
		//void calculateJumpTerm_Guiggiani(const MyVec3& n1, const MyVec3& n2, const MyFloat mu, MyMatrix_3X3& jumpTerm);
		void DBIEkernels(const MyInt i, const MyInt j, const MyFloat r, const MyVec3& dr, const MyFloat& drdn, const MyVec3& normals, MyFloat& Tij, MyFloat& Uij);
		void integrateHGsubmatrices_GLQ(const MyVec3& collocGlbPt, MyMatrix& Hsubmatrix, MyMatrix& Gsubmatrix);
		void compute_Shape_Deris_Jacobi();

		void initialize();
		void assembleSystemMatrix();
		void computeShapeFunction();
		void computeJxW();
		void computeShapeGrad();
		static void computeGaussPoint();
		void printElemInfo()const;

		MyVec3 getTriCenter()
		{			
			return (m_elemEndPts.row(0).transpose() + m_elemEndPts.row(1).transpose() + m_elemEndPts.row(2).transpose()) / 3;
		}
	public:
		static int getTriangleSize(){ return s_Triangle_Cache.size(); }
		static std::vector< TriangleElemPtr >& getTriangleVector(){ return s_Triangle_Cache; }
		static TriangleElemPtr getTriangle(int idx){ return s_Triangle_Cache[idx]; }
		static TriangleElemPtr makeTriangleElem(const MyVec3& vtxPt0, const MyVec3& vtxPt1, const MyVec3& vtxPt2);

		static MyFloat lineShapeFunction(const MyVec2ParamSpace localCoords, const MyInt n);
		static MyFloat lineDeris(const MyVec2ParamSpace localCoords, const MyInt n, const MyInt eta);
		static MyFloat jacobian(const MyVec3& vtxEndPt0, const MyVec3& vtxEndPt1, const MyVec3& vtxEndPt2);
		
		static MyFloat lineShapeFunction_1(const MyFloat x, const MyFloat x0, const MyFloat x1);
		static MyFloat lineShapeFunction_2(const MyFloat x, const MyFloat x0, const MyFloat x1);
		static MyFloat lineDeris_0(const MyFloat /*x*/, const MyFloat x0, const MyFloat x1);
		static MyFloat lineDeris_1(const MyFloat /*x*/, const MyFloat x0, const MyFloat x1);
	private:
		MyInt m_nID;
		VertexPtr m_elem_vertex[Geometry::vertexs_per_tri];
		MyFloat JxW_values[Geometry::vertexs_per_tri];
		MyMatrix_3X3 m_elemEndPts, m_elemVtxPts;
		MyVec3 m_TriElemNormal;

		/*row 0 is ( N1(XSI,ETA) = 1-XSI-ETA; N2(XSI,ETA) = XSI; N3(XSI,ETA) = ETA) of gauss 0 point ;
		  row 1 is ( N1(XSI,ETA) = 1-XSI-ETA; N2(XSI,ETA) = XSI; N3(XSI,ETA) = ETA) of gauss 1 point ;
		  row 2 is ( N1(XSI,ETA) = 1-XSI-ETA; N2(XSI,ETA) = XSI; N3(XSI,ETA) = ETA) of gauss 2 point ; */
		MyMatrix_3X3 shapeFunctionN;

		//      ^
		//    1 | 2
		//      | |.
		//    Y | | .
		//      | |  .
		//    0 | 0---1
		//      +------->
		//        0 X 1
		static MyVec2ParamSpace s_paramSpace[Geometry::vertexs_per_tri];//[0,+1]

		// 0 : (¦Î1, ¦Ç1) = (1/6, 1/6) , (¦Î2, ¦Ç2) = (2/3 , 1/6) , (¦Î3, ¦Ç3) = (1/6 , 2/3) , w1 = w2 = w3 = 1/3
		static MyVec2ParamSpace s_gaussPtInParamSpace[Geometry::vertexs_per_tri];//[-sqrt(2/3), +sqrt(2/3)]

		// w1 = w2 = w3 = 1/3
		static MyFloat s_gaussPtWeigth[Geometry::vertexs_per_tri];
		
		/*deris_dN[gauss  point id]:[N0, N1, N2] : [XSI, ETA]*/
		/*eg: deris_dN[2][1][0]: dN1 / dXSI of gauss point 2*/
		MyMatrix_3X2 deris_dN[Geometry::vertexs_per_tri];
		MyMatrix_3X3 Q_Discontinuous;
		TriElemType m_TriElemType;
		static std::vector< TriangleElemPtr > s_Triangle_Cache;


	};
}
#endif//_bemTriangleElem_h_