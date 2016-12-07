#ifndef _bemVertex_h_
#define _bemVertex_h_
#include "bemDefines.h"
#include <boost/smart_ptr.hpp>
#include "constant_numbers.h"
namespace VR
{
	class TriangleElem;
	typedef boost::shared_ptr< TriangleElem > TriangleElemPtr;

	class Vertex;
	typedef boost::shared_ptr< Vertex > VertexPtr;

	class Vertex
	{
		class VertexCompare
		{
		public:
			VertexCompare(MyFloat x, MyFloat y, MyFloat z) :m_point(x, y, z){}

			bool operator()(VertexPtr& p)
			{
				return  (numbers::isZero(m_point(0) - (*p).m_point(0))) &&
					(numbers::isZero(m_point(1) - (*p).m_point(1))) &&
					(numbers::isZero(m_point(2) - (*p).m_point(2)));
				/*return  (numbers::IsEqual(m_point(0) , (*p).m_point(0))) &&
				(numbers::IsEqual(m_point(1) , (*p).m_point(1))) &&
				(numbers::IsEqual(m_point(2) , (*p).m_point(2)));*/
			}
		private:
			MyDenseVector m_point;
		};

	public:
		Vertex(MyFloat x, MyFloat y, MyFloat z) :m_point(x, y, z), m_nID(Invalid_Id), m_dofs(Invalid_Id, Invalid_Id, Invalid_Id){}
		void setId(int id){ m_nID = id; }
		void setDof(unsigned dof_0, unsigned dof_1, unsigned dof_2){ vrASSERT(!isValidDofs()); m_dofs = MyVec3I(dof_0, dof_1, dof_2); }
	public:
		MyInt getId()const{ return m_nID; }
		MyVec3 getPos()const{ return m_point; }

		bool isValidDofs(){ return m_dofs[0] != Invalid_Id; }
		MyVec3I getDofs()const{ return m_dofs; }
		unsigned getDof(unsigned idx)const{ return m_dofs[idx]; }

		const std::vector< TriangleElemPtr >& getShareElement()const { return  m_vec_ShareTriangleElem; }
		void addShareTriangleElement(TriangleElemPtr triPtr){ m_vec_ShareTriangleElem.push_back(triPtr); }
		
	public:
		static VertexPtr makeVertex(const MyVec3& point);
		static VertexPtr searchVertex(const MyVec3& point);
		static void makeTriangleVertex(const std::vector< MyVec3 >& coords, std::vector< VertexPtr >& vertexes);
		static int getVertexSize(){ return s_vertexCache.size(); }
		static VertexPtr getVertex(int idx);
	private:
		MyVec3 m_point;
		MyInt m_nID;
		MyVec3I m_dofs;

		static std::vector< VertexPtr > s_vertexCache;
		static VertexPtr s_invalid_vertex;
	private:
		std::vector< TriangleElemPtr > m_vec_ShareTriangleElem;
	};
}
#endif//_bemVertex_h_