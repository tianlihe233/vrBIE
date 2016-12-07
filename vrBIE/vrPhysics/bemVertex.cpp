#include "bemVertex.h"

namespace VR
{
	std::vector< VertexPtr > Vertex::s_vertexCache;
	
	VertexPtr Vertex::s_invalid_vertex(new Vertex(Invalid_Id, Invalid_Id, Invalid_Id));

	VertexPtr Vertex::makeVertex(const MyPoint& point)
	{
		std::vector< VertexPtr >::reverse_iterator itr = std::find_if(s_vertexCache.rbegin(), s_vertexCache.rend(), VertexCompare(point[0], point[1], point[2]));
		if (s_vertexCache.rend() == itr)
		{
			//no find
			//printf("new pos {%f,%f,%f}\n",point[0],point[1],point[2]);
			s_vertexCache.push_back(VertexPtr(new Vertex(point[0], point[1], point[2])));
			s_vertexCache[s_vertexCache.size() - 1]->setId(s_vertexCache.size() - 1);
			return s_vertexCache[s_vertexCache.size() - 1];
		}
		else
		{
			//find it
			//Q_ASSERT(false);
			/*printf("{%f,%f,%f}\n",point[0],point[1],point[2]);
			MyPause;*/
			return (*itr);
		}
	}
	VertexPtr Vertex::searchVertex(const MyPoint& point)
	{
		std::vector< VertexPtr >::reverse_iterator itr = std::find_if(s_vertexCache.rbegin(), s_vertexCache.rend(), VertexCompare(point[0], point[1], point[2]));
		if (s_vertexCache.rend() == itr)
		{
			printf("vertex not find!");
			system("pause");
			return s_invalid_vertex;
			//no find

		}
		else
		{
			return (*itr);
		}
	}
	void Vertex::makeTriangleVertex(const std::vector< MyPoint >& coords, std::vector< VertexPtr >& vertexes)
	{
		vertexes.resize(coords.size());
		for (int v = 0; v < coords.size();++v)
		{
			vertexes[v] = makeVertex(coords[v]);
		}
	}

	VertexPtr Vertex::getVertex(int idx)
	{
		return s_vertexCache[idx];
	}

}//namespace VR