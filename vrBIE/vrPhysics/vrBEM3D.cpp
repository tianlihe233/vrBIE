#include "vrBEM3D.h"
#include <GL/glew.h>
#include <GL/freeglut.h>
#include "vrBase/vrLog.h"
#include "quad_rule.h"
#include "bemTriangleElem.h"
#include <fstream>
namespace VR
{
	//typedef enum{ Continuous, Discontinuous } TriElemType;
	struct TriangleSetWithType
	{
		//           2
		//          / .
		//         /   .
		//        /     .
		//       0-------1


		//      ^
		//    1 | 2
		//      | |.
		//    Y | | .
		//      | |  .
		//    0 | 0---1
		//      +------->
		//        0 X 1

		//param space vertex order
		//vertex 0 :¡¾ 0.0, 0.0¡¿
		//vertex 1 :¡¾ 1.0, 0.0¡¿
		//vertex 2 :¡¾ 0.0, 1.0¡¿
		//notice right hand identity

	public:

		void setVertexBoundary(MyInt idx, bool flag)
		{
			vertex_end_isBoundary[idx] = flag;
		}

		bool getVertexBoundary(MyInt idx)const
		{
			return vertex_end_isBoundary[idx];
		}
		
		const MyVec3& getEndVertex(MyInt idx)const
		{
			return vertex_end[idx];
		}

		const MyVec3& getCollectPoint(MyInt idx)const
		{
			return vertex_collection[idx];
		}
		const MyVec3 getTriangleNormal()const
		{
			return triNormal;
		}

		TriElemType getTriElemType()const
		{
			return type;
		}


		void setVertexPts(const MyVec3& vtxPts0, const MyVec3& vtxPts1, const MyVec3& vtxPts2)
		{
			vertex_end[0] = vtxPts0;
			vertex_end[1] = vtxPts1;
			vertex_end[2] = vtxPts2;
		}

		void setType(TriElemType _type)
		{
			type = _type;
		}

		void computeTriNormal()
		{
			triNormal = (vertex_end[1] - vertex_end[0]).cross(vertex_end[2] - vertex_end[1]);
			triNormal.normalize();
		}


		void computeCollectionPts()
		{

			if (Continuous == type)
			{
				vertex_collection[0] = vertex_end[0];
				vertex_collection[1] = vertex_end[1];
				vertex_collection[2] = vertex_end[2];
			}
			else
			{
				const MyFloat l = std::sqrt(2.0) / 2.0;
				const MyFloat l1 = l * 0.25;
				const MyFloat l2 = l * 0.75;
				const MyFloat s1 = l2 / l;
				const MyFloat s2 = l1 / (2 * l);
				MyVec2ParamSpace paramCoordsInDiscontinuous[MyDim] = { MyVec2ParamSpace(s2, s2), MyVec2ParamSpace(s1, s2), MyVec2ParamSpace(s2,s1 ) };

				reference_to_physical_t3(vertex_end[0], vertex_end[1], vertex_end[2], paramCoordsInDiscontinuous[0], vertex_collection[0]);
				reference_to_physical_t3(vertex_end[0], vertex_end[1], vertex_end[2], paramCoordsInDiscontinuous[1], vertex_collection[1]);
				reference_to_physical_t3(vertex_end[0], vertex_end[1], vertex_end[2], paramCoordsInDiscontinuous[2], vertex_collection[2]);
				/*infoLog << "param coords : " << paramCoordsInDiscontinuous[0].transpose() << " , " << paramCoordsInDiscontinuous[1].transpose() << " , " << paramCoordsInDiscontinuous[2].transpose() << std::endl;
				infoLog << "collect pt : " << vertex_collection[0].transpose() << " , " << vertex_collection[1].transpose() << " , " << vertex_collection[2].transpose() << std::endl;
				vrPause;*/
			}
		}

	private:
		MyVec3 vertex_end[Geometry::vertexs_per_tri];
		bool   vertex_end_isBoundary[Geometry::vertexs_per_tri];
		TriElemType type;
		MyVec3 triNormal;
		MyVec3 vertex_collection[Geometry::vertexs_per_tri];
		
	};

	MyVec3 computeTriNormal(const MyVec3& vtx0, const MyVec3& vtx1, const MyVec3& vtx2)
	{
		MyVec3 normals = (vtx1 - vtx0).cross(vtx2 - vtx1);
		normals.normalize();
		return normals;
	}


	vrBEM3D::vrBEM3D(const MyFloat _E , const MyFloat _mu ,
		const MyFloat _shearMod , const MyFloat _const4 ,
		const MyFloat _const3 , const MyFloat _const2 ,
		const MyFloat _const1 , const MyFloat _kappa )
		:E(_E), mu(_mu), shearMod(_shearMod), const4(_const4), const3(_const3), const2(_const2), const1(_const1), kappa(_kappa){}
	vrBEM3D::~vrBEM3D(){}

	int vrBEM3D::initPhysicalSystem(vrLpsz lpszObjName)
	{
		infoLog << "loadOBJ" << std::endl;
		m_ObjMesh_ptr = Geometry::MeshDataStructPtr(new Geometry::MeshDataStruct);
		m_ObjMesh_ptr->loadOBJ(lpszObjName, false, false);

		const std::vector< vrGLMVec3 >& points = m_ObjMesh_ptr->points;
		const std::vector<vrInt>& faces = m_ObjMesh_ptr->faces;
		infoLog << "make triangle struct." << std::endl;
		std::vector< TriangleSetWithType > vecTriSetWithType;



		for (int f = 0; f < faces.size();f+=3)
		{
			TriangleSetWithType node;
			const vrGLMVec3& point0 = points[faces[f]];
			const vrGLMVec3& point1 = points[faces[f+1]];
			const vrGLMVec3& point2 = points[faces[f+2]];
			node.setVertexPts(MyVec3(point0.x, point0.y, point0.z), MyVec3(point1.x, point1.y, point1.z), MyVec3(point2.x, point2.y, point2.z));
			
			node.computeTriNormal();

#if USE_Mantic_CMat
			node.setType(TriElemType::Continuous);

			for (int v = 0; v < Geometry::vertexs_per_tri;++v)
			{
				if (isDCCondition(node.getEndVertex(v)))
				{
					node.setVertexBoundary(v, true);
				}
				else
				{
					node.setVertexBoundary(v, false);
				}
			}
#else
			if (isDCCondition(node.getEndVertex(0)) && isDCCondition(node.getEndVertex(1)) && isDCCondition(node.getEndVertex(2)))
			{
				/*infoLog << "end pt : " << node.getEndVertex(0).transpose() << " , " << node.getEndVertex(1).transpose() << " , " << node.getEndVertex(2).transpose() << std::endl;
				vrPause;*/
				node.setType(TriElemType::DisContinuous);
				node.setBoundary(true);
			}
			else
			{
				node.setType(TriElemType::Continuous);
				node.setBoundary(false);
			}
#endif
			
			node.computeCollectionPts();
			vecTriSetWithType.push_back(node);

			
		}
		infoLog << "make triangle element." << std::endl;
		m_vec_triElem.clear();
		m_vec_vertex_boundary.clear();

		for (iterAllOf(ci, vecTriSetWithType))
		{
			const TriangleSetWithType& node = *ci;
			TriangleElemPtr newTriElemPtr = TriangleElem::makeTriangleElem(node.getCollectPoint(0), node.getCollectPoint(1), node.getCollectPoint(2));
			m_vec_triElem.push_back(newTriElemPtr);

#if USE_Mantic_CMat
			newTriElemPtr->setTriElemContinuous();
			for (int v=0;v<Geometry::vertexs_per_tri;++v)
			{
				if (node.getVertexBoundary(v))
				{
					m_vec_vertex_boundary.push_back( newTriElemPtr->getVertex(v) );
				}
			}
#else
			if (TriElemType::Continuous == node.getTriElemType())
			{
				newTriElemPtr->setTriElemContinuous();
			}
			else
			{
				newTriElemPtr->setTriElemDiscontinuous();
				if (node.IsBoundary())
				{
					m_vec_triElem_boundaryElement.push_back(newTriElemPtr);
				}

			}
#endif
			
			newTriElemPtr->setElemEndPoint(node.getEndVertex(0), node.getEndVertex(1), node.getEndVertex(2));
			/*infoLog << node.getCollectPoint(0).transpose() << " * " << node.getCollectPoint(1).transpose() << " * " << node.getCollectPoint(2).transpose() << std::endl;
			vrPause;*/
			newTriElemPtr->setElemVtxPoint(node.getCollectPoint(0), node.getCollectPoint(1), node.getCollectPoint(2));
			newTriElemPtr->setElemNormals(node.getTriangleNormal());
		}

#if USE_Mantic_CMat
		sortVertexConnectedVertexSurface();
#endif
		printf("TriangleElement size %d, vertex size %d, boundary vertex size %d \n", TriangleElem::getTriangleSize(), Vertex::getVertexSize(), m_vec_vertex_boundary.size());
		//vrPause;
		infoLog << "distributeDof2d();";
		distributeDof3d();
		infoLog << "createForceBoundaryCondition2d();";
		createForceBoundaryCondition3d();
		//createDCBoundaryCondition2d();
		infoLog << "computeFrenetSerretResortNormals();";
		//computeFrenetSerretResortNormals();
		infoLog << "createGMatrixAndHMatrixBEM3d();";
		createGMatrixAndHMatrixBEM3d();
		return 0;
	}

#if USE_Mantic_CMat

	vrFloat vrBEM3D::sgn(vrFloat val)
	{
		if (val > 0)
		{
			return 1.0;
		}
		else if (val <0)
		{
			return -1.0;
		}
		else
		{
			return 0.0;
		}
	}

	vrMat3 vrBEM3D::tensor_product(const MyVec3 vec_a, const MyVec3& vec_b)
	{
		vrMat3 retMat;

		
		for (int i = 0; i < 3;++i)
		{
			for (int j = 0; j < 3;++j)
			{
				retMat.coeffRef(i, j) = vec_a[i] * vec_b[j];
			}
		}
		return retMat;
	}

	void print_map_originId_dstId_set_trianleId(const std::map< std::pair<int, int>, std::set<int> >& map_originId_dstId_set_trianleId)
	{
		for (iterAllOf(ci, map_originId_dstId_set_trianleId))
		{
			auto myPair = (*ci).first;
			auto mySet = (*ci).second;

			
			std::cout << std::endl;
			for (iterAllOf(ci_in, mySet))
			{
				std::cout << "<< " << myPair.first << " , " << myPair.second << " >> " << *ci_in << std::endl;
			}
			std::cout << std::endl;
		}
	}

	void vrBEM3D::compute_Guiggiani_CMatrix_for_Test()
	{
		const vrFloat fai_1 = (1.0 / 6.0) * numbers::MyPI;
		const vrFloat fai_2 = (2.0 / 6.0) * numbers::MyPI;
		const vrFloat theta = (62.632 / 180.0) * numbers::MyPI;
		/*const vrFloat fai_1 = (0.0 / 6.0) * numbers::MyPI;
		const vrFloat fai_2 = (3.0 / 6.0) * numbers::MyPI;
		const vrFloat theta = (90.0 / 180.0) * numbers::MyPI;*/

		const vrFloat C = 1.0 / (8*numbers::MyPI * 0.7);
		const vrFloat a = 0.4;
		vrMat3 CMat;
		CMat.coeffRef(0, 0) = C * (( ( ((fai_2 - fai_1) / 2.0) + ((std::sin(2*fai_2)-std::sin(2*fai_1))/4.0))* (2.0 - 3.0 * std::cos(theta) + std::pow(std::cos(theta),3))) + (a * (fai_2-fai_1) * (1 - std::cos(theta))));
		CMat.coeffRef(1, 1) = CMat.coeffRef(0, 0);
		CMat.coeffRef(2, 2) = C * (((fai_2 - fai_1)*(1 - std::pow(std::cos(theta), 3))) + (a * (fai_2 - fai_1) * (1 - std::cos(theta))));
		CMat.coeffRef(0, 1) = CMat.coeffRef(1, 0) = C * ((std::pow(std::sin(fai_2), 2) - std::pow(std::sin(fai_1), 2)) / 2.0) * (2.0 - 3.0 * std::cos(theta) + std::pow(std::cos(theta), 3));
		CMat.coeffRef(0, 2) = CMat.coeffRef(2, 0) = C * (std::sin(fai_2) - std::sin(fai_1)) * (std::pow(std::sin(theta),3));
		CMat.coeffRef(1, 2) = CMat.coeffRef(2, 1) = C * (vrNotice/*std::cos(fai_2) - std::cos(fai_1)*/std::cos(fai_1) - std::cos(fai_2)) * (std::pow(std::sin(theta), 3));

		std::cout << "compute_Guiggiani_CMatrix_for_Test : " << std::endl << CMat << std::endl;
	}

	void vrBEM3D::sortVertexConnectedVertexSurface()
	{
		/*
		1. r(v1) = v1 - x, the length of the vector r is one ? 
		2. the mantic matrix is suit for tangent surface is not along the coordinate see figure 5 in A General Algorithm for Multidimensional Cauchy Principal Value Integrals in the Boundary Element Method
		*/
		std::ofstream outfile("D:/vrBIE/out.txt");
		const MyInt vertexSize = Vertex::getVertexSize();
		for (int v = 0; v < vertexSize; ++v)
		{
			VertexPtr curVtxPtr = Vertex::getVertex(v);
			const MyInt originVtxId = curVtxPtr->getId();
			const MyVec3 origin_x = curVtxPtr->getPos();

			std::map< std::pair<int, int>, std::set<int> > map_originId_dstId_set_trianleId;

			const std::vector< TriangleElemPtr >& vecShareElement = curVtxPtr->getShareElement();
			Q_ASSERT(vecShareElement.size()>1);

			outfile << "vertex id = " << v << "  pos : " << origin_x.transpose() << std::endl;
			std::cout << "vertex id = " << v << "  pos : " << origin_x.transpose() << std::endl;
			std::cout << "vecShareElement.size is " << vecShareElement.size() << std::endl;
			for (iterAllOf(ci, vecShareElement))
			{
				const TriangleElemPtr triPtr = *ci;
				const MyInt triId = triPtr->getID();
				const MyInt originIndexInTri = triPtr->searchVtxIndexByVtxId(originVtxId);
				Q_ASSERT(Invalid_Id != originIndexInTri);

				{
					//line 1
					VertexPtr vtx_1_Ptr = triPtr->getVertex((originIndexInTri + 1) % Geometry::vertexs_per_tri);
					const MyInt vtx_1_id = vtx_1_Ptr->getId();
					map_originId_dstId_set_trianleId[std::make_pair(originVtxId, vtx_1_id)].insert(triId);

					//outfile << (curVtxPtr->getPos()).transpose() << "  " << (vtx_1_Ptr->getPos()).transpose() << "  " << triId << std::endl;
				}
				{
					//line 2
					VertexPtr vtx_2_Ptr = triPtr->getVertex((originIndexInTri + 2) % Geometry::vertexs_per_tri);
					const MyInt vtx_2_id = vtx_2_Ptr->getId();
					map_originId_dstId_set_trianleId[std::make_pair(originVtxId, vtx_2_id)].insert(triId);
					//outfile << (curVtxPtr->getPos()).transpose() << "  " << (vtx_2_Ptr->getPos()).transpose() << "  " << triId << std::endl;
				}
			}
			print_map_originId_dstId_set_trianleId(map_originId_dstId_set_trianleId);
			{
				
				std::map< vrInt, VertexPtr > mapSortedVertex;
				std::map< std::pair<vrInt, vrInt>, TriangleElemPtr > normal_i_i_plus_1;
				vrInt index_i = 0/*, index_i_plus_1 = 1*/;
				TriangleElemPtr triPtr = *(vecShareElement.begin());
				MyInt currentTriangleId = triPtr->getID();
				MyInt originIndexInCurrentTriangle = triPtr->searchVtxIndexByVtxId(originVtxId);
				Q_ASSERT(Invalid_Id != originIndexInCurrentTriangle);

				MyInt vtx_1_index_InCurrentTriangle = (originIndexInCurrentTriangle + 1) % Geometry::vertexs_per_tri;
				VertexPtr vtx_1_Ptr = triPtr->getVertex(vtx_1_index_InCurrentTriangle);
				MyInt vtx_1_id = vtx_1_Ptr->getId();

				mapSortedVertex[index_i] = vtx_1_Ptr;

				std::set<int>& refSet = map_originId_dstId_set_trianleId.at(std::make_pair(originVtxId, vtx_1_id));
				auto itr = refSet.find(currentTriangleId);
				Q_ASSERT(refSet.end() != itr);
				refSet.erase(itr);
				Q_ASSERT(1 == refSet.size());

				VertexPtr nextVtxPtr = triPtr->getVertex(index2to1(originIndexInCurrentTriangle, vtx_1_index_InCurrentTriangle));
				MyInt nextVtxId = nextVtxPtr->getId();

				
				while (!map_originId_dstId_set_trianleId.empty())
				{
					print_map_originId_dstId_set_trianleId(map_originId_dstId_set_trianleId);
					{
						
						std::set<int>& refTriSet = map_originId_dstId_set_trianleId.at(std::make_pair(originVtxId, nextVtxId));
						auto eraseItr = refTriSet.find(currentTriangleId);
						Q_ASSERT(eraseItr != refTriSet.end());

						if (map_originId_dstId_set_trianleId.size() > 1 )
						{
							Q_ASSERT( 2 == refTriSet.size());
							refTriSet.erase(eraseItr);
							{
								//compute normal
								normal_i_i_plus_1[std::make_pair(vtx_1_id, nextVtxId)] = triPtr;
							}
							Q_ASSERT(1 == refTriSet.size());
							currentTriangleId = (*refTriSet.begin());

							auto eraseItr_1 = map_originId_dstId_set_trianleId.find(std::make_pair(originVtxId, nextVtxId));
							map_originId_dstId_set_trianleId.erase(eraseItr_1);

							mapSortedVertex[++index_i] = nextVtxPtr;
						}
						else
						{
							Q_ASSERT( 1 == refTriSet.size());
							currentTriangleId = (*refTriSet.begin());

							auto eraseItr_1 = map_originId_dstId_set_trianleId.find(std::make_pair(originVtxId, nextVtxId));
							map_originId_dstId_set_trianleId.erase(eraseItr_1);

							Q_ASSERT(mapSortedVertex.size() == (index_i + 1));
							mapSortedVertex[index_i + 1] = mapSortedVertex[0];
							mapSortedVertex[-1] = mapSortedVertex[index_i];
							
							{
								//compute normal
								normal_i_i_plus_1[std::make_pair(vtx_1_id, nextVtxId)] = triPtr;
							}
						}
						
					}
					triPtr = TriangleElem::getTriangle(currentTriangleId);
					vtx_1_id = nextVtxId;
					originIndexInCurrentTriangle = triPtr->searchVtxIndexByVtxId(originVtxId);
					vtx_1_index_InCurrentTriangle = triPtr->searchVtxIndexByVtxId(nextVtxId);
					Q_ASSERT(Invalid_Id != vtx_1_index_InCurrentTriangle && Invalid_Id != originIndexInCurrentTriangle && originIndexInCurrentTriangle != vtx_1_index_InCurrentTriangle);

					nextVtxPtr = triPtr->getVertex(index2to1(originIndexInCurrentTriangle, vtx_1_index_InCurrentTriangle));
					nextVtxId = nextVtxPtr->getId();
				}

				std::map< vrInt, MyVec3 > map_ri;

				const vrInt n = mapSortedVertex.size() - 2;
				
				for (iterAllOf(ci, mapSortedVertex))
				{
					const vrInt vv  = (*ci).first;
					const MyVec3 vi = (*ci).second->getPos();
					//r(vi) = vi - x
					map_ri[vv] = (vi - origin_x);
					map_ri[vv].normalize(); vrNotice;
				}

				vrFloat value_0 = 0.0;
				vrMat3  value_1; value_1.setZero();
				for (int vv = 0; vv < n;++vv)
				{
					const vrInt i = vv;
					const vrInt vid_i = mapSortedVertex.at(i)->getId();
					const vrInt vid_iplus1 = mapSortedVertex.at(i + 1)->getId();
					const vrInt vid_isub1 = mapSortedVertex.at(i - 1)->getId();

					const MyVec3& n_isub1_i = normal_i_i_plus_1.at(std::make_pair(/*i - 1*/vid_isub1,/* i*/vid_i))->getElemNormals();
					const MyVec3& n_i_iplus1 = normal_i_i_plus_1.at(std::make_pair(/* i*/vid_i, /*i + 1*/vid_iplus1))->getElemNormals();
					const MyVec3& r_i		 = map_ri.at(i);
					const MyVec3& r_iplus1	 = map_ri.at(i+1);

					value_0 += sgn((n_isub1_i.cross(n_i_iplus1)).dot(r_i)) * std::acos(n_isub1_i.dot(n_i_iplus1));

					value_1 += tensor_product((r_iplus1 - r_i).cross(n_i_iplus1), n_i_iplus1);
				}

				vrMat3 Cmatrix = (1.0 / (4.0 * numbers::MyPI)) * (2 * numbers::MyPI + value_0) * vrMat3::Identity();
				Cmatrix -= (1.0 / (8.0*numbers::MyPI * (1 - mu))) * value_1;

				
				outfile << Cmatrix << std::endl;

				if (2 == v)
				{
					/*vertex id = 0  pos : 0.866025      0.5      0.5
						vertex id = 1  pos : 0 0 1
						vertex id = 2  pos : 0 0 0
						vertex id = 3  pos : 0.5 0.866025      0.5*/
					for (int tri = 0; tri < TriangleElem::getTriangleSize();++tri )
					{
						TriangleElemPtr  curTriPtr = TriangleElem::getTriangle(tri);

						std::cout << "triangle " << tri << " : " << (curTriPtr->getVertex(0)->getPos().transpose()) << " , " << curTriPtr->getVertex(0)->getId() << " ## " << (curTriPtr->getVertex(1)->getPos().transpose()) << " , " << curTriPtr->getVertex(1)->getId() << " ## " << (curTriPtr->getVertex(2)->getPos().transpose()) << " , " << curTriPtr->getVertex(2)->getId() << std::endl;
					}


					{
						MyVec3 n1 = TriangleElem::getTriangle(0)->getElemNormals();
						MyVec3 n2(0, 1, 0);
						vrFloat t0 = (acos(n1.dot(n2) / (n1.norm()*n2.norm())) / numbers::MyPI) * 180;

						std::cout << "arc 0 = " << t0 << std::endl;
					}
					{
						MyVec3 n1 = TriangleElem::getTriangle(1)->getElemNormals();
						MyVec3 n2(0, 1, 0);
						vrFloat t1 = (acos(n1.dot(n2) / (n1.norm()*n2.norm())) / numbers::MyPI) * 180;

						std::cout << "arc 1 = " << t1 << std::endl;
					}
					{
						MyVec3 n1 = TriangleElem::getTriangle(2)->getElemNormals();
						MyVec3 n2(0, 0, 1);
						vrFloat t2 = (acos(n1.dot(n2) / (n1.norm()*n2.norm())) / numbers::MyPI) * 180;

						std::cout << "arc 2 = " << t2 << std::endl;
					}
					vrPause;
				}

				/*outfile << "sorted point : " << originVtxId << std::endl;
				for (iterAllOf(ci, mapSortedVertex))
				{
					outfile << "(" << (*ci).first << " , " << (*ci).second->getPos().transpose() << " , " << (*ci).second->getId() << ") * ";
				}
				outfile << std::endl;

				outfile << "sorted surface : " << std::endl;
				for (iterAllOf(ci, normal_i_i_plus_1))
				{
					outfile << "[" << (*ci).first.first << " , " << (*ci).first.second << "] * ";
				}
				outfile << std::endl;

				outfile << "point size is " << mapSortedVertex.size() << "  tangent surface size is " << normal_i_i_plus_1.size() << std::endl;*/


			}
		}
		compute_Guiggiani_CMatrix_for_Test();
		outfile.close();
		vrExit;
	}
#endif

	void vrBEM3D::createGMatrixAndHMatrixBEM3d()
	{
		const MyInt nPts = Vertex::getVertexSize();
		const MyInt ne = TriangleElem::getTriangleSize();
		const MyInt nDofs = getDofs();

		m_Hsubmatrix.resize(nDofs, nDofs); m_Hsubmatrix.setZero();
		m_Gsubmatrix.resize(nDofs, nDofs); m_Gsubmatrix.setZero();
		m_A_matrix.resize(nDofs, nDofs); m_A_matrix.setZero();
		m_rhs.resize(nDofs); m_rhs.setZero();

		for (int element = 0; element < ne; ++element)
		{
			TriangleElem::getTriangle(element)->compute_Shape_Deris_Jacobi();
		}

		MyMatrix Hsubmatrix, Gsubmatrix;
		for (int c = 0; c < nPts; ++c)
		{
			VertexPtr curSourcePtPtr = Vertex::getVertex(c);
			const MyVec3 collocGlbPt = curSourcePtPtr->getPos();
			
			const MyInt curSourcePtId = curSourcePtPtr->getId();

			for (int element = 0; element < ne; ++element)
			{
				TriangleElemPtr curElemPtr = TriangleElem::getTriangle(element);
				const MyInt curElemId = curElemPtr->getID();
				bool ptInElem = false;


				if (ptInElem && false)
				{
					/*MyMatrix_2X2 jumpTerm;
					if (isElemDiscontinuous)
					{
					curElemPtr->calculateJumpTerm_smooth(jumpTerm);
					}
					else
					{
					curElemPtr->calculateJumpTerm_Guiggiani(collectPtNormal[0], collectPtNormal[1], mu, jumpTerm);
					}*/


				}
				else
				{
					//we have a regular integral
					/*MyMatrix_3X3 jumpTerm;
					curElemPtr->calculateJumpTerm_smooth(jumpTerm);*/
					curElemPtr->integrateHGsubmatrices_GLQ(collocGlbPt, Hsubmatrix, Gsubmatrix);
					//[Hsubmatrix, Gsubmatrix] = integrateHGsubmatrices_GLQ(ngp_r, elcoords, glbBsFnConn, collocGlbPt, range);
				}
				MyVector9I sctrVec;
				curElemPtr->get_dof_indices(sctrVec);

				MyVec3I rowSctrVec = curSourcePtPtr->getDofs();

				for (int r = 0; r < rowSctrVec.size(); ++r)
				{
					const MyInt rowId = rowSctrVec[r];

					for (int c = 0; c < sctrVec.size(); ++c)
					{
						const MyInt colId = sctrVec[c];
						m_Hsubmatrix.coeffRef(rowId, colId) += Hsubmatrix.coeff(r, c);
						m_Gsubmatrix.coeffRef(rowId, colId) += Gsubmatrix.coeff(r, c);
					}
				}

			}// for all element
		}//for collect point

		makeRigidH();

		MyFloat trace_H = 0.0;
		MyFloat trace_G = 0.0;

		for (int row = 0; row < nDofs; ++row)
		{
			trace_H += m_Hsubmatrix.coeff(row, row);
			trace_G += m_Gsubmatrix.coeff(row, row);
		}
		const MyFloat SF = abs(trace_H / trace_G);

		std::set< MyInt > boundaryDofsSet;

#if USE_Mantic_CMat
		for (iterAllOf(ci, m_vec_vertex_boundary))
		{
			const VertexPtr  curVtxPtr = *ci;
			MyVec3I dofs = curVtxPtr->getDofs();
			for (int i = 0; i < dofs.size(); ++i)
			{
				boundaryDofsSet.insert(dofs[i]);
			}
		}
#else
		for (iterAllOf(ci, m_vec_triElem_boundaryElement))
		{
			const TriangleElemPtr  curElemPtr = *ci;
			MyVector9I dofs;
			curElemPtr->get_dof_indices(dofs);
			for (int i = 0; i < dofs.size(); ++i)
			{
				boundaryDofsSet.insert(dofs[i]);
			}
		}
#endif
		

		//move displacement unknow

		for (int d = 0; d < nDofs; ++d)
		{
			if (!Iterator::has_elt(boundaryDofsSet, d))
			{
				MyNotice;//notice is col, not row
				m_A_matrix.col(d) = m_Hsubmatrix.col(d);
			}
		}

		for (iterAllOf(ci, boundaryDofsSet))
		{
			const MyInt traceId = *ci;
			m_A_matrix.col(traceId) = m_Gsubmatrix.col(traceId) * SF * -1.0;
		}

		//make right hand side
		MyVector dirichletVals(nDofs);
		dirichletVals.setZero();

#if USE_Mantic_CMat
		for (iterAllOf(ci, m_vec_vertex_boundary))
		{
			MyVec3I dofs = (*ci)->getDofs();
			for (int v = 0; v < dofs.size(); ++v)
			{
				dirichletVals[dofs[v]] = 0.0;
			}
		}
#else
		for (iterAllOf(ci, m_vec_triElem_boundaryElement))
		{
			MyVector9I dofs;
			(*ci)->get_dof_indices(dofs);
			for (int v = 0; v < dofs.size(); ++v)
			{
				dirichletVals[dofs[v]] = 0.0;
			}
		}
#endif
		m_rhs = m_rhs - m_Hsubmatrix * dirichletVals;

		MyVector externalForceVals(nDofs);
		externalForceVals.setZero();
		m_trace3d[0] = 400.0;
		m_trace3d[1] = 0.0;
		m_trace3d[2] = 0.0;
		for (iterAllOf(ci, m_vec_vertex2d_trace))
		{
			MyVec3I dofs = (*ci)->getDofs();
			externalForceVals[dofs[0]] = m_trace3d[0];
			externalForceVals[dofs[1]] = m_trace3d[1];
			externalForceVals[dofs[2]] = m_trace3d[2];
		}

		m_rhs = m_rhs + m_Gsubmatrix * externalForceVals;

		//m_displacement = m_A_matrix.inverse() * m_rhs;
		//SHOWMATRIX(m_A_matrix);
		//m_displacement = GaussElimination(m_A_matrix, m_rhs);
		m_displacement = m_A_matrix.inverse() * m_rhs;
		//m_displacement = m_rhs * 100;
#if 0
		MyMatrix outputMat(m_vec_vertex2d_trace.size()*MyDim, m_A_matrix.cols());
		MyVector outputRHS(m_vec_vertex2d_trace.size()*MyDim);
		MyVector outputDisplace(m_vec_vertex2d_trace.size()*MyDim);
		int i = 0;
		for (iterAllOf(ci, m_vec_vertex2d_trace))
		{
			VertexPtr ptr = *ci;
			MyVec3I dofs = ptr->getDofs();
			outputMat.row(i + 0) = m_A_matrix.row(dofs[0]);
			outputMat.row(i + 1) = m_A_matrix.row(dofs[1]);
			outputMat.row(i + 2) = m_A_matrix.row(dofs[2]);
			outputRHS[i + 0] = m_rhs[dofs[0]];
			outputRHS[i + 1] = m_rhs[dofs[1]];
			outputRHS[i + 2] = m_rhs[dofs[2]];

			outputDisplace[i + 0] = m_displacement[dofs[0]];
			outputDisplace[i + 1] = m_displacement[dofs[1]];
			outputDisplace[i + 2] = m_displacement[dofs[2]];
			i += 3;
		}

		std::ofstream outfile("d:/out.txt");
		outfile << outputMat << std::endl << std::endl << std::endl << outputRHS.transpose() << std::endl << std::endl << std::endl << outputDisplace.transpose() << std::endl;
		outfile.close();
#endif

		/*for (iterAllOf(ci, m_vec_triElem_boundaryElement))
		{
			MyVector9I dofs;
			(*ci)->get_dof_indices(dofs);
			for (int v = 0; v < dofs.size(); ++v)
			{
				m_displacement[dofs[v]] = 0.0;
			}
		}*/
	}

	void vrBEM3D::makeRigidH()
	{
		const MyInt nDofs = getDofs();
		MyMatrix& currentH = m_Hsubmatrix;
		//rigid H
		const MyInt nVtx = Vertex::getVertexSize();

		for (int row_v = 0; row_v < nVtx; ++row_v)
		{
			MyFloat sum = 0.0;
			//for each vertex
			const MyVec3I& rowDofs = Vertex::getVertex(row_v)->getDofs();
			for (int d = 0; d < MyDim; ++d)
			{
				//for each dimension
				const MyInt curRowId = rowDofs[d];
				for (int col_v = 0; col_v < nVtx; ++col_v)
				{
					if (col_v != row_v)
					{
						const MyInt curColId = Vertex::getVertex(col_v)->getDofs()[d];
						sum += currentH.coeff(curRowId, curColId);
					}
				}

				currentH.coeffRef(curRowId, curRowId) = -1.0* sum;
			}
		}
	}

	void vrBEM3D::createForceBoundaryCondition3d()
	{
		m_vec_vertex2d_trace.clear();
		
		for (iterAllOf(ci, m_vec_triElem))
		{
			
			TriangleElemPtr triPtr = *ci;
			const MyInt triId = triPtr->getID();
			//if (triPtr->isContinuous() /*&& (153 == triId || 158 == triId || 159 == triId || 164 == triId || 165 == triId)*/)
			{
				const MyVec3& vtxPos0 = triPtr->getVertex(0)->getPos();
				const MyVec3& vtxPos1 = triPtr->getVertex(1)->getPos();
				const MyVec3& vtxPos2 = triPtr->getVertex(2)->getPos();
				if (isForceCondition(vtxPos0) && isForceCondition(vtxPos1) && isForceCondition(vtxPos2))
				{
					m_vec_vertex2d_trace.push_back(triPtr->getVertex(0));
					m_vec_vertex2d_trace.push_back(triPtr->getVertex(1));
					m_vec_vertex2d_trace.push_back(triPtr->getVertex(2));

					//std::cout << "triPtr->getID() :" << triPtr->getID() << std::endl;
				}
			}
		}
		/*const MyInt nVtx = Vertex::getVertexSize();
		for (int v = 0; v < nVtx; ++v)
		{
			VertexPtr  vtxPtr = Vertex::getVertex(v);
			MyVec3 vtxPos = vtxPtr->getPos();
			if (isForceCondition(vtxPos))
			{
				m_vec_vertex2d_trace.push_back(vtxPtr);
			}
		}*/
		infoLog << "m_vec_vertex2d_trace.size " << m_vec_vertex2d_trace.size() << std::endl;
		vrASSERT(m_vec_vertex2d_trace.size() > 0);
	}

	void vrBEM3D::distributeDof3d()
	{
		m_nGlobalDof = Geometry::first_dof_idx;
		const MyInt nTriSize = TriangleElem::getTriangleSize();
		for (MyInt c = 0; c < nTriSize; ++c)
		{
			TriangleElemPtr curTriPtr = TriangleElem::getTriangle(c);
			for (MyInt v = 0; v < Geometry::vertexs_per_tri; ++v)
			{
				VertexPtr curVtxPtr = curTriPtr->getVertex(v);
				if (!(curVtxPtr->isValidDofs()))
				{
					curVtxPtr->setDof(m_nGlobalDof, m_nGlobalDof + 1, m_nGlobalDof + 2);
					m_nGlobalDof += MyDim;
				}
			}
		}
		printf("global dof %d.\n", m_nGlobalDof);
	}

	bool vrBEM3D::isForceCondition(const MyVec3& pos)
	{
		if (pos[2] >= (0.0) && pos[2] < 0.2 && pos[0] >= 0.1 )
		{
			return true;
		}
		else
		{
			return false;
		}
	}

	bool vrBEM3D::isDCCondition(const MyVec3& pos)
	{
		/*static MyFloat min = FLT_MAX;
		if (min > pos[2])
		{
			min = pos[2];

			infoLog << "pos[2] " << pos[2] << std::endl;
			vrPause;
		}*/
		if (pos[2] <= (-0.5))
		{
			return true;
		}
		else
		{
			return false;
		}
	}

	void Draw_Axes(void)
	{
		float ORG[3] = { 0, 0, 0 };

		float XP[3] = { 1, 0, 0 }, XN[3] = { -1, 0, 0 },
			YP[3] = { 0, 1, 0 }, YN[3] = { 0, -1, 0 },
			ZP[3] = { 0, 0, 1 }, ZN[3] = { 0, 0, -1 };

		glPushMatrix();
		glLineWidth(2.0);

		glBegin(GL_LINES);
		glColor3f(1, 0, 0); // X axis is red.
		glVertex3fv(ORG);
		glVertex3fv(XP);
		glColor3f(0, 1, 0); // Y axis is green.
		glVertex3fv(ORG);
		glVertex3fv(YP);
		glColor3f(0, 0, 1); // z axis is blue.
		glVertex3fv(ORG);
		glVertex3fv(ZP);
		glEnd();

		glPopMatrix();
	}

	void vrBEM3D::renderScene() const
	{
		const std::vector< vrGLMVec3 >& points = m_ObjMesh_ptr->points;
		const std::vector<vrInt>& faces = m_ObjMesh_ptr->faces;

		glBegin(GL_TRIANGLES);
		for (iterAllOf(ci, m_vec_triElem))
		{
			TriangleElemPtr ptr = *ci;
			for (int i = 0; i < MyDim;++i)
			{
				const MyVec3& vtx = ptr->getVertex(i)->getPos();
				const MyVec3I dofs = ptr->getVertex(i)->getDofs();
				glVertex3d(vtx[0] + m_displacement[dofs[0]], vtx[1] + m_displacement[dofs[1]], vtx[2] + m_displacement[dofs[2]]);
				//glVertex3d(vtx[0] , vtx[1] , vtx[2] );
			}

		}	
		
		/*for (iterAllOf(ci, m_vec_vertex2d_trace))
		{
			const MyVec3I dofs = (*ci)->getDofs();
			printf("(%f,%f,%f)\n", m_displacement[dofs[0]], m_displacement[dofs[1]], m_displacement[dofs[2]]);
		}*/
		glEnd();

		glLineWidth(2.0);
		glColor3f(0,0,0);
		glBegin(GL_LINES);
		int linePair[MyDim][2] = { { 0, 1 }, { 1, 2 }, { 2, 0 } };
		for (iterAllOf(ci, m_vec_triElem))
		{
			TriangleElemPtr ptr = *ci;
			for (int i = 0; i < MyDim; ++i)
			{
				for (int j = 0; j < 2;++j)
				{
					const MyVec3& vtx = ptr->getVertex(linePair[i][j])->getPos();
					const MyVec3I dofs = ptr->getVertex(linePair[i][j])->getDofs();
					glVertex3d(vtx[0] + m_displacement[dofs[0]], vtx[1] + m_displacement[dofs[1]], vtx[2] + m_displacement[dofs[2]]);
					//glVertex3d(vtx[0], vtx[1], vtx[2]);
				}
			}

			
		}
		/*for (iterAllOf(ci, m_vec_triElem))
		{
			TriangleElemPtr ptr = *ci;
			const MyVec3& center = ptr->getTriCenter();
			MyVec3 endd = center + ptr->getElemNormals();
			glVertex3d(center[0], center[1], center[2]);
			glVertex3d(endd[0], endd[1], endd[2]);
		}*/
		glEnd();
	}

	MyVector vrBEM3D::GaussElimination(const MyMatrix& K, MyVector& b)
	{
		const MyInt n = K.rows();

		MyMatrix AK(n, n + 1); AK.setZero();
		AK.block(0, 0, n, n) = K;
		AK.block(0, n, n, 1) = b;
		for (MyInt i = 0; i < n; i++) {
			// Search for maximum in this column
			MyFloat maxEl = abs(AK.coeff(i, i)/*[i][i]*/);
			MyInt maxRow = i;
			for (MyInt k = i + 1; k<n; k++) {
				if (abs(AK.coeff(k, i)/*[k][i]*/) > maxEl) {
					maxEl = abs(AK.coeff(k, i)/*[k][i]*/);
					maxRow = k;
				}
			}

			// Swap maximum row with current row (column by column)
			for (MyInt k = i; k < n + 1; k++) {
				MyFloat tmp = AK.coeff(maxRow, k);//A[maxRow][k];
				/*A[maxRow][k]*/AK.coeffRef(maxRow, k) = AK.coeff(i, k);//A[i][k];
				/*A[i][k]*/AK.coeffRef(i, k) = tmp;
			}

			// Make all rows below this one 0 in current column
			for (MyInt k = i + 1; k < n; k++) {
				MyFloat c = -AK.coeff(k, i) / AK.coeff(i, i);/*A[k][i]/A[i][i]*/;
				for (MyInt j = i; j < n + 1; j++) {
					if (i == j) {
						/*A[k][j]*/ AK.coeffRef(k, j) = 0;
					}
					else {
						/*A[k][j]*/AK.coeffRef(k, j) += c * AK.coeff(i, j) /*A[i][j]*/;
					}
				}
			}
		}

		// Solve equation Ax=b for an upper triangular matrix A
		MyVector x(n);
		for (MyInt i = n - 1; i >= 0; i--) {
			x[i] = AK.coeff(i, n) / AK.coeff(i, i);//A[i][n]/A[i][i];
			for (MyInt k = i - 1; k >= 0; k--) {
				/*A[k][n]*/AK.coeffRef(k, n) -= /*A[k][i]*/AK.coeff(k, i) * x[i];
			}
		}
		return x;
	}
}