#include "bemTriangleElem.h"
#include "vrBase/vrLog.h"
#include <boost/math/constants/constants.hpp> // pi
namespace VR
{
	std::vector< TriangleElemPtr > TriangleElem::s_Triangle_Cache;

	MyVec2ParamSpace TriangleElem::s_paramSpace[Geometry::vertexs_per_tri] = { MyVec2ParamSpace(0.0,0.0), MyVec2ParamSpace(1.0,0.0), MyVec2ParamSpace(0.0,1.0)};//[0,+1]
	// 0 : (¦Î1, ¦Ç1) = (1/6, 1/6) , (¦Î2, ¦Ç2) = (2/3 , 1/6) , (¦Î3, ¦Ç3) = (1/6 , 2/3) , w1 = w2 = w3 = 1/3
	MyVec2ParamSpace TriangleElem::s_gaussPtInParamSpace[Geometry::vertexs_per_tri] = { MyVec2ParamSpace(1.0 / 6.0, 1.0 / 6.0), MyVec2ParamSpace(2.0 / 3.0, 1.0 / 6.0), MyVec2ParamSpace(1.0 / 6.0, 2.0 / 3.0) };//[-sqrt(2/3), +sqrt(2/3)]
	MyFloat TriangleElem::s_gaussPtWeigth[Geometry::vertexs_per_tri] = { 1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0};

	TriangleElemPtr TriangleElem::makeTriangleElem(const MyVec3& vtxPt0, const MyVec3& vtxPt1, const MyVec3& vtxPt2)
	{
		std::vector< TriangleElemPtr >::reverse_iterator itr = std::find_if(s_Triangle_Cache.rbegin(), s_Triangle_Cache.rend(), TriangleElemCompare(vtxPt0, vtxPt1, vtxPt2));
		if (s_Triangle_Cache.rend() == itr)
		{
			//no find
			std::vector< MyVec3 > coords;
			std::vector< VertexPtr > vertexes;
			coords.push_back(vtxPt0);
			coords.push_back(vtxPt1);
			coords.push_back(vtxPt2);
			Vertex::makeTriangleVertex(coords, vertexes);

			TriangleElemPtr curTriangleElemPtr(new TriangleElem(vertexes));
			s_Triangle_Cache.push_back(curTriangleElemPtr);
			curTriangleElemPtr->setId(s_Triangle_Cache.size() - 1);

			for (int v = 0; v < Geometry::vertexs_per_tri; ++v)
			{
				vertexes[v]->addShareTriangleElement(curTriangleElemPtr); 
			}
			return curTriangleElemPtr;
		}
		else
		{
			//find it
			//Q_ASSERT(false);
			//printf("{%f,%f,%f,%f}\n",point[0],point[1],point[2],radius);
			return (*itr);
		}
	}

	MyInt TriangleElem::searchVtxIndexByVtxId(const MyInt nId)
	{
		for (int v = 0; v < Geometry::vertexs_per_tri;++v)
		{
			if (nId == (m_elem_vertex[v]->getId()))
			{
				return v;
			}
		}
		return Invalid_Id;
	}

	MyInt TriangleElem::searchVtxIndexByVtxPos(const MyVec3& pos)
	{
		for (int v = 0; v < Geometry::vertexs_per_tri; ++v)
		{
			if (numbers::IsEqual(m_elem_vertex[v]->getPos(), pos))
			{
				return v;
			}
		}
		return Invalid_Id;
	}

	MyFloat TriangleElem::lineShapeFunction(const MyVec2ParamSpace localCoords, const MyInt n)
	{
		switch (n)
		{
		case 0:
		{
				  return 1.0 - localCoords[0] - localCoords[1];
				  break;
		}
		case 1:
		{
				  return localCoords[0];
				  break;
		}
		case 2:
		{
				  return localCoords[1];
				  break;
		}
		default:
		{
				   vrError("n > 2");
				   return -1.0;
				   break;
		}
		}
	}

	MyFloat TriangleElem::lineDeris(const MyVec2ParamSpace localCoords, const MyInt n, const MyInt eta)
	{
		vrASSERT((n>=0)&&(n<=2));
		vrASSERT((eta>=0)&&(eta<=1));
		if (0 == n)
		{
			if (0 == eta)
			{
				return -1.0;
			}
			else
			{
				return -1.0;
			}

		}
		else if (1 == n)
		{
			if (0 == eta)
			{
				return 1.0;
			}
			else
			{
				return 0.0;
			}
		}
		else 
		{
			if (0 == eta)
			{
				return 0.0;
			}
			else
			{
				return 1.0;
			}
		}
	}

	MyFloat TriangleElem::jacobian(const MyVec3& vtxEndPt0, const MyVec3& vtxEndPt1, const MyVec3& vtxEndPt2)
	{
		MyMatrix_2X2 m11, m12, m13;
		const MyFloat x1 = vtxEndPt0[0];
		const MyFloat y1 = vtxEndPt0[1];
		const MyFloat z1 = vtxEndPt0[2];

		const MyFloat x2 = vtxEndPt1[0];
		const MyFloat y2 = vtxEndPt1[1];
		const MyFloat z2 = vtxEndPt1[2];

		const MyFloat x3 = vtxEndPt2[0];
		const MyFloat y3 = vtxEndPt2[1];
		const MyFloat z3 = vtxEndPt2[2];

		m11.coeffRef(0, 0) = (y2-y1);
		m11.coeffRef(0, 1) = (z2-z1);
		m11.coeffRef(1, 0) = (y3-y1);
		m11.coeffRef(1, 1) = (z3-z1);

		m12.coeffRef(0, 0) = (x2 - x1);
		m12.coeffRef(0, 1) = (z2 - z1);
		m12.coeffRef(1, 0) = (x3 - x1);
		m12.coeffRef(1, 1) = (z3 - z1);

		m13.coeffRef(0, 0) = (x2 - x1);
		m13.coeffRef(0, 1) = (y2 - y1);
		m13.coeffRef(1, 0) = (x3 - x1);
		m13.coeffRef(1, 1) = (y3 - y1);

		const MyFloat m11_det = m11.determinant();
		const MyFloat m12_det = m12.determinant();
		const MyFloat m13_det = m13.determinant();

		return std::sqrt(m11_det*m11_det + m12_det*m12_det + m13_det*m13_det);
	}

	void TriangleElem::compute_Shape_Deris_Jacobi()
	{
		//1. compute gauss points : param space [-1, 1]
		
		if (isDiscontinuous())
		{
			MyMatrix_3X3 Q_Discontinuous_tmp;
			const MyFloat l = std::sqrt(2.0) / 2.0;
			const MyFloat l1 = l * 0.25;
			const MyFloat l2 = l * 0.75;
			const MyFloat s1 = l2 / l;
			const MyFloat s2 = l1 / (2 * l);
			MyVec2ParamSpace paramCoordsInDiscontinuous[MyDim] = { MyVec2ParamSpace(s2, s2), MyVec2ParamSpace(s1, s2), MyVec2ParamSpace(s2, s1) };

			for (int gpt = 0; gpt < MyDim; ++gpt)			
			{
				for (int n = 0; n < MyDim; ++n)
				{
					Q_Discontinuous_tmp.coeffRef(gpt, n) = lineShapeFunction(paramCoordsInDiscontinuous[gpt], n);
				}
			}
			
			Q_Discontinuous = Q_Discontinuous_tmp.inverse();

			/*std::cout << std::endl;
			std::cout << Q_Discontinuous * m_elemVtxPts << std::endl;
			std::cout << m_elemEndPts << std::endl << std::endl;			
			vrPause;*/

			MyVec3 gpt0[3];
			for (int g = 0; g < 3;++g)
			{
				gpt0[g] = Q_Discontinuous.coeff(g, 0) * getElemVtxPoint(0) + Q_Discontinuous.coeff(g, 1) * getElemVtxPoint(1) + Q_Discontinuous.coeff(g, 2) * getElemVtxPoint(2);
			}
			
			/*infoLog << "gauss pt : " << gpt0[0].transpose() << " , " << gpt0[1].transpose() << " , " << gpt0[2].transpose() << std::endl;
			infoLog << " element id : " << getID();
			infoLog << "end pt : " << getElemEndPoint(0).transpose() << " , " << getElemEndPoint(1).transpose() << " , " << getElemEndPoint(2).transpose() << std::endl;
			vrPause;*/
			//infoLog << "Discontinuous " << Q_Discontinuous << std::endl;
		}
		else
		{
			Q_Discontinuous = MyMatrix_3X3::Identity();
			vrASSERT(numbers::isEqual(getElemEndPoint(0), getVertex(0)->getPos()));
			vrASSERT(numbers::isEqual(getElemEndPoint(1), getVertex(1)->getPos()));
			//infoLog << "Continuous " << Q_Discontinuous << std::endl;
		}
		//Q_Discontinuous = MyMatrix_2X2::Identity();
		//2. compute shape function N1, N2
		for (int gpt = 0; gpt < MyDim; ++gpt)
		{
			for (int n = 0; n < MyDim; ++n)
			{
				shapeFunctionN.coeffRef(gpt, n) = lineShapeFunction(s_gaussPtInParamSpace[gpt], n);
			}
		}
		shapeFunctionN = shapeFunctionN * Q_Discontinuous;

		//3. compute shape function deris
		/*deris_dN[gauss  point id]:[N0, N1, N2] : [XSI, ETA]*/
		/*eg: deris_dN[2][1][0]: dN1 / dXSI of gauss point 2*/
		//MyMatrix_3X2 deris_dN[Geometry::vertexs_per_tri];
		for (int gpt = 0; gpt < MyDim; ++gpt)
		{
			for (int n = 0; n < MyDim; ++n)
			{
				for (int eta = 0; eta < MyParaDim;++eta)
				{
					deris_dN[gpt].coeffRef(n, eta) = lineDeris(s_gaussPtInParamSpace[gpt],n,eta);
				}
				
			}
		}

		//deris_dN = deris_dN * Q_Discontinuous;
		//4. compute Jacobi


		MyFloat jacobi = jacobian(getElemEndPoint(0), getElemEndPoint(1), getElemEndPoint(2));
		vrASSERT(jacobi > 0.0);

		//printf("jacobi(%f)  length(%f)\n", jacobi, (getElemEndPoint(0) - getElemEndPoint(1)).norm());
		//vrASSERT(numbers::isEqual(jacobi*(s_gaussPtWeigth[0] + s_gaussPtWeigth[1]), (getElemEndPoint(0) - getElemEndPoint(1)).norm()));

		JxW_values[0] = jacobi * s_gaussPtWeigth[0];
		JxW_values[1] = jacobi * s_gaussPtWeigth[1];
		JxW_values[2] = jacobi * s_gaussPtWeigth[2];
	}

	void TriangleElem::integrateHGsubmatrices_GLQ(const MyVec3& collocGlbPt, MyMatrix& Hsubmatrix, MyMatrix& Gsubmatrix)
	{
		const MyInt nGaussPtSize = MyDim;

		Hsubmatrix.resize(MyDim, nGaussPtSize*MyDim); Hsubmatrix.setZero();
		Gsubmatrix.resize(MyDim, nGaussPtSize*MyDim); Gsubmatrix.setZero();

		MyMatrix tmpH(Hsubmatrix);
		MyMatrix tmpG(Gsubmatrix);
		for (int ngpt = 0; ngpt < nGaussPtSize; ++ngpt)
		{
			MyFloat JxW;
			MyVec3 fieldNormals;
			MyFloat r;
			MyVec3 dr;
			MyFloat drdn;
			getKernelParameters(ngpt, collocGlbPt, JxW, fieldNormals, r, dr, drdn);

			MyMatrix_3X3 Ttemp, Utemp;
			Ttemp.setZero();
			Utemp.setZero();

			for (int i = 0; i < nGaussPtSize; ++i)
			{
				for (int j = 0; j < nGaussPtSize; ++j)
				{
					DBIEkernels(i, j, r, dr, drdn, fieldNormals, Ttemp.coeffRef(i, j), Utemp.coeffRef(i, j));
				}
			}

			//std::cout << "Ttemp :" << std::endl << Ttemp << std::endl << "Utemp : " << std::endl << Utemp << std::endl; vrPause;
			const MyVec3& N = shapeFunctionN.row(ngpt).transpose();

			tmpH.block(0, 0, MyDim, MyDim) = N[0] * Ttemp;
			tmpH.block(0, MyDim, MyDim, MyDim) = N[1] * Ttemp;
			tmpH.block(0, MyDim + MyDim, MyDim, MyDim) = N[2] * Ttemp;
			Hsubmatrix = Hsubmatrix + tmpH * JxW;

			tmpG.block(0, 0, MyDim, MyDim) = N[0] * Utemp;
			tmpG.block(0, MyDim, MyDim, MyDim) = N[1] * Utemp;
			tmpG.block(0, MyDim + MyDim, MyDim, MyDim) = N[2] * Utemp;
			Gsubmatrix = Gsubmatrix + tmpG * JxW;
			/*Hsubmatrix = Hsubmatrix + [N(1)*Ttemp N(2)*Ttemp N(3)*Ttemp] * jacob*gwt(pt);
			Gsubmatrix = Gsubmatrix + [N(1)*Utemp N(2)*Utemp N(3)*Utemp] * jacob*gwt(pt);*/
		}
	}
	const MyFloat my_pi = boost::math::constants::pi<MyFloat>();
	const MyFloat E = 1e5;
	const MyFloat mu = 0.3;
	const MyFloat shearMod = E / (2 * (1 + mu));
	
	const MyFloat const1_3d = 16 * my_pi*shearMod * (1 - mu);
	const MyFloat const2_3d = (3 - 4 * mu);
	const MyFloat const3_3d = (8 * my_pi*(1 - mu));
	const MyFloat const4_3d = (1 - 2 * mu);

	void TriangleElem::DBIEkernels(const MyInt i, const MyInt j, const MyFloat r, const MyVec3& dr, const MyFloat& drdn, const MyVec3& normals, MyFloat& Tij, MyFloat& Uij)
	{
		MyFloat i_equal_j;
		if (i == j)
		{
			i_equal_j = 1.0;
		} 
		else
		{
			i_equal_j = 0.0;
		}

		Uij = (1.0 / (16 * my_pi*shearMod * (1 - mu)*r))* 
			  (
			      ((3 - 4 * mu)*i_equal_j) 
				  + 
				  (dr(i)*dr(j))
			  );
#if 0
		Tij = (-1.0 / ((8 * my_pi*(1 - mu)))*(r*r)) *
			(
			drdn*((1 - 2 * mu)*i_equal_j + 3 * (dr(i)*dr(j)))
			-
			(1 - 2 * mu)*(dr(i)*normals(j) - dr(j)*normals(i))
			);
#else
		Tij = (-1.0 / ((8 * my_pi*(1 - mu))*r*r)) *
			(
			drdn*((1 - 2 * mu)*i_equal_j + 3 * (dr(i)*dr(j)))
			-
			(1 - 2 * mu)*(dr(i)*normals(j) - dr(j)*normals(i))
			);
#endif
		

	}

	TriangleElem::TriangleElem(const std::vector< VertexPtr >& vertexes)
	{
		m_TriElemType = Continuous;// default Line Element Type
		vrASSERT(vertexes.size() == Geometry::vertexs_per_tri);
		m_elem_vertex[0] = vertexes[0];
		m_elem_vertex[1] = vertexes[1];
		m_elem_vertex[2] = vertexes[2];
	}

	TriangleElem::~TriangleElem()
	{}

	void TriangleElem::get_dof_indices(MyVector9I &vecDofs)
	{
		for (int v = 0; v < MyDim;++v)
		{
			vecDofs.block(v*MyDim,0,MyDim,1) = getVertex(v)->getDofs();
		}
	}

	void TriangleElem::getKernelParameters(const MyInt gpt/*0,1,2*/, const MyVec3& collectPt, MyFloat& JxW, MyVec3& fieldNormals, MyFloat& r, MyVec3& dr, MyFloat& drdn)
	{
		
		MyVec3 fieldPt = shapeFunctionN.coeff(gpt, 0)*(getElemVtxPoint(0)) +
			shapeFunctionN.coeff(gpt, 1)*(getElemVtxPoint(1)) + shapeFunctionN.coeff(gpt, 2)*(getElemVtxPoint(2));

		JxW = JxW_values[gpt];
		fieldNormals = m_TriElemNormal;

		MyVec3 relDist = fieldPt - collectPt;
		r = relDist.norm();
		
		//vrASSERT(r > 0.01f);
		dr = (1.0 / r) * relDist;

		drdn = dr.transpose() * fieldNormals;
	}
	
	void TriangleElem::calculateJumpTerm_smooth(MyMatrix_3X3& jumpTerm)
	{
		jumpTerm = MyMatrix_3X3::Identity();
		jumpTerm *= 0.5;
	}
}//namespace VR