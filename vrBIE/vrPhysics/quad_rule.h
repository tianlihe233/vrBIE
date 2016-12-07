#ifndef _quad_rule_h_
#define _quad_rule_h_

#include "bemDefines.h"

//param space variable (XSI,ETA)
namespace VR
{
	//maps reference points to physical points.
	void reference_to_physical_t3(const MyVec3& gloablCoords_0, const MyVec3& gloablCoords_1, const MyVec3& gloablCoords_2,
		const MyVec2ParamSpace& referenceCoords, MyVec3& phyCoords)
	{
		MyFloat shapeFunction[MyDim];
		shapeFunction[0] = 1.0 - referenceCoords[0] - referenceCoords[1];
		shapeFunction[1] = referenceCoords[0];
		shapeFunction[2] = referenceCoords[1];

		phyCoords = shapeFunction[0] * gloablCoords_0 + shapeFunction[1] * gloablCoords_1 + shapeFunction[2] * gloablCoords_2;
		//for (int i = 0; i < 2; i++)
		//{
		//	for (int j = 0; j < MyDim; j++)
		//	{
		//		/*phyCoords.coeffRef();
		//		phy[i + j * 2] = t[i + 0 * 2] * (1.0 - ref[0 + j * 2] - ref[1 + j * 2])
		//			+ t[i + 1 * 2] * +ref[0 + j * 2]
		//			+ t[i + 2 * 2] * +ref[1 + j * 2];*/
		//	}
		//}
	}
	//quad_order : 
	// 0 : (¦Î1, ¦Ç1) = (1/6, 1/6) , (¦Î2, ¦Ç2) = (2/3 , 1/6) , (¦Î3, ¦Ç3) = (1/6 , 2/3) , w1 = w2 = w3 = 1/3
	// 1 : (¦Î1, ¦Ç1) = (0, 1/2)   , (¦Î2, ¦Ç2) = (1/2 , 0)   , (¦Î3, ¦Ç3) = (1/2 , 1/2) , w1 = w2 = w3 = 1/3
	void quad_rule(const MyInt quad_order, MyDenseVector& quad_w, MyCoords_3X2& quad_xy, MyCoords_3X2& local_Coords)
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
		local_Coords.row(0) = MyVec2ParamSpace(0.0,0.0);
		local_Coords.row(1) = MyVec2ParamSpace(1.0, 0.0);
		local_Coords.row(2) = MyVec2ParamSpace(0.0, 1.0);
		if (0 == quad_order)
		{
			const MyFloat const_1_6 = 1.0 / 6.0;
			const MyFloat const_2_3 = 2.0 / 3.0;
			const MyFloat const_w = 1.0 / 3.0;

			quad_w[0] = quad_w[1] = quad_w[2] = const_w;
			quad_xy.row(0) = MyVec2ParamSpace(const_1_6, const_1_6);
			quad_xy.row(1) = MyVec2ParamSpace(const_2_3, const_1_6);
			quad_xy.row(2) = MyVec2ParamSpace(const_1_6, const_2_3);
		}
		else if (1 == quad_order)
		{
			MyError("unknow quad_order value!");
			const MyFloat const_1_2 = 0.5;
			const MyFloat const_0 = 0.0;
			const MyFloat const_w = 1.0 / 3.0;
			quad_w[0] = quad_w[1] = quad_w[2] = const_w;
			quad_xy.row(0) = MyVec2ParamSpace(const_0, const_1_2);
			quad_xy.row(1) = MyVec2ParamSpace(const_1_2, const_0);
			quad_xy.row(2) = MyVec2ParamSpace(const_1_2, const_1_2);
		}
		else
		{
			MyError("unknow quad_order value!");
		}
	}
}
#endif//_quad_rule_h_