#ifndef _bemMarco_h_
#define _bemMarco_h_

#define DoublePrecision (1)
#define MyError(X)do{printf("%s\n",#X);exit(66);}while(false)
#define MyPause system("pause")
#define MyExit exit(66)
#define Q_ASSERT(X) do{if (!(X)){/*printf("%s\n",#X);*/system("pause");}}while(false)
#define MyArrayBase (1)
#define MyTest (1)
#define SHOWMATRIX(mat) std::cout << #mat << " : " << std::endl << mat << std::endl;
#define SHOWVECTOR(vec) std::cout << #vec << " : " << std::endl << vec.transpose() << std::endl;
#define SHOWVAR(var) std::cout << #var << " : " << std::endl << var << std::endl;

#define SHOWPLOT (0)
#define MyDim (3)
#define MyParaDim (2)

#define Hole_Sample (0)
#define Lplate_Sample (0)
#define Spanner_Sample (0)
#define uniaxial_tension_Sample (1)

#if DoublePrecision
#define MyEPS DBL_EPSILON
#else
#define  MyEPS FLT_EPSILON
#endif

#define MyNotice
#define USE_TBB (1)

#define USE_Mantic_CMat (1)
#endif//_bemMarco_h_