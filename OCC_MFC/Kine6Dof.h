#ifdef Dll_PPDll
#else
#define Dll_PPDll _declspec(dllimport)
#endif

//#include <stdio.h>
//#include <afx.h>

#define Ni 3000

Dll_PPDll double Joints[Ni][6];  //规划路径关节角输出
Dll_PPDll double JointNode[6]; //当前6个关节角
Dll_PPDll unsigned int Status; //输出状态标识 
Dll_PPDll double d1, a1, a2, a3, d4, d6, d2, y2;  // 6个关键尺寸
Dll_PPDll int JntCoe[6];  // 6个关节角方向  JntCoe: 关节角方向（以ABB为标准，1表示同向，-1表示反向）
Dll_PPDll double PEtool[6];   // 对应于Twhq   

///***********函数体声明****************//
//////////////一、调用的功能模块函数
//Dll_PPDll unsigned char StraightLineMotion(double NowNode[6], double TargetNode[6], double Obstacle[6], double VelocityLimit[6], double Acc, unsigned int ExpectedTime, 
//	       			                                   unsigned char FirstFlag); //开环直线运动规划函数
Dll_PPDll unsigned char StraightLineMotion(double NowNode[6], double TargetNode[6], double Acc, unsigned int ExpectedTime, unsigned char FirstFlag, bool tool, double JointsNode[6]);

Dll_PPDll unsigned char ArcMotion(double NowNode[6], double MiddleNode[6], double TargetNode[6], double Obstacle[6], double VelocityLimit[6],double Acc, unsigned int ExpectedTime, 
											  unsigned char FirstFlag); //开环圆弧运动规划函数
Dll_PPDll unsigned char ConstainedCurveMotion(double NowNode[6], double MiddleNode1[6], double MiddleNode2[6], double MiddleNode3[6], double TargetNode[6], double Obstacle[6], double VelocityLimit[6], double Acc, unsigned int ExpectedTime, 
														  unsigned char FirstFlag); //开环约束曲线规划函数
Dll_PPDll unsigned char EndTipMotion(double NowNode[6], unsigned char EndTipStatus[3], double Obstacle[6], 
												 double VelocityLimit[6]); //操作员手动操作规划函数
Dll_PPDll unsigned char Collision(double NowNode[6], double Obstacle[6]); //碰撞检测函数

////////////二、运动学正逆解函数
Dll_PPDll void forwardkine_6Dof( double Joint[6], double T06[4][4]);
Dll_PPDll void forwardkine_rad( double Joint[6], unsigned char tool, double Tst[4][4]); //运动学正解
Dll_PPDll void forwardkine_ang( double Joint[6], unsigned char tool, double Tst[4][4]);  //运动学正解

Dll_PPDll void backwardkine_6Dof( double T06[4][4], double zceta[8][6]); //运动学反解(8组解)
Dll_PPDll void backwardkine_6DofChoose( double T06[4][4], unsigned char Index, double cceta[6]);  //运动学反解(选取第Index组解输出)


//在vtk环境下采用旋量法创建机器人模型16.6.12
Dll_PPDll void vtkforwardkine_ang(double Joint[6], double PEM[6][6], double PETool0[6], double PETool1[6]);  
 

//在vtk环境下采用旋量法创建可参数修改的机器人运动学模型16.7.24
Dll_PPDll void vtkparmforwardkine_ang(double Joint[6], unsigned int flag,  double PEM[6][6], double PETool0[6], double PETool1[6]);  
Dll_PPDll void parmforwardkine_ang( double Joint[6], unsigned char tool, double Tst[4][4]);  //带参数的运动学正解
Dll_PPDll void vtkparmpositionerkine_ang(double qper, double PE[6]);  //变位机正解
//2021.11.19 川崎
Dll_PPDll void vtkparmforwardkine_ang_Kawasaki(double Joint[6], unsigned int flag,  double PEM[6][6], double PETool0[6], double PETool1[6]);
Dll_PPDll void parmforwardkine_ang_Kawasaki( double Joint[6], unsigned char tool, double Tst[4][4]);  //带参数的运动学正解

////////////三、子功能模块函数
///1:圆弧规划子函数
Dll_PPDll double SolveAngle(double p1[2], double p2[2], double p3[2], double Rad); //计算圆弧对应的圆心角大小,用于圆弧运动规划函数内部调用
Dll_PPDll double SolveAngle2D(double p1[2], double p2[2], double Rad); 
Dll_PPDll void TriAngleCharacter(double p1[2], double p2[2], double p3[2], double p0[2], unsigned int Character[3]); //确定圆弧转向,用于圆弧运动规划函数内部调用
Dll_PPDll double space_circle(double pp[3][3], double p0[3], double R[3][3]); //确定空间圆弧圆心、半径及圆弧所在平面
///2:约束曲线规划子函数
Dll_PPDll void InterPolnomial(double ti[4], double cord[5], double coe[7]); //6次多项式插值函数,用于开环约束曲线规划函数内部调用
///3:操作员手动规划子函数
Dll_PPDll unsigned char EndTipTransMotion(double NowNode[6], unsigned int MissionNum, int MotionCoe, double aveVel, double aveAcc, double Obstacle[6], 
													  double VelocityLimit[6]); //操作员手动平移操作规划, 用于操作员手动操作规划函数内部调用
Dll_PPDll unsigned char EndTipRotMotion(double NowNode[6], unsigned int MissionNum, int MotionCoe, double aveW, double aveAif, double Obstacle[6], 
													double VelocityLimit[6]);   //操作员手动旋转操作规划, 用于操作员手动操作规划函数内部调用
Dll_PPDll void EndTipStatusDetection(unsigned char EndTipStatus[3], unsigned char St[3]); //操作员手动操作状态检测, 用于操作员手动操作规划函数内部调用
///4:碰撞检测子函数
Dll_PPDll void Self_Collision(double NowNode[6], unsigned int* CollisionStatus); //机器人自身碰撞检测
Dll_PPDll void Capsule_Collision(double NowNode[6], unsigned int CapsIndex, unsigned int* CollisionFlag);
Dll_PPDll void CollisionKeyPoint(double NowNode[6], double KeyPoint[7][3]); //求解碰撞检测关键点坐标,用于碰撞检测调用
Dll_PPDll void UniplanarLinesDist(double Point[4][3], double Pedal[2][3], double* dis, double *ang); //计算异面直线距离、垂足和夹角,用于碰撞检测调用
Dll_PPDll double DotLineDist(double PL[2][3], double PW[3]); //计算空间点到直线距离,用于碰撞检测调用
Dll_PPDll double TwoLineSegmentMinDist(double Point[4][3]); //空间中两线段上点的最短距离
Dll_PPDll unsigned char Obstacle_Collision(double NowNode[6], double Obstacle[6]); //障碍物碰撞检测函数
Dll_PPDll unsigned char Object_Collision(double NowNodeIn[6], double Object[6]); //目标碰撞检测函数

//Dll_PPDll void RbtSelf_Collision(double KeyPoint[5][3], double Lr[3], unsigned int* CollisionStatus, unsigned int CollisionLinkIndex[2], double *dist0, double *dist1);  ////机器人自身碰撞检测  2016.7.26
Dll_PPDll void RbtSelf_Collision(double KeyPoint[5][3], double Lr[3], unsigned int* CollisionStatus, unsigned int CollisionLinkIndex[2],double *dist0, double *dist1, double tt[2], double KP1[3], double KP2[3]);  ////机器人自身碰撞检测
Dll_PPDll double TwoLineSegmentMinDist2(double Point[4][3], double tt[2], double KeyP1[3], double KeyP2[3]);

Dll_PPDll void TwoSpaceLineIntersection(double P1[3], double P2[3], double n1[3], double n2[3], double Pintsec[3]); //两空间直线求交点
Dll_PPDll void GetFootOfPerpendicular(double pt[3], double begin[3], double end[3], double retVal[3]);  // 三维空间点到直线的垂足。
///2017.4.8根据两平面相交四垂足点确定实际焊缝两点，鼠标点选两平面确定实际直线焊缝专用函数
Dll_PPDll void FootOrderInnerPoints(double FootPnts1[3], double FootPnts2[3], double FootPnts3[3], double FootPnts4[3], int begin_endchange, double BeginPnts[3], double EndPnts[3]);  
Dll_PPDll double TwoPntDist(double a[3], double b[3]);  //两点之间的距离
Dll_PPDll double TwoPntDist2D(double a[2], double b[2]);  //平面两点之间的距离


///////////四、公共模块函数
///1:机器人位姿转换基本函数
Dll_PPDll void RbtTr2EulerZyx(double dTr[3][3],double dEulerZyx[3]); //3X3姿态矩阵转Z-Y-X欧拉角(弧度,下同)
Dll_PPDll void euler_Rzyx( double oula[3],double a[3][3]); //Z-Y-X欧拉角转3X3姿态矩阵
Dll_PPDll void RbtTr2EulerXyz(double dTr[3][3], double dEulerXyz[3]);  //3X3姿态矩阵转X-Y-Z欧拉角(弧度,下同)  2017.1.17
Dll_PPDll void euler_Rxyz( double oula[3],double a[3][3]); //X-Y-Z欧拉角转3X3姿态矩阵
Dll_PPDll void Eulxyz_Eulzyx(double Xyz[3], double Zyx[3]); //X-Y-Z欧拉角转Z-Y-X欧拉角 (角度)
Dll_PPDll void Eulzyx_Eulxyz(double Zyx[3], double Xyz[3]); //Z-Y-X欧拉角转X-Y-Z欧拉角 (角度)  


Dll_PPDll void T2PE(double T[4][4], double PE[6]); //4X4齐次矩阵转6维位姿坐标(前3个是位置坐标,后3个是Z-Y-X欧拉角姿态坐标)
Dll_PPDll void T2PE2(double T[4][4], double P[3], double E[3]);  // T2PE函数的P、E分开形式
Dll_PPDll void T2PExyz(double T[4][4], double PE[6]);  //4X4齐次矩阵转6维位姿坐标(前3个是位置坐标,后3个是X-Y-Z欧拉角姿态坐标)

Dll_PPDll void T2PR(double T[4][4], double P[3], double R[3][3]); //T转P,E
Dll_PPDll void PE2T(double PE[6], double T[4][4]); //6维位姿坐标转4X4齐次矩阵
Dll_PPDll void PE2T2(double P[3], double E[3], double T[4][4]); 
Dll_PPDll void RP2T(double P[3], double R[3][3], double T[4][4]); //已知位置向量和姿态矩阵,转4X4齐次矩阵e 
///2:基本矩阵运算函数
Dll_PPDll void RbtMulMtrx( int m, int n, int p, double *A, double *B, double *C ); //矩阵相乘
Dll_PPDll int RbtInvMtrx( double *C, double *IC, int n ); //矩阵求逆
Dll_PPDll double det(double a[3][3]); //矩阵行列式计算
Dll_PPDll double dotm(double a[3], double b[3]); //向量点积
Dll_PPDll double dotm2D(double a[2], double b[2]);  // 向量点积
Dll_PPDll void crossm( double a[],double b[], double c[] ); //向量叉积
///3:基本代数运算函数
Dll_PPDll double dou_abs(double a); //绝对值计算
Dll_PPDll int sign(double a); //符号函数
Dll_PPDll double ffmin(double a, double b); //求两个值中的最大值
Dll_PPDll double ffmax(double a, double b); //求两个值中的最小值
Dll_PPDll double FindArrayMin(double* Array, int ArraySize);   //求一个数组中的最小值
Dll_PPDll double FindArrayMax(double* Array, int ArraySize);   //求一个数组中的最大值
Dll_PPDll int FindArrayMin_INT(int* Array, int ArraySize);   //求一个数组(int)中的最小值
Dll_PPDll int FindArrayMax_INT(int* Array, int ArraySize);  //求一个数组(int)中的最大值

Dll_PPDll void FindArrayMinandMax(double* Array,int ArraySize, double *Min, double *Max);  //求一个数组中的最大和最小值
Dll_PPDll double nnorm1(double a, double b, double c); //求模函数1
Dll_PPDll double nnorm2(double a[3]);  //求模函数2
Dll_PPDll double nnorm2_2D(double a[2]);  //求模函数2 2D版本
Dll_PPDll double norm(double a[3], double b[3]);  //求模函数3
///4:运动学逆解内部调用函数
Dll_PPDll unsigned char ChooseSolution(double zceta[8][6], double jointangle[6]); //运动学逆解选解,返回选出解的序列
Dll_PPDll double ntrt3(double re, double im); //运动学逆解函数内部调用
Dll_PPDll int choose(int index); //运动学逆解函数内部调用
Dll_PPDll int choose2(int index); //运动学逆解函数内部调用

Dll_PPDll void ObjEndTip2Globle(double NowNode[6], double EndTipPos[6], double GloblePos[6]); //目标相对于末端位姿转换为目标相对于基系位姿
Dll_PPDll void ObjGloble2EndTip(double NowNode[6], double GloblePos[6], double EndTipPos[6]); //目标相对于基系位姿转换为目标相对于末端位姿
Dll_PPDll void singlejointmatic(double NowNode[6], int index, double vel); //单关节运动函数



//旋量部分
Dll_PPDll void Epx(double w[3], double p[3], double Repx[6]);      //生成6维旋量坐标
Dll_PPDll void Rod(double w[3], double ceta, double Ro[3][3]);     //Rodrigues公式  e^(w*ceta)  SO(3)
Dll_PPDll void Asymtx(double w[3], double wR[3][3]);               //向量w生成的反对称矩阵w^
Dll_PPDll void wwT(double w[3], double Rw[3][3]);               
Dll_PPDll void Rvp(double w[3], double v[3], double ceta, double vp[3]);
Dll_PPDll void TwiMtx(double yp[6], double ceta, double T[4][4]);   //SE(3)  e^(yp*ceta)

Dll_PPDll void SE3toAdg(double g[4][4], double Adg[6][6]);   // Ad: SE(3)的伴随映射关系
//Dll_PPDll void Adi_Multip(int n, double Adn[6][6]);   // 求Adi  运动学标定函数



///运动学反解
Dll_PPDll void solveceta23(double qw2[2][3], double ceta2[4], double ceta3[4]);
Dll_PPDll void solveceta45(double qm[3], double mg1[4][4], double ceta4[2], double ceta5[2]);
Dll_PPDll void solveceta6(double qm[5], double mg1[4][4], double* ceta6);

Dll_PPDll void backwardkine_tool0( double Tst[4][4], unsigned char r_a, double zceta[8][6]);
Dll_PPDll void backwardkine_hq( double Thq[4][4], unsigned char r_a, double zceta[8][6]);
Dll_PPDll void backwardkine_tool0Choose( double Tst[4][4], unsigned char Index, unsigned char r_a, double cceta[6]);
Dll_PPDll void backwardkine_hqChoose( double Thq[4][4], unsigned char Index, unsigned char r_a,  double cceta[6]);

Dll_PPDll void backwardkine_tool0_Kawasaki( double Tst[4][4], unsigned char r_a, double zceta[8][6]);
Dll_PPDll void backwardkine_hq_Kawasaki( double Thq[4][4], unsigned char r_a, double zceta[8][6]);
Dll_PPDll void backwardkine_tool0Choose_Kawasaki( double Tst[4][4], unsigned char Index, unsigned char r_a, double cceta[6]);
Dll_PPDll void backwardkine_hqChoose_Kawasaki( double Thq[4][4], unsigned char Index, unsigned char r_a,  double cceta[6]);


///参数化运动学反解
/*
Dll_PPDll void parmsolveceta23(double qw2[2][3], double LL[6], double ceta2[4], double ceta3[4]);
Dll_PPDll void parmsolveceta45(double qm[3], double LL[6], double mg1[4][4], double ceta4[2], double ceta5[2]);
Dll_PPDll void parmsolveceta6(double qm[5], double LL[6], double mg1[4][4], double* ceta6);

Dll_PPDll void parmbackwardkine_tool0( double Tst[4][4], double LL[6], unsigned char r_a, double zceta[8][6]);
Dll_PPDll void parmbackwardkine_hq( double Thq[4][4], double LL[6], unsigned char r_a, double zceta[8][6]);
Dll_PPDll void parmbackwardkine_tool0Choose( double Tst[4][4], double LL[6], unsigned char Index, unsigned char r_a, double cceta[6]);
Dll_PPDll void parmbackwardkine_hqChoose( double Thq[4][4], double LL[6], unsigned char Index, unsigned char r_a, double cceta[6]);
*/

Dll_PPDll void Euler2Qua(double Euler[3], double Qua[4]);  //Euler角(角度)转四元数
Dll_PPDll void Qua2Euler(double Qua[4], double Euler[3]);  //四元数转Euler角(角度)
Dll_PPDll void Qua2RotMtx(double Qua[4], double R[3][3]);   //四元数转旋转矩阵R
Dll_PPDll void RotMtx2Kceta(double R[3][3], double K[3], double* ceta);  //旋转矩阵转旋转轴K和ceta
Dll_PPDll void Kceta2RotMtx(double K[3], double ceta, double R[3][3]);  // 旋转轴Kceta转旋转矩阵R
Dll_PPDll void Kceta2Qua(double K[3], double ceta, double Qua[4]);  // 旋转轴转四元数
Dll_PPDll void Qua2Kceta(double Qua[4], double K[3], double* ceta);  // 四元数转旋转轴
Dll_PPDll void RotMtx2Qua(double R[3][3], double Qua[4]);   //旋转矩阵转四元数


//Dll_PPDll void T2Qua(double T[4][4], double Qua[4]);

///四元数的相关运算（参考：3D数学基础第11章）
Dll_PPDll void QuaExtV(double Qua[4], double V[3]);  //提取四元数虚部向量
Dll_PPDll void QuaNormalize(double Qua[4], double QuaNormal[4]);   //正则化四元数
Dll_PPDll double QuaDotProduct(double Qua1[4], double Qua2[4]);   //四元数的点乘
Dll_PPDll void QuaSlerp(double Qua0[4], double Qua1[4], double t, double Quat[4]); // 四元数的球面线性插值
Dll_PPDll void ArrayEquation(double* a, double* b, int n);   //两数组相等, a为原数组，n为数组大小，目标数组b=a
Dll_PPDll void QuaConjugate(double Qua[4], double QuaConj[4]);  //四元数共轭
Dll_PPDll void QuaPow(double Qua[4], double exponent, double Quat[4]); //四元数的幂
Dll_PPDll double QuaNorm(double Qua[4]);  //四元数的模
Dll_PPDll void QuaInverse(double Qua[4], double QuaInv[4]); //四元数的逆
Dll_PPDll void QuaEqu(double Qua1[4], double Qua2[4]);   //四元数相等  Qua2=Qua1
Dll_PPDll void QuaScalarMultip(double Qua[4], double QuaSM[4], double Scalar); //四元数的数乘
Dll_PPDll void QuaMultipation(double Qua1[4], double Qua2[4], double QuaMul[4]);  //四元数的乘法
Dll_PPDll void QuaDifference(double Qua1[4], double Qua2[4], double QuaDif[4]);  //四元数的差
Dll_PPDll void QuaAdd(double Qua1[4], double Qua2[4], double QuaA[4]);  //四元数加法
Dll_PPDll void DualQuaMake(double Q1[4], double Q2[4], double DQ[8]);    //两个四元数构造对偶四元数
Dll_PPDll void DualQuaExt(double DQ[8], double Q1[4], double Q2[4]);    //对偶四元数抽取两个四元数
Dll_PPDll void DualQuaMul(double DQ1[8], double DQ2[8], double DQm[8]);  //对偶四元数相乘
Dll_PPDll void DualQuaConj(double DQa[8], double DQb[8]);  //对偶四元数的共轭
Dll_PPDll void Kdceta2DualQua(double d[3], double K[3], double ceta, double DQ[8]);  //由平移d和旋转轴Kceta生成对偶四元数DQ
Dll_PPDll void DualQua2QrQp(double DQ[8], double Qr[4], double Qp[4]);  //将对偶四元数转化为旋转四元数Qr和平移四元数Qp 

Dll_PPDll void Hq2RbtE(double PE[6]);  ///焊枪工具坐标系相对于机器人tool0系的PE

Dll_PPDll void GesRelocationXYZMotion(double NowNode[6], double TargetGes[3], unsigned int totalNum, unsigned int No, double JointsNode[6]);   //姿态重定位
Dll_PPDll void GesRelocationXYZMotion2(double NowNode[6], double TargetGes[3], unsigned int totalNum, unsigned int No, double JointsNode[6]);   //姿态重定位(四元数实现)
Dll_PPDll unsigned char StraightLineMotion2(double NowNode[6], double TargetNode[6], double Acc, unsigned int ExpectedTime, unsigned char FirstFlag, bool tool, double JointsNode[6]);  //直线（姿态四元数）
Dll_PPDll unsigned char EndTipRotMotion2(double NowNode[6], unsigned int MissionNum, int MotionCoe, double Ceta, unsigned int totalNum, unsigned int No, double JointsNode[6]); //末端姿态旋转
Dll_PPDll unsigned char EndTipTransMotion2(double NowNode[6], unsigned int MissionNum, int MotionCoe, double S, unsigned int totalNum, unsigned int No, double JointsNode[6]);  //末端线性
Dll_PPDll void EndTipTransMotion3(double NowNode[6], unsigned int MissionNum, int MotionCoe, double aveVel, unsigned char coordindex, double JointsNode[6]);
Dll_PPDll void EndTipRotMotion3(double NowNode[6], unsigned int MissionNum, int MotionCoe, double aveW, unsigned char coordindex, double JointsNode[6]);
Dll_PPDll void EndTipMotion3(double NowNode[6], unsigned char EndTipStatus[3], unsigned char coordindex, double JointsNode[6]);

Dll_PPDll void JointSpaceMotion(double NowNode[6], double TargetNode[6], int NN);  //关节空间运动规划


//焊缝曲线离散化坐标系建立相关函数
Dll_PPDll void SpaceLineGesSetting(double n1[3], double n2[3], double Ps[3], double Pe[3], int hqcoe[2], double R[3][3]); //空间直线焊缝曲线坐标系（提供两空间平面）
Dll_PPDll void P3_plane2(double pp[3][3], double z[3]); 
Dll_PPDll void P3_planeN(double p1[3], double p2[3], double p3[3], double z[3]);
Dll_PPDll void P3_planeCoe(double p1[3], double p2[3], double p3[3], double Coe[4]); // 由三点确定所在平面方程系数
Dll_PPDll void SpaceLineMeldRectCoord(double P1[3], double P2[3], double P3[3], int zcoe, double P[3], double R[3][3]);  
Dll_PPDll void DrawMeldRect(double Pm[3], double Rm[3][3], double RectCC[3], double Po[3], double Euler[3]);
Dll_PPDll int planeNmake(double p0[3], double p1[3], double p2[3], double n1[3], double n2[3]);

Dll_PPDll double MA_CetaPlanning(double V0, double T, double R, double r, double nowceta);
Dll_PPDll void MA_PtPlanning(double nowceta, double R, double r, double Pt[3]);
Dll_PPDll void MA_PbPlanning(double Tb[4][4], double Pt[3], double Pb[3]);
Dll_PPDll void MA_GesbPlanning(double nowceta, double R, double r, double Rb[3][3]);



///16.1.27  斜交偏移管模型
Dll_PPDll void XiJiaoPianGuanlisan(double ceta, double da[4], double T[4][4]);  //斜交偏移管弗-雪矢量离散坐标系
Dll_PPDll void XiJiaoPianGuanlisan2f(double ceta, double da[4], double T[4][4]);  //斜交偏移管主法面二分法
Dll_PPDll void QiuGuanlisan2f(double ceta, double da[3], double T[4][4]);  //球管主法面二分法
Dll_PPDll void QiuGuanlisanFX(double ceta, double da[3], double T[4][4]);  //球管弗-雪矢量离散坐标系
Dll_PPDll void RotZ(double ceta, double T[4][4]); 
Dll_PPDll void RotX(double ceta, double T[4][4]); 
Dll_PPDll void RotY(double ceta, double T[4][4]); 
Dll_PPDll void TransXYZ(double x, double y, double z, double T[4][4]);

//16.2.2  加移动导轨7dofABB1410机器人运动学相关函数
Dll_PPDll void ABB1410_DHMatrix7Dof(double qm[7], double DHM[7][4]);  //1410机器人带导轨 7dof DH参数表
Dll_PPDll void DH2T(double DH[4], double T[4][4]);
Dll_PPDll void Kinehq(double qm[7], double T07[4][4]);  //7dof 正解
Dll_PPDll void EquMtx(double T1[4][4], double T2[4][4]);  //两个矩阵全等 T2=T1
Dll_PPDll void Jacobhq(double qm[7], double J[6][7]);  //带焊枪重定位7dof的雅克比矩阵
Dll_PPDll void InvKinehqZXFS(double qm[7], double dp[6], double dq[7]);  //最小范数逆解
Dll_PPDll void InvKinehqWLN(double qm[7], double dp[6], double dq[7]);  //WLN逆解
Dll_PPDll void pinv(double J[6][7], double iJ[7][6]);   //伪逆
Dll_PPDll void QiuGuanPlanning(int method, double ceta, double Tbwp[4][4], double q[7], double dq[7]);   //球管规划
Dll_PPDll void XiJiaoPianGuanPlanning(int method, double ceta, double Tbwp[4][4], double q[7], double dq[7]);   //斜交偏置管规划7dof


Dll_PPDll void MAGuanlisan2f(double ceta, double r, double R, double Ts[4][4]);  //MA焊缝相对工件的离散化坐标系

//Dll_PPDll void PositionerScrKine(double Joint[2], double d0, double d2, double Tst[4][4]);  //2R变位机Scr运动学正解
Dll_PPDll void PositionerKine(double Joint[2], double d0, double d2, double Twp[4][4]); //2R变位机运动学正解解析式
Dll_PPDll void InvPositionerKine(double Rgh[3][3], double Joint[2]);  //变位机逆解  
Dll_PPDll void InvPositionerQiuGuanKine(double Rgh[3][3], double Joint[2]);  //QiuGuan变位机逆解 
Dll_PPDll void Thfhq(double alpha, double beta, double T[4][4]);  //// 焊枪工艺角度建模：焊枪工艺角度是指焊枪相对于焊缝特征坐标系的位姿

Dll_PPDll void InvRobotPositionerMAKine(double Tgh[4][4], double PositionerParm[2], double Tpg[4][4], double WeldgunJoint[2], double Twb[4][4], double Joints[8], double PE[6]);  //针对MA的变位机和机器人联动逆解问题
Dll_PPDll void InvRobotPositionerQiuGuanKine(double Tgh[4][4], double PositionerParm[2], double Tpg[4][4], double WeldgunJoint[2], double Twb[4][4], double Joints[8], double PE[6]);  //针对QiuGuan的变位机和机器人联动逆解问题

//Dll_PPDll void InvRobotPositionerMAKineTest(double ceta, double Joints[8]); //针对MA的变位机和机器人联动逆解问题仿真测试
/// 圆弧规划函数
Dll_PPDll double circ(double *x, double *y, int Num, double O[2]);  //平面圆拟合
Dll_PPDll void CrossPt_LineSegmentCirc(double P1[2], double P2[2], double Po[2], double Ro, double Pc[2]);  // 求平面内直线段P1P2和圆(Po,Ro)的交点Pc

Dll_PPDll void PtoP(double T[4][4], double P1[3], double P2[3]);  // 点P的坐标变换 P2=T*P1;
Dll_PPDll void EtoE(double T[4][4], double E1[3], double E2[3]);   //E的坐标变换
Dll_PPDll void RtoR(double T[4][4], double R1[3][3], double R2[3][3]);  // R的坐标变换  R2=T*R1; 
Dll_PPDll void VectoVec(double T[4][4], double Vec1[3], double Vec2[3]);  //Vec的坐标变换

//碰撞检测
Dll_PPDll unsigned char LineSegment_TriAnglePlane_CrossJudge(double pp[2][3], double PlaneCoe[4]);  // 判断线段与三角形所在平面是否相交
Dll_PPDll unsigned char LineSegment_TriAngle_CrossJudge(double SegPP[2][3], double TriP0[3], double TriP1[3], double TriP2[3]);  // 判断线段与三角形是否相交
Dll_PPDll unsigned char HqWorkPiece_CollisionDetection1(double Sgo[3], double Sgr, double Shqo[3], double Shqr);  //焊枪与工件的碰撞检测第1步判断

//Dll_PPDll void SolveTwoSpaceLineCrossPnt(double P11[3], double P12[3], double P21[3], double P22[3], double CPnt[3]);  //求空间两直线的交点
Dll_PPDll void SolveTwoSpaceLineCrossPnt(double P11[3], double P12[3], double P21[3], double P22[3], double FaceNormalVec[3], double CPt[3], double Normal[3], double CPtNormal[3], double Xvec[3]);  ////求空间两直线的交点、法向量、法向上点

Dll_PPDll void BSplineInterpolation(int n, double* X, double** Bd1, double** Bd2);     //均匀三次B样条曲线插值
Dll_PPDll void BSplineInterpolation3D(int n, double* X, double** Bd1, double** Bd2, double** Bd3);     //均匀三次B样条曲线插值(2D的情况)


Dll_PPDll void TwoPntVec(double Pt1[3], double Pt2[3], double Vec[3]);  //由两点确定的单位矢量 
Dll_PPDll void VecXYZtoR(double VecX[3], double VecY[3], double VecZ[3], double R[3][3]);  //矢量X、Y、Z构成R
Dll_PPDll void RtoVecXYZ(double R[3][3], double VecX[3], double VecY[3], double VecZ[3]);  //R分解为矢量X、Y和Z
Dll_PPDll void DataInterchange(double *a, double *b, int n);   //两数据交换
Dll_PPDll void ReverseOrder(double *X, int num);   //数组逆序排列
Dll_PPDll double SolveTwoVecAngle(double a[3], double b[3]);  //两空间矢量的夹角

Dll_PPDll void PntVec2Kceta(double P[3], double Vec[3], double K[3], double ceta); // 由点Pt和其方向Vec确定一个旋转矩阵

Dll_PPDll int pnpoly(int nvert, double *vertx, double *verty, double testx, double testy);  //判断一个点是否在多边形的内部 PNPoly 算法
Dll_PPDll int FindPnploy(int nvert, double *vertx, double *verty, double testx, double testy);

Dll_PPDll void CalPlaneLineIntersectPoint(double planeVector[3], double planePoint[3], double lineVector[3], double linePoint[3], double IntSecPoint[3]); //直线与平面的求交点 
Dll_PPDll double CalPlaneSpacePntDistance(double planeVector[3], double planePoint[3], double spacePoint[3]); // 计算空间点到平面的距离（平面采用点-法式），返回值为距离
Dll_PPDll void CalProjPt_PttoPlane(double v1[3], double v2[3], double v3[3], double p1[3], double p2[3]);  //计算空间点到平面的投影点坐标
Dll_PPDll bool PointinTriangleJudge(double A[3], double B[3], double C[3], double P[3]);  // 判定点P是否在三角形ABC内部(假设P已经在ABC平面内)
Dll_PPDll bool PointinPlaneJudge(double A[3], double B[3], double C[3], double P[3], double thres);  // 判定点是否在平面上(允许一定误差阈值)

//Dll_PPDll void GetPntsFileOfArrayPnts(double** Array, int num, CString Filename);   // 从2维点数组Array中提取点坐标保存文件
//Dll_PPDll void GetPntsFileOfArrayPE(double** Array, int num, CString Filename);   // 从2维PE数组Array中提取PE坐标保存文件
//Dll_PPDll void GetPntsFileOfArray(double* Array, int num, CString Filename);   // 从1维数组Array中提取点坐标保存文件
Dll_PPDll void CalProjVec_VectoPlane(double v1[3], double n[3], double vec1[3], double vec2[3]);  //计算空间矢量到平面的投影矢量 （方法1: 夹角法） 

///B样条2维曲线弓高误差规划系列函数
Dll_PPDll void SolveBSplinePu(double *Bd1, double *Bd2, double u, double Pu[2]); //根据B样条系数Bd（1*4矩阵）和u（0<u<1）值求出Pu点值
Dll_PPDll double SolveGonggaowucha(double P0[2], double P1[2], double Pu[2]);  //计算实际弓高误差返回弓高误差值
Dll_PPDll void SolveMaxGonggaoParam(double P0[2], double P1[2], double tu0, double tu1, double *Bd1, double *Bd2, double *ypmax, double *tmax); //计算P0P1上最大弓高误差对应的参数u   
Dll_PPDll void SolveNextPMaxGonggao2(double e, double Dt0, double Dt1, double *Bd1, double *Bd2, double P0[2], double Pmaxkk[2], double *tmaxkk, double *ypmaxkk);  // 采用线性搜索法求最大弓高=e的下一个点
Dll_PPDll void SolveMaxGonggaoParamCrossKnot(double P0[2], double P1[2], double tu0, double tu1, double* Bd11, double* Bd12, double* Bd21, double* Bd22, double* ypmax, double* tmaxvalue);  // 计算P0P1上最大弓高误差对应的参数u
Dll_PPDll void SolveNextPMaxGonggaoCrossKnot(double e, double P0[2], double tu0, double* Bd11, double* Bd12, double* Bd21, double* Bd22, double Pmaxkk[2], double* tmaxkk, double* ypmaxkk);   //采用线性搜索法求最大弓高=e的下一个点  （特殊情况处理，跨节点的情况，下一个点在第二段的Bd上）
Dll_PPDll void PlanBspline2DGKnotPtm_GonggaoYunCha(double e, double** Bd1, double** Bd2, int KnotNum, int* GKnotNum, double **GKnotPnt); //获取一条Bspline曲线上的所有按弓高允差得到的节点位置