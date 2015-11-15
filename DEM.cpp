#include "stdio.h"
#include "math.h"
#include "stdlib.h"
#include "windows.h"
#include <iostream>
#include <string>
#include <fstream>
#include <strstream>
#include <sstream>
#include <vector>
#include <stdlib.h>
#include <algorithm>
#include <iomanip>
#define MAX_VERTEX_NUM 4092


typedef struct VERTEX3D_TYP
{
	double x;
	double y;
	double z;
	VERTEX3D_TYP operator = (const VERTEX3D_TYP &point3D)
	{
		x = point3D.x;
		y = point3D.y;
		z = point3D.z;
		return (*this);
	}
	bool operator ==(const VERTEX3D_TYP &point3D)
	{
		return(x==point3D.x&&y==point3D.y&&z==point3D.z);
	}
} VERTEX3D, *VERTEX3D_PTR;

typedef struct EDGE_TYP
{
	VERTEX3D v1;
	VERTEX3D v2;

} EDGE, *EDGE_PTR;

typedef struct TRIANGLE_TYP
{
	int i1; // 点索引
	int i2; 
	int i3; 

	TRIANGLE_TYP* pNext;
	TRIANGLE_TYP* pPrev;

} TRIANGLE, *TRIANGLE_PTR;

typedef struct MESH_TYP
{
	int vertex_num;
	int triangle_num;
	int bounding_num;
	std::vector<VERTEX3D>pVerArr; // 点数组
	std::vector<VERTEX3D>bounding;//输入边界点
	std::vector<int>bounding_index;//构建三角网的点云中的边界点索引

	std::vector<std::vector<TRIANGLE_PTR>> AdjacentFacesPerVertex;//存放点的邻接面的集合，与点集一一对应
	std::vector<std::vector<int>*> AdjacentVerticesPerVertex;//存放点的邻接点的集合，与点集一一对应
	TRIANGLE_PTR pTriArr; // 三角形链表

} MESH, *MESH_PTR;


//函数声明
void InitMesh(MESH_PTR pMesh, int ver_num);
void InitBounding(MESH_PTR pMesh, int bound_num );
void UnInitMesh(MESH_PTR pMesh);

void AddBoundingBox(MESH_PTR pMesh);
void RemoveBoundingBox(MESH_PTR pMesh);
void IncrementalDelaunay(MESH_PTR pMesh);

void Insert(MESH_PTR pMesh, int ver_index);//插入点
void Insert(MESH_PTR pMesh,int pb, int pp,int);//插入边

bool FlipTest(MESH_PTR pMesh, TRIANGLE_PTR pTestTri);

double InCircle(VERTEX3D pa, VERTEX3D pb, VERTEX3D pp, VERTEX3D  pd);
double InTriangle(MESH_PTR pMesh, VERTEX3D pVer, TRIANGLE_PTR pTri);

void InsertInTriangle(MESH_PTR pMesh, TRIANGLE_PTR pTargetTri, int ver_index);
void InsertOnEdge(MESH_PTR pMesh, TRIANGLE_PTR pTargetTri, int ver_index);


void RemoveTriangleNode(MESH_PTR pMesh, TRIANGLE_PTR pTri);
TRIANGLE_PTR AddTriangleNode(MESH_PTR pMesh, TRIANGLE_PTR pPrevTri, int i1, int i2, int i3);


void Input(std::string pFile, MESH_PTR pMesh);
void Output(std::string pFile, MESH_PTR pMesh);

void ReomveTriangleBounding(MESH_PTR pMesh);
int findOutTriangle(MESH_PTR pMesh, TRIANGLE_PTR pTri);

bool InMesh(MESH_PTR pMesh,VERTEX3D pVer1,VERTEX3D pVer2);

bool SegmentIntersect(VERTEX3D p1,VERTEX3D p2,VERTEX3D p3,VERTEX3D p4);

void CaculateAdjacentFacesPerVertex(MESH_PTR pMesh);//计算每个点的邻接面

int main(int argc, char** argv)
{
	std::string str1="E:\\SpecializedCourses\\DEM\\CDT\\DEM\\point.txt";
	std::string str2="E:\\SpecializedCourses\\DEM\\CDT\\DEM\\mesh.ply";
	MESH mesh;
	double last_time, this_time;

	Input(str1, &mesh);

	last_time = GetTickCount();		

	IncrementalDelaunay(&mesh);

	this_time = GetTickCount();	

	printf("带边界约束条件的增量三角剖分: %lg ms", this_time - last_time);

	Output(str2, &mesh);

	return 0;
}

void Input(std::string pFile, MESH_PTR pMesh)
{
	std::ifstream fp;fp.open(pFile);
	if (!fp)
	{
		fprintf(stderr,"Error:%s open failed\n", pFile);
		exit(1);
	}
	int amount;//总点数
	std::string line,type;

	std::getline(fp,line);
	double x,y,z;
	std::istringstream l_stream1(line);
	l_stream1>>type>>amount;
	if(type=="point")
	{
		pMesh->pVerArr.clear();
		InitMesh(pMesh, amount);
		for(int i=0;i<amount;i++)
		{
			std::getline(fp,line);
			std::istringstream l_stream1(line);
			VERTEX3D v1;
			l_stream1>>v1.x>>v1.y>>v1.z;
			pMesh->pVerArr.push_back(v1);
		}
	}
	std::getline(fp,line);
	std::istringstream l_stream2(line);
	l_stream2>>type>>amount;
	if(type=="bounding")
	{
		pMesh->bounding.clear();
		pMesh->bounding_num=amount;
		for(int i=0;i<amount;i++)
		{
			std::getline(fp,line);
			std::istringstream l_stream1(line);
			VERTEX3D v1;
			l_stream1>>v1.x>>v1.y>>v1.z;
			pMesh->bounding.push_back(v1);
		}
		std::cout<<std::endl;
	}
	pMesh->bounding_index.clear();
	for(size_t i=0;i<pMesh->bounding.size();++i)
	{
		bool in=false;
		for(size_t j=0;j<pMesh->pVerArr.size();++j)
		{
			if(pMesh->bounding[i]==pMesh->pVerArr[j])
			{
				pMesh->bounding_index.push_back(j);
				in=true;
				break;
			}
		}
		if(!in)
		{
			pMesh->pVerArr.push_back(pMesh->bounding[i]);
			pMesh->bounding_index.push_back(pMesh->pVerArr.size()-1);
		}
	}
	fp.close();

}
//增量三角剖分
/*
IncrementalDelaunay（V）
Input: 由n个点组成的二维点集V
Output: Delaunay三角剖分DT
1.添加一个超级三角形 （ 例如：使用三角形 abc， a=（0， 4M）， b=（-4M，-4M）， c=（4M， 0）， M=Max（{|x1|，|x2|，|x3|，...} U {|y1|，|y2|，|y3|，...}））
2. 使用三角形 abc初始化 DT（a，b，c）
3. for  i  1 to n 
do （Insert（DT（a，b，c，v1，v2，...，vi-1）， vi））   
4.从三角网（a，b，c，v1，v2，...，vn）中删除超级三角形和包含超级三角形中顶点的三角形并返回三角网DT（v1，v2，...，vn）.
*/
void IncrementalDelaunay(MESH_PTR pMesh)
{
	//添加超级三角形
	AddBoundingBox(pMesh);
	//按顺序将数组中所有点添加到三角网中
	for (int i=3; i<pMesh->pVerArr.size(); ++i)
	{
		Insert(pMesh, i);
	}
	//	边界约束
	for(int i=0;i<pMesh->bounding_index.size()-1;++i)
	{
		Insert(pMesh,pMesh->bounding_index[i],pMesh->bounding_index[i+1],i+1);
	}

	RemoveBoundingBox(pMesh);

}

void Output(std::string pFile, MESH_PTR pMesh)
{

	std::ofstream fp;
	fp.open(pFile);
	TRIANGLE_PTR pTri = pMesh->pTriArr;
	int* pi;
	int vertex_index;
	int tri_index = 0;
	fp<<"ply"<<std::endl<<"format ascii 1.0"<<std::endl<<"comment Author: CloudCompare (TELECOM PARISTECH/EDF R&D)"
		<<std::endl<<"obj_info Generated by CloudCompare!"<<std::endl<<"element vertex"<<" ";
	fp<<pMesh->pVerArr.size()<<std::endl;
	fp<<"property double x"<<std::endl<<"property double y"<<std::endl<<"property double z"<<std::endl<<"element face ";
	fp<<pMesh->triangle_num<<std::endl;
	fp<<"property list uchar int vertex_indices"<<std::endl<<"end_header"<<std::endl;
	for(size_t i=0;i<pMesh->pVerArr.size();++i)
	{
		fp<<std::setiosflags(std::ios::fixed)<<std::setprecision(7)<<pMesh->pVerArr[i].x<<" "
			<<pMesh->pVerArr[i].y<<" "
			<<pMesh->pVerArr[i].z<<" "<<std::endl;
	}
	while(pTri != NULL)	
	{
		fp<<3<<" ";
		pi = &(pTri->i1);
		for (int j=0; j<3; j++)	
		{	
			vertex_index = *pi++;
			fp<<vertex_index<<" ";
		}
		fp<<std::endl;
		pTri = pTri->pNext;
	}

	fp.close();

	std::ofstream outfile("E:\\SpecializedCourses\\DEM\\CDT\\DEM\\bounding.txt");
	for(int i=0;i<pMesh->bounding_index.size();++i)
		outfile<<std::setiosflags(std::ios::fixed)<<std::setprecision(7)<<pMesh->pVerArr[pMesh->bounding_index[i]].x<<" "
		<<pMesh->pVerArr[pMesh->bounding_index[i]].y<<" "
		<<pMesh->pVerArr[pMesh->bounding_index[i]].z<<" "<<std::endl;
	outfile.close();
	UnInitMesh(pMesh);
}

//初始化三角网内部的点
void InitMesh(MESH_PTR pMesh, int ver_num )
{
	pMesh->triangle_num=0;
	pMesh->vertex_num = ver_num;
	VERTEX3D v1;v1.x=0;v1.y=0;v1.z=0;
	pMesh->pVerArr.push_back(v1);
	pMesh->pVerArr.push_back(v1);
	pMesh->pVerArr.push_back(v1);
}

void UnInitMesh(MESH_PTR pMesh)
{
	pMesh->pVerArr.clear();
	pMesh->bounding.clear();

	TRIANGLE_PTR pTri = pMesh->pTriArr;
	TRIANGLE_PTR pTemp = NULL;
	while (pTri != NULL)
	{
		pTemp = pTri->pNext;
		free(pTri);
		pTri = pTemp;
	}
}

void AddBoundingBox(MESH_PTR pMesh)
{
	double max = 0;
	double max_x = 0;
	double max_y = 0;
	double t;

	for (int i=3; i<pMesh->pVerArr.size(); i++)
	{
		t = abs(pMesh->pVerArr[i].x);
		if (max_x < t)
		{
			max_x = t;
		}

		t = abs(pMesh->pVerArr[i].y);
		if (max_y < t)
		{
			max_y = t;
		}
	}

	max = max_x > max_y ? max_x:max_y;


	VERTEX3D v1 = {0, 4*max,0};
	VERTEX3D v2 = {-4*max, -4*max,0};
	VERTEX3D v3 = {4*max, 0,0};
	//将三个边界点添加到三角网的点数组最前面
	pMesh->pVerArr[0]= v1;
	pMesh->pVerArr[1]= v2;
	pMesh->pVerArr[2]= v3;
	pMesh->vertex_num=pMesh->vertex_num+3;
	AddTriangleNode(pMesh, NULL, 0, 1, 2);
}

void RemoveBoundingBox(MESH_PTR pMesh)
{
	int statify[3]={0,0,0};
	int vertex_index;
	int* pi;
	int k = 1;

	TRIANGLE_PTR pTri = pMesh->pTriArr;
	TRIANGLE_PTR pNext = NULL;
	while (pTri != NULL)
	{
		pNext = pTri->pNext;

		statify[0] = 0;
		statify[1] = 0;
		statify[2] = 0;

		pi = &(pTri->i1);
		for (int j=0, k = 1; j<3; j++, k*= 2)
		{			
			vertex_index = *pi++;

			if(vertex_index == 0 || vertex_index == 1 || vertex_index == 2) 
			{
				statify[j] = k;
			}
		}

		switch(statify[0] | statify[1] | statify[2] )
		{
		case 0: //不包含边界点
			break;
		case 1:
		case 2:
		case 4: //有一个点是边界点
			RemoveTriangleNode(pMesh, pTri);		
			break;
		case 3:
		case 5:
		case 6: // 有两个点是边界点
			RemoveTriangleNode(pMesh, pTri);			
			break;
		case 7: // 有三个点是边界点
			RemoveTriangleNode(pMesh, pTri);
			break;
		default:
			break;
		}
		pTri = pNext;
	}
	ReomveTriangleBounding(pMesh);
}

//正值：逆时针顺序；负值：顺时针顺序；0：共线
//pa->pb->pc的走向
double CounterClockWise(VERTEX3D& pa, VERTEX3D& pb, VERTEX3D& pc)
{
	return ((pb.x - pa.x)*(pc.y - pb.y) - (pc.x - pb.x)*(pb.y - pa.y));
}

//判断点是否在三角形中，设三角形顶点逆时针排序，若点在三角形中那么点与
//三角形的三个点任意两点的方向定是逆时针方向
//参数：三角网（得到三角形的点）、待检查点、待检查三角形
double InTriangle(MESH_PTR pMesh, VERTEX3D pVer, TRIANGLE_PTR pTri)
{
	int vertex_index;
	VERTEX3D pV1, pV2, pV3;
	//获取三角形的三个顶点
	vertex_index =pTri->i1;
	pV1 = pMesh->pVerArr[vertex_index];
	vertex_index =pTri->i2;
	pV2 = pMesh->pVerArr[vertex_index];
	vertex_index =pTri->i3;
	pV3 = pMesh->pVerArr[vertex_index];

	double ccw1 = CounterClockWise(pV1, pV2, pVer);
	double ccw2 = CounterClockWise(pV2, pV3, pVer);
	double ccw3 = CounterClockWise(pV3, pV1, pVer);

	double r = -1;
	if (ccw1>0 && ccw2>0 && ccw3>0)//在三角形内
	{
		r = 1;
	}
	else if(ccw1*ccw2*ccw3 == 0 && (ccw1*ccw2 > 0 || ccw1*ccw3 > 0 || ccw2*ccw3 > 0) )//在三角形的一条边上
	{
		r = 0;
	}
	if((pVer.x==pV1.x&&pVer.y==pV1.y&&pVer.z==pV1.z)//判断点是否在三角形的顶点上
		||(pVer.x==pV2.x&&pVer.y==pV2.y&&pVer.z==pV2.z)
		||(pVer.x==pV3.x&&pVer.y==pV3.y&&pVer.z==pV3.z))
		r=-2;
	return r;
}

/*
Insert（DT（a，b，c，v1，v2，...，vi-1）， vi）
1.找到包含vi的三角形vavbvc 
2.if （vi 在三角形 vavbvc）  
3.    添加 三角形 vavbvi， vbvcvi and vcvavi 到DT中 
FlipTest（DT， va， vb， vi）
FlipTest（DT， vb， vc， vi）
FlipTest（DT， vc， va， vi）
4.else if （vi 在三角形的一条边界上 （E.g. 边界 vavb） of vavbvc） 
5.    添加 vavivc， vivbvc， vavdvi ，vivdvb到 DT （d是包含边vavb的另一个三角形的第三个点）
FlipTest（DT， va， vd， vi）
FlipTest（DT， vc， va， vi）
FlipTest（DT， vd， vb， vi）
FlipTest（DT， vb， vc， vi）
6.return DT（a，b，c，v1，v2，...，vi）
*/
void Insert(MESH_PTR pMesh, int ver_index)
{
	//获得第i个点
	VERTEX3D pVer = pMesh->pVerArr[ver_index];
	TRIANGLE_PTR pTargetTri = NULL;
	TRIANGLE_PTR pEqualTri1 = NULL;
	TRIANGLE_PTR pEqualTri2 = NULL;
	int j = 0;
	TRIANGLE_PTR pTri = pMesh->pTriArr;//三角网第一个三角形
	while (pTri != NULL)
	{
		double r = InTriangle(pMesh, pVer, pTri);
		if(r > 0) // 在三角形中
		{
			pTargetTri = pTri;//获得当前点所在的三角形
		}
		else if (r == 0) //在边界上，则获得包含当前点的两个三角形，即有相邻边的两个三角形
		{
			if(j == 0)
			{
				pEqualTri1 = pTri;
				j++;				
			}
			else
			{
				pEqualTri2 = pTri;
			}

		}

		pTri = pTri->pNext;
	}
	if (pEqualTri1 != NULL && pEqualTri2 != NULL)
	{
		InsertOnEdge(pMesh, pEqualTri1, ver_index);//在第一个边界三角形中插入点
		InsertOnEdge(pMesh, pEqualTri2, ver_index);//在第二个边界三角形中插入点
	}
	else
	{
		InsertInTriangle(pMesh, pTargetTri, ver_index);//将点插入到所在三角形中
	}
}


void InsertInTriangle(MESH_PTR pMesh, TRIANGLE_PTR pTargetTri, int ver_index)
{
	int index_a, index_b, index_c;
	TRIANGLE_PTR pTri = NULL;
	TRIANGLE_PTR pNewTri = NULL;
	pTri = pTargetTri;	
	if(pTri == NULL)
	{
		return;
	}

	index_a = pTri->i1;
	index_b = pTri->i2;
	index_c = pTri->i3;

	for(int i=0; i<3; i++)
	{
		if(i == 0)
		{
			pNewTri = AddTriangleNode(pMesh, pTri, index_a, index_b, ver_index);
		}
		else if(i == 1)
		{
			pNewTri = AddTriangleNode(pMesh, pTri, index_b, index_c, ver_index);
		}
		else
		{
			pNewTri = AddTriangleNode(pMesh, pTri, index_c, index_a, ver_index);
		}

		if (pNewTri != NULL)
		{
			pTri = pNewTri;
		}
		else
		{
			pTri = pTri;
		}
	}
	pTri = pTargetTri;	
	TRIANGLE_PTR pTestTri[3];
	for (int i=0; i< 3; i++)
	{
		pTestTri[i] = pTri->pNext;

		pTri = pTri->pNext;
	}

	RemoveTriangleNode(pMesh, pTargetTri);

	for (int i=0; i< 3; i++)
	{
		FlipTest(pMesh, pTestTri[i]);
	}
}

void InsertOnEdge(MESH_PTR pMesh, TRIANGLE_PTR pTargetTri, int ver_index)
{
	//	std::cout<<"找到约束边1"<<std::endl;
	int index_a, index_b, index_c;
	TRIANGLE_PTR pTri = NULL;
	TRIANGLE_PTR pNewTri = NULL;

	pTri = pTargetTri;	//获得当前三角形
	if(pTri == NULL)
	{
		return;
	}

	// 获得当前三角形的三个顶点
	index_a = pTri->i1;
	index_b = pTri->i2;
	index_c = pTri->i3;

	// 插入两个三角形，另外一个三角形因为三点共线，不被插入
	for(int i=0; i<3; i++)
	{
		if(i == 0)
		{
			pNewTri = AddTriangleNode(pMesh, pTri, index_a, index_b, ver_index);
		}
		else if(i == 1)
		{
			pNewTri = AddTriangleNode(pMesh, pTri, index_b, index_c, ver_index);
		}
		else
		{
			pNewTri = AddTriangleNode(pMesh, pTri, index_c, index_a, ver_index);
		}		

		if (pNewTri != NULL)
		{
			pTri = pNewTri;
		}
		else
		{
			pTri = pTri;
		}
	}

	// 获取插入的两个三角形
	pTri = pTargetTri;	
	TRIANGLE_PTR pTestTri[2];
	for (int i=0; i< 2; i++)
	{
		pTestTri[i] = pTri->pNext;
		pTri = pTri->pNext;
	}

	// 删除当前被插入点的三角形
	RemoveTriangleNode(pMesh, pTargetTri);
	//对三角形进行
	for (int i=0; i< 2; i++)
	{
		FlipTest(pMesh, pTestTri[i]);
	}
}

/*
FlipTest（DT（a，b，c，v1，v2，...，vi）， va， vb， vi）
1.找到包含边界vavb的三角形第三个点（vd）
2.if（vi is in circumcircle of abd）  // InCircle（）
3.    then remove edge vavb， add new edge vivd into DT // UpdateDT（）
FlipTest（DT， va， vd， vi）
FlipTest（DT， vd， vb， vi）
*/
bool FlipTest(MESH_PTR pMesh, TRIANGLE_PTR pTestTri)
{
	bool flipped = false;

	int index_a = pTestTri->i1;
	int index_b = pTestTri->i2;
	int index_p = pTestTri->i3;

	int statify[3]={0,0,0};
	int vertex_index;
	int* pi;
	int k = 1;

	// 找到共边三角形
	TRIANGLE_PTR pTri = pMesh->pTriArr;

	int index_d = -1;
	while (pTri != NULL)
	{
		statify[0] = 0;
		statify[1] = 0;
		statify[2] = 0;
		pi = &(pTri->i1);
		for (int j=0, k = 1; j<3; j++, k*= 2)
		{
			vertex_index = *pi++;
			if(vertex_index == index_a || vertex_index == index_b)
			{
				statify[j] = k;
			}
		}
		//001、010、100按位或组合，代表1、2、4，找出共边的两个点
		switch(statify[0] | statify[1] | statify[2] )
		{
		case 3://三角形第一个点与第二个点共边
			if(CounterClockWise(pMesh->pVerArr[index_a], pMesh->pVerArr[index_b], pMesh->pVerArr[pTri->i3]) < 0)
			{
				index_d = pTri->i3;
			}				
			break;
		case 5://三角形第一个点与第三个点共边
			if(CounterClockWise(pMesh->pVerArr[index_a], pMesh->pVerArr[index_b], pMesh->pVerArr[pTri->i2]) < 0)
			{
				index_d = pTri->i2;
			}				
			break;
		case 6://三角形第二个点与第三个点共边 
			if(CounterClockWise(pMesh->pVerArr[index_a], pMesh->pVerArr[index_b], pMesh->pVerArr[pTri->i1]) < 0)
			{
				index_d = pTri->i1;
			}
			break;

		default:
			break;
		}

		if (index_d != -1)
		{
			VERTEX3D pa = pMesh->pVerArr[index_a];
			VERTEX3D pb = pMesh->pVerArr[index_b];
			VERTEX3D pp = pMesh->pVerArr[index_p];
			VERTEX3D pd = pMesh->pVerArr[index_d];


			if(InCircle( pa, pb, pp, pd) < 0) //在局部范围内不是Delaunay三角剖分
			{
				flipped = true;

				TRIANGLE_PTR pT1 = AddTriangleNode(pMesh, pTestTri, pTestTri->i1, index_d, pTestTri->i3);				

				TRIANGLE_PTR pT2 = AddTriangleNode(pMesh, pT1, index_d, pTestTri->i2, index_p);				

				RemoveTriangleNode(pMesh, pTestTri);

				RemoveTriangleNode(pMesh, pTri);

				FlipTest(pMesh, pT1); 
				FlipTest(pMesh, pT2); 

				break;
			}			
		}
		pTri = pTri->pNext;
	}

	return flipped;
}
/*
InCircle（va， vb， vd， vc）
if |a^b^， c^b^， d^b^| > 0， vd 不在三角形 abc的外接圆中 //a^坐标为（ax，ay，ax*ax+ay*ay），a^b^为向量
if |a^b^， c^b^， d^b^| == 0， vd 在三角形 abc的外接圆上
if |a^b^， c^b^， d^b^| < 0， vd 在三角形 abc的外接圆内
*/
double InCircle(VERTEX3D pa, VERTEX3D pb, VERTEX3D pp, VERTEX3D  pd)
{
	double det;
	double alift, blift, plift, bdxpdy, pdxbdy, pdxady, adxpdy, adxbdy, bdxady;

	double adx = pa.x - pd.x;
	double ady = pa.y - pd.y;

	double bdx = pb.x - pd.x;
	double bdy = pb.y - pd.y;

	double pdx = pp.x - pd.x;
	double pdy = pp.y - pd.y;

	bdxpdy = bdx * pdy;
	pdxbdy = pdx * bdy;
	alift = adx * adx + ady * ady;

	pdxady = pdx * ady;
	adxpdy = adx * pdy;
	blift = bdx * bdx + bdy * bdy;

	adxbdy = adx * bdy;
	bdxady = bdx * ady;
	plift = pdx * pdx + pdy * pdy;

	det = alift * (bdxpdy - pdxbdy)
		+ blift * (pdxady - adxpdy)
		+ plift * (adxbdy - bdxady);

	return -det;//返回的负值才是行列式的值
}
//从三角形链表中删除一个三角形

void RemoveTriangleNode(MESH_PTR pMesh, TRIANGLE_PTR pTri)
{
	if (pTri == NULL)
	{
		return;
	}

	//从三角形链表中删除
	if (pTri->pPrev != NULL)
	{
		pTri->pPrev->pNext = pTri->pNext;
	}
	else //删除头结点，需要将头结点后移
	{
		pMesh->pTriArr = pTri->pNext;
	}

	if (pTri->pNext != NULL)//节点尾部飞空，需要将节点
	{
		pTri->pNext->pPrev = pTri->pPrev;
	}
	//删除当前点
	pMesh->triangle_num=pMesh->triangle_num-1;
	free(pTri);	
}
//创建一个新节点，并将其输入到新三角网中，并返回当前创建的三角形
//参数：pMesh三角网 pPrevtri三角网最后一个三角形 i1、i2、i3待插入三角形的三个顶点
//返回三角网最后一个三角形，也就是新插入的三角形
TRIANGLE_PTR AddTriangleNode(MESH_PTR pMesh, TRIANGLE_PTR pPrevTri, int i1, int i2, int i3)
{
	//判断三点是否在一条直线上

	if(CounterClockWise(pMesh->pVerArr[i1], pMesh->pVerArr[i2], pMesh->pVerArr[i3]) == 0)
	{
		return NULL;
	}
	//创建一个新的三角形节点

	TRIANGLE_PTR pNewTestTri = (TRIANGLE_PTR)malloc(sizeof(TRIANGLE));

	pNewTestTri->i1 = i1;
	pNewTestTri->i2 = i2;
	pNewTestTri->i3 = i3;
	//将当前三角形添加到三角形链表的最后，如果三角形链表为空，则将当前链表值指向新建的三角形

	if (pPrevTri == NULL)
	{
		pMesh->pTriArr = pNewTestTri;
		pNewTestTri->pNext = NULL;
		pNewTestTri->pPrev = NULL;
	}
	else
	{
		pNewTestTri->pNext = pPrevTri->pNext;
		pNewTestTri->pPrev = pPrevTri;

		if(pPrevTri->pNext != NULL)
		{
			pPrevTri->pNext->pPrev = pNewTestTri;
		}

		pPrevTri->pNext = pNewTestTri;
	}
	pMesh->triangle_num=pMesh->triangle_num+1;
	return pNewTestTri;
}

//在边界索引中查询
bool findindex(MESH_PTR pMesh,int index)
{
	bool finded=false;
	for(int i=0;i<pMesh->bounding_index.size();++i)
	{
		if(index==pMesh->bounding_index[i])
		{
			finded=true;
			break;
		}
	}
	return finded;
}
//删除三个点都是边界点的三角形
void ReomveTriangleBounding(MESH_PTR pMesh)
{


	TRIANGLE_PTR pTri = pMesh->pTriArr;
	TRIANGLE_PTR pNext = NULL;
	while (pTri != NULL)
	{
		pNext = pTri->pNext;
		switch(findOutTriangle(pMesh,pTri))
		{
		case 0: //不包含边界点
		case 1:
		case 2:
		case 4: //有一个点是边界点	
		case 3:
		case 5:
		case 6: // 有两个点是边界点		
			break;
		case 7: // 有三个点是边界点
			RemoveTriangleNode(pMesh, pTri);
			break;
		default:
			break;
		}	
		pTri = pNext;
	}	
}
//查询三角形三顶点有几个是边界点
int findOutTriangle(MESH_PTR pMesh, TRIANGLE_PTR pTri)
{
	int statify[3]={0,0,0};
	int vertex_index;
	int* pi;
	int k = 1;

	statify[0] = 0;
	statify[1] = 0;
	statify[2] = 0;

	pi = &(pTri->i1);
	for (int j=0, k = 1; j<3; j++, k*= 2)
	{			
		vertex_index = *pi++;

		if(findindex(pMesh,vertex_index)) 
		{
			statify[j] = k;
		}
	}
	return (statify[0] | statify[1] | statify[2]);

}

/*
插入边界边
*/
//判断约束线是否已在三角网中
bool InMesh(MESH_PTR pMesh,int pVer1,int pVer2)
{
	bool in=false;
	TRIANGLE_PTR pTri = pMesh->pTriArr;//三角网第一个三角形
	while (pTri != NULL)
	{

		//判断点是否在三角形的顶点上
		if(pVer1==pTri->i1||pVer1==pTri->i2||pVer1==pTri->i3)
		{//判断点是否在三角形的顶点上
			if(pVer2==pTri->i1||pVer2==pTri->i2||pVer2==pTri->i3)
			{
				in=true;
				break;
			}
		}
		pTri = pTri->pNext;
	}
	return in;
}

/************************两线段是否相交*******************************/
//direction函数用于计算p1->p2和p1->p3两向量的的叉积。
double direction(VERTEX3D p1,VERTEX3D p2,VERTEX3D p3)
{
	std::pair<double,double> d1=std::make_pair(p3.x-p1.x,p3.y-p1.y);
	std::pair<double,double> d2=std::make_pair(p2.x-p1.x,p2.y-p1.y);
	return d1.first*d2.second-d1.second*d2.first;
}
//当p3在直线p1->p2上时，OnSegment函数用于确认p3在p1->p2上，还是在p1->p2的延长线上(判断p3是否在以p1->p2为对角线的矩形内)
bool OnSegment(VERTEX3D p1,VERTEX3D p2,VERTEX3D p3)
{
	double x_min,x_max,y_min,y_max;
	if(p1.x<p2.x)
	{
		x_min=p1.x;
		x_max=p2.x;
	}
	else
	{
		x_min=p2.x;
		x_max=p1.x;
	}
	if(p1.y<p2.y)
	{
		y_min=p1.y;
		y_max=p2.y;
	}
	else
	{
		y_min=p2.y;
		y_max=p1.y;
	}
	if(p3.x<x_min || p3.x>x_max || p3.y<y_min || p3.y>y_max)
		return false;
	else
		return true;
}
//判断向量p1->p2与向量p3->p4是否相交
bool SegmentIntersect(VERTEX3D p1,VERTEX3D p2,VERTEX3D p3,VERTEX3D p4){
	double d1=direction(p3,p4,p1);
	double d2=direction(p3,p4,p2);
	double d3=direction(p1,p2,p3);
	double d4=direction(p1,p2,p4);

	if(d1*d2<0 && d3*d4<0)
		return true;
	else if(d1==0 && OnSegment(p3,p4,p1))
		return true;
	else if(d2==0 && OnSegment(p3,p4,p2))
		return true;
	else if(d3==0 && OnSegment(p1,p2,p3))
		return true;
	else if(d4==0 && OnSegment(p1,p2,p4))
		return true;
	else
		return false;
}

int InTriangle(MESH_PTR pMesh,TRIANGLE_PTR pTri,int p1,int p2)
{
	int statify[3]={0,0,0};
	statify[0] = 0;
	statify[1] = 0;
	statify[2] = 0;
	if(SegmentIntersect(pMesh->pVerArr[p1],pMesh->pVerArr[p2],pMesh->pVerArr[pTri->i1],pMesh->pVerArr[pTri->i2]))
	{
		statify[1] = 1;
	}
	if(SegmentIntersect(pMesh->pVerArr[p1],pMesh->pVerArr[p2],pMesh->pVerArr[pTri->i2],pMesh->pVerArr[pTri->i3]))
	{
		statify[1] = 2;
	}
	if(SegmentIntersect(pMesh->pVerArr[p1],pMesh->pVerArr[p2],pMesh->pVerArr[pTri->i1],pMesh->pVerArr[pTri->i3]))
	{
		statify[1] = 4;
	}
	return (statify[0] | statify[1] | statify[2]);
	//返回0：不相交
	//返回1:1、2边相交
	//返回2:2、3相交

}

//相交线段p1-p2与p3->p4求交
VERTEX3D PointX(VERTEX3D p1,VERTEX3D p2,VERTEX3D p3,VERTEX3D p4)
{
	double b1=(p2.y-p1.y)*p1.x+(p1.x-p2.x)*p1.y;
	double b2=(p4.y-p3.y)*p3.x+(p3.x-p4.x)*p3.y;
	double D=(p2.x-p1.x)*(p4.y-p3.y)-(p4.x-p3.x)*(p2.y-p1.y);
	double D1=b2*(p2.x-p1.x)-b1*(p4.x-p3.x);
	double D2=b2*(p2.y-p1.y)-b1*(p4.y-p3.y);
	VERTEX3D v;
	v.x=D1/D;v.y=D2/D;v.z=(p1.z+p2.z+p3.z+p4.z)/4;
	return v;
}
void Insert(MESH_PTR pMesh,int p1, int p2,int a)
{
	CaculateAdjacentFacesPerVertex(pMesh);
	bool add=false;
	if(!InMesh(pMesh,p1,p2))
	{
		if(pMesh->AdjacentFacesPerVertex[p1].size()!=0)
		{
			size_t j=0;
			for(;j<pMesh->AdjacentFacesPerVertex[p1].size();++j)
			{
				TRIANGLE_PTR pTr=pMesh->AdjacentFacesPerVertex[p1][j];
				if(pTr->i1==p1)
				{
					if(SegmentIntersect(pMesh->pVerArr[p1],pMesh->pVerArr[p2],pMesh->pVerArr[pTr->i2],pMesh->pVerArr[pTr->i3]))
					{
						//求出相交线段的交点（x0、y0、z0）插入到点云最后，并将其索引插入到边界列表中
						VERTEX3D v=PointX(pMesh->pVerArr[p1],pMesh->pVerArr[p2],pMesh->pVerArr[pTr->i2],pMesh->pVerArr[pTr->i3]);
						pMesh->pVerArr.push_back(v);add=true;
						pMesh->bounding_index.insert(pMesh->bounding_index.begin()+a,pMesh->pVerArr.size()-1);
						Insert(pMesh,pMesh->pVerArr.size()-1);
						break;
					}
				}
				else if(pTr->i2==p1)
				{
					if(SegmentIntersect(pMesh->pVerArr[p1],pMesh->pVerArr[p2],pMesh->pVerArr[pTr->i1],pMesh->pVerArr[pTr->i3]))
					{
						VERTEX3D v=PointX(pMesh->pVerArr[p1],pMesh->pVerArr[p2],pMesh->pVerArr[pTr->i1],pMesh->pVerArr[pTr->i3]);
						pMesh->pVerArr.push_back(v);add=true;
						pMesh->bounding_index.insert(pMesh->bounding_index.begin()+a,pMesh->pVerArr.size()-1);
						Insert(pMesh,pMesh->pVerArr.size()-1);
						break;
					}
				}
				else if(pTr->i3==p1)
				{
					if(SegmentIntersect(pMesh->pVerArr[p1],pMesh->pVerArr[p2],pMesh->pVerArr[pTr->i2],pMesh->pVerArr[pTr->i1]))
					{
						VERTEX3D v=PointX(pMesh->pVerArr[p1],pMesh->pVerArr[p2],pMesh->pVerArr[pTr->i2],pMesh->pVerArr[pTr->i1]);
						pMesh->pVerArr.push_back(v);add=true;
						pMesh->bounding_index.insert(pMesh->bounding_index.begin()+a,pMesh->pVerArr.size()-1);
						Insert(pMesh,pMesh->pVerArr.size()-1);
						break;
					}
				}
			}
		}
	}
	//插入完成后删除邻接表
	if(add==true)
	{
		for (size_t i = 0; i <pMesh->pVerArr.size()-1; i++)
		{
			pMesh->AdjacentFacesPerVertex[i].clear();
		}//首先分配好存储空间
	}
	else
	{
		for (size_t i = 0; i <pMesh->pVerArr.size(); i++)
		{
			pMesh->AdjacentFacesPerVertex[i].clear();
		}//首先分配好存储空间	
	}
	pMesh->AdjacentFacesPerVertex.clear();
}

//计算邻接面
void CaculateAdjacentFacesPerVertex(MESH_PTR pMesh)
{
	TRIANGLE_PTR pTri = pMesh->pTriArr;//三角网第一个三角形
	pMesh->AdjacentFacesPerVertex.reserve(pMesh->pVerArr.size());
	for (size_t i = 0; i <pMesh->pVerArr.size(); i++)
	{
		std::vector<TRIANGLE_PTR>list;
		pMesh->AdjacentFacesPerVertex.push_back(list);
	}//首先分配好存储空间
	int i=0;
	while (pTri!=NULL)
	{
		pMesh->AdjacentFacesPerVertex[pTri->i1].push_back(pTri);
		pMesh->AdjacentFacesPerVertex[pTri->i2].push_back(pTri);
		pMesh->AdjacentFacesPerVertex[pTri->i3].push_back(pTri);

		i=i+1;
		pTri=pTri->pNext;
	}//遍历三角形集合，使用三角形信息补充邻接面表
}
