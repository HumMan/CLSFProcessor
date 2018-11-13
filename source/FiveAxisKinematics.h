#pragma once

#include "vec.h"
#include "matrix.h"
using namespace BaluLib;

#include "refactored/atptokenizer.h"

using namespace CLSFProcessor;

#include <map>
#include <set>

#define M_PI 3.14159265358979323846
#define M_PI_2 1.57079632679489661923

struct Utils
{

public:

	double sqr(double value)
	{
		return value * value;
	}
	double AngleFromDir(TVec<double, 2> vec)
	{
		auto c = acos(vec[0]);
		if (asin(vec[1]) > 0)
			return c;
		else
			return c + M_PI;
	}

	float DegToRad(float deg)
	{
		return deg / 180.0 * M_PI;
	}

	bool AIsInPole(double A)
	{
		return !IsInMinMax(A,A_pole_min,A_pole_max);
	}

	bool IsInMinMax(double value, double min, double max)
	{
		return value > min && value < max;
	}

	bool CIsInPole(T C)
	{
		return !IsInMinMax(C,C_pole_min,C_pole_max);
	}

	T To0_360Space(T angle)
	{
		angle=fmod(angle,T(2*M_PI));
		if(angle<0)angle=2*M_PI+angle;
		return angle;
	}
	bool AIsMoveOverPole(T start,T end)
		//перемещение вокруг оси A идет через полюс
		//start,end - перемещение в виде линейных координат (с учетом скручиваний и раскручиваний)
	{
		return start>A_pole_max||start<A_pole_min||
			end>A_pole_max||end<A_pole_min;
	}
	bool CIsMoveOverPole(T start,T end)
		//перемещение вокруг оси C идет через полюс
		//start,end - перемещение в виде линейных координат (с учетом скручиваний и раскручиваний)
	{
		bool isvalid= start>C_pole_max||start<C_pole_min||
			end>C_pole_max||end<C_pole_min;
		return isvalid;
	}
};

#include "Uni5axis.h"

template<class T,class TMachineToolKinematics>
class TATPProcessor:public TMachineToolKinematics
{
public:

	T tolerance;
	T rapid_tolerance;
	T rapid_feed;
	T contour_max_feed;
	int frames_on_1sec_max;

	T pole_change_height;
	T pole_change_engage_height;

	bool circle_interpolation_center_absol;

	TATPProcessor(std::map<string,string> &ini_params):TMachineToolKinematics(ini_params)
	{
		tolerance=atof(ini_params["tolerance"].c_str());
		rapid_tolerance=atof(ini_params["rapid_tolerance"].c_str());
		rapid_feed=atof(ini_params["rapid_feed"].c_str());
		contour_max_feed=atof(ini_params["contour_max_feed"].c_str());
		frames_on_1sec_max = atoi(ini_params["frames_on_1sec_max"].c_str());

		pole_change_height=atof(ini_params["pole_change_height"].c_str());
		pole_change_engage_height=atof(ini_params["pole_change_engage_height"].c_str());

		circle_interpolation_center_absol=atoi(ini_params["circle_interpolation_center_absol"].c_str());
	}

	struct TKinematicsNode:public TKinematicsPair<T>
	{
		bool move_over_pole[2];		//индекс в массив - текущий проверяемый вариант, но не текущая линия (т.к. маршруты перекрываются)
		bool change_line[2];

		T A_pole_change_delta[2];	//
		T C_pole_change_delta[2];	//
		T A_inc[2];					//
		T C_inc[2];					//
		int poles_count[2];			//

		int best_line;
		T tol;
	};

	struct TPipelineElement:public TMachineState<T>
	{
		TToolOrientation<T> tool_orient;
		TKinematicsNode machine_orient;
	};

	T min(T v0, T v1)
	{
		if (v0 < v1)
			return v0;
		else
			return v1;
	}
	
	bool IsCCWMove(T a0,T a1,T& dist)
		//result - направление наикратчайшего перемещения из a0 в a1
		//dist - величина и знак перемещения (+ это CCW)
	{
		a0=To0_360Space(a0);
		a1=To0_360Space(a1);
		T dist_0=abs(a0-a1);
		T dist_1=abs(2*M_PI-abs(dist_0));

		bool result=(dist_0<dist_1)?(a1>a0):(a1<a0);
		dist=(result?1.0:-1.0)*min(dist_0,dist_1);
		return result;
	}

	T MovementCost(TKinematics<T> v0,TKinematics<T> v1)
		//result - стоимость перемещения по наикратчайшему пути из v0 в v1
	{
		T da,dc;
		IsCCWMove(v0.A,v1.A,da);
		IsCCWMove(v0.C,v1.C,dc);
		return sqrt(sqr(da)+sqr(dc));
	}

	void FindAnyC(std::vector<TPipelineElement> &pipe, int start_from,int &region_end)
	{
		region_end=pipe.size()-1;
		for(int i=start_from;i<=pipe.size()-1;i++)
		{
			if(pipe[i].machine_orient.any_C)
			{
				region_end=i;
				break;
			}
		}
	}

	bool NeedLinearizeMovementSwap(std::vector<TPipelineElement> &pipe,int i)
		//result - кратчайшие перемещения находятся в вариантах с одинаковыми индексами
	{
		TKinematicsNode &t0=pipe[i].machine_orient;
		TKinematicsNode &t1=pipe[i+1].machine_orient;
		bool result= (
			MovementCost(t0.variant[0],t1.variant[0])>
			MovementCost(t0.variant[0],t1.variant[1]));
		assert(result==(MovementCost(t0.variant[1],t1.variant[0])<MovementCost(t0.variant[1],t1.variant[1])));
		return result;
	}

	T min_tol;
	T max_tol;

	T GetInterpolationTolerance(TKinematics<T> p0,TKinematics<T> p1,TToolOrientation<T> &middle_tool)
	{
		TVec<T,3> kp0,kp1,kp_middle,tool_dir0,tool_dir1;
		TKinematics<T> k;

		FromMachineToolKinematics(p0,kp0,tool_dir0);
		FromMachineToolKinematics(p1,kp1,tool_dir1);

		TKinematics<T> middle;
		middle.A=(p0.A+p1.A)*0.5; //TODO можно перейти к кинематике как к 5мерному вектору -> получаем простейшую интерполяцию и расстояние
		middle.C=(p0.C+p1.C)*0.5;
		middle.pos=(p0.pos+p1.pos)*0.5;
		FromMachineToolKinematics(middle,middle_tool.pos,middle_tool.dir);

		T tol=((kp0+kp1)*0.5).Distance(middle_tool.pos);
		if(tol>max_tol)max_tol=tol;
		if(tol<min_tol)min_tol=tol;

		middle_tool.pos=(kp0+kp1)*0.5;
		middle_tool.dir=((tool_dir0+tool_dir1)*0.5).GetNormalized();

		return tol;
	}

	void GetMovement(TKinematics<T> p0,TKinematics<T> p1,T& delta_A,T& delta_C)
		//delta_A,delta_C - приращения необходимые для перемещения из кинематики 0 в кинематику 1
	{
		IsCCWMove(p0.A,p1.A,delta_A);
		IsCCWMove(p0.C,p1.C,delta_C);
	}

	void Clear(std::vector<TPipelineElement> &pipe)
	{
		for(int i=0;i<pipe.size();i++)
		{
			memset(&pipe[i].machine_orient,0,sizeof(pipe[i].machine_orient));
			for(int k=0;k<2;k++)
				pipe[i].machine_orient.poles_count[k]=-1;
			pipe[i].machine_orient.best_line=-1;
		}
	}

	void CalcKinematics(std::vector<TPipelineElement> &pipe)
	{
		for(int i=0;i<pipe.size();i++)
		{
			if(pipe[i].mask&PrimitiveMask::CIRCLE
				||pipe[i].mask&PrimitiveMask::LINE)
				ToMachineToolKinematics(pipe[i].tool_orient.pos,pipe[i].tool_orient.dir,pipe[i].machine_orient);
		}
	}

	bool IsOrthogonalVector(TVec<T,3> v,int& axis,bool& pos_dir)
	{
		int c=0;
		for(int i=0;i<3;i++)
			if(abs(v[i])>ortho_vec_epsilon)
			{
				axis=i;
				c++;
			}
		if(c==1)
		{
			pos_dir=(v[axis]>0);
			return true;
		}else
			return false;
	}

	void CircleLinearApprox(TVec<T,2> v0,TVec<T,2> v1,T rad,T arc_error,std::vector<TVec<T,2>>& result,bool nearest_arc,bool ccw,bool full_circle=false)
	{
		T l0,l1,dist;
		if(nearest_arc)
		{
			l0=To0_360Space(AngleFromDir(v0));
			l1=To0_360Space(AngleFromDir(v1));
			bool ccw=IsCCWMove(l0,l1,dist);
		}else //с явным указанием направления поворота
		{
			l0=To0_360Space(AngleFromDir(v0));
			l1=To0_360Space(AngleFromDir(v1));
			bool _ccw=IsCCWMove(l0,l1,dist);
			if(ccw!=_ccw)
			{
				dist=dist-2*M_PI;
			}
			if(full_circle)dist=ccw?2*M_PI:-2*M_PI;
		}
		int points_high=abs(dist)/acos(1-arc_error/rad);
		result.resize(points_high-1);
		result[0]=v0;
		result.back()=v1;
		for(int i=1;i<points_high;i++)
		{
			//T ang=l0*(points_high-i)/(T)points_high+l1*i/(T)points_high;
			T ang=l0+dist*i/(T)points_high;
			result[i]=TVec<T,2>(cos(ang),sin(ang));
		}
	}

	double Blend(double v0, double v1, double factor)
	{
		return v0 * (1 - factor) + v1 * factor;
	}

	void CalcCircleParameters(std::vector<TPipelineElement> &pipe)//TODO пока что здесь все дуги разбиваются на мелкие линейные отрезки
	{

		std::vector<TVec<T,2>> points;
		std::vector<TPipelineElement> new_pipe;
		new_pipe.reserve(pipe.size());

		for(int i=0;i<pipe.size();i++)
		{
			if(pipe[i].mask==PrimitiveMask::CIRCLE)
			{
				pipe[i].tool_orient.pos=pipe[i+1].tool_orient.pos;
				if(pipe[i].tool_orient.pos.Distance(pipe[i-1].tool_orient.pos)<0.01)
					pipe[i].mask=PrimitiveMask::LINE;
				new_pipe.push_back(pipe[i]);
				i++;
			}else
				new_pipe.push_back(pipe[i]);
		}
		int t=0,off=0;

		pipe = new_pipe;

		//for(int calc_count_to_insert=0;calc_count_to_insert<2;calc_count_to_insert++)
		for(int i=0;i<pipe.size();i++)
		{
			if(pipe[i].mask==PrimitiveMask::CIRCLE)
			{
				int axis;
				bool pos_dir;
				if((use_circles&&!IsOrthogonalVector(pipe[i].normal,axis,pos_dir))||(!use_circles))
				{
					TVec<T,3> lx,ly,lz;
					TVec<T,3> center=pipe[i].center;
					T rad=pipe[i].radius;

					lx=(pipe[i-1].tool_orient.pos-center).GetNormalized();
					lz=pipe[i].normal;
					ly=lz.Cross(lx).GetNormalized();

					if(pipe[i].spiral_times!=-1)
					{
						points.clear();
						std::vector<TVec<T,2>> _points;
						ly=lz.Cross((pipe[i-1].tool_orient.pos-center).GetNormalized()).GetNormalized();
						lx=ly.Cross(lz);

						for(int t=0;t<pipe[i].spiral_times-1;t++)
						{
							CircleLinearApprox(TVec<T,2>(1,0),TVec<T,2>(1,0),
								rad,0.01,_points,false,false,true);
							for(int k=0;k<_points.size();k++)points.push_back(_points[k]);
						}
						CircleLinearApprox(TVec<T,2>(1,0),TVec<T,2>(
							(pipe[i].tool_orient.pos-center)*lx,
							(pipe[i].tool_orient.pos-center)*ly).GetNormalized(),
							rad,0.01,_points,false,false);
						for(int k=0;k<_points.size();k++)points.push_back(_points[k]);
					}else
					{
						CircleLinearApprox(TVec<T,2>(1,0),
							TVec<T,2>(
							(pipe[i].tool_orient.pos-center)*lx,
							(pipe[i].tool_orient.pos-center)*ly).GetNormalized(),
							rad,0.01,points,false,false);
					}

					//if(calc_count_to_insert==0)
					//{

					//}else
					{
						T z_step=lz*(pipe[i].tool_orient.pos-pipe[i-1].tool_orient.pos);
						T z_first=pipe[i-1].tool_orient.pos[2];
						T z_last=pipe[i].tool_orient.pos[2];

						pipe[i].mask=PrimitiveMask::LINE;
						int points_count=points.size();
						if(points.size()>2)
						{
							//TODO очень медленно при большом количествве
							
							//TODO я убрал pipe.InsertCount(points_count-2,i);
							
							int last_point=i+points_count-2;
							pipe[last_point].tool_orient=pipe[i].tool_orient;
							

							*(TMachineState<T>*)&pipe[last_point]=*(TMachineState<T>*)&pipe[i];

							pipe[last_point].mask=PrimitiveMask::LINE;

							for(int t=1;t<points_count-1;t++)
							{
								*(TMachineState<T>*)&pipe[i+t-1]=*(TMachineState<T>*)&pipe[last_point];
								pipe[i+t-1].mask=PrimitiveMask::LINE;
								
								if(pipe[i].spiral_times!=-1)
								{
									pipe[i+t-1].tool_orient.pos=(lx*points[t][0]+ly*points[t][1])*rad+center;
									pipe[i+t-1].tool_orient.pos[2]=Blend(z_first,z_last,(t)/T(points_count));
								}else
									pipe[i+t-1].tool_orient.pos=(lx*points[t][0]+ly*points[t][1])*rad+center;

								pipe[i+t-1].tool_orient.dir=pipe[last_point].tool_orient.dir;
							}
						}
					}
				}else
				{
				}
			}
		}
	}

	void ValidateKinematics(std::vector<TPipelineElement> &pipe)
	{
		for(int i=0;i<=pipe.size()-1;i++)
			if(pipe[i].mask&PrimitiveMask::CIRCLE
				||pipe[i].mask&PrimitiveMask::LINE)
		{
			for(int k=0;k<2;k++)
				if(pipe[i].machine_orient.any_C)
					pipe[i].machine_orient.valid[k]=true;
				else
					pipe[i].machine_orient.valid[k]=!AIsInPole(pipe[i].machine_orient.variant[k].A);
			if(!(pipe[i].machine_orient.valid[0]||pipe[i].machine_orient.valid[1]))
				throw exception("Invalid tool orientation");
		}
	}

	void SwapVariants(std::vector<TPipelineElement> &pipe)
	{
		for(int i=0;i<pipe.size()-1;i++)
			if(NeedLinearizeMovementSwap(pipe,i))
				Swap(pipe[i+1].machine_orient.variant[0],pipe[i+1].machine_orient.variant[1]);
	}

	void GetCWithMaxX(TVec<T,3> tool_pos,T A,T initial_C,T& result_C,TVec<T,3>& result_pos)
	{
		T step=DegToRad(2.0);
		T dist=10e20;
		TVec<T,3> init_pos,p0,p1,t;
		//while(dist>1)
		{
			init_pos=ToMachineToolKinematics(tool_pos,A,initial_C);
			result_pos=init_pos;
			p0=ToMachineToolKinematics(tool_pos,A,initial_C-step);
			p1=ToMachineToolKinematics(tool_pos,A,initial_C+step);
			if(p0[0]>init_pos[0]&&p0[0]>p1[0])
			{
				initial_C-=step;
				//while((t=ToMachineToolKinematics(tool_pos,A,initial_C-=step))[0]>p0[0])
				{
					//p0=t;
					result_pos=p0;
				}
			}else if(p1[0]>init_pos[0]&&p1[0]>p0[0])
			{
				initial_C+=step;
				//while((t=ToMachineToolKinematics(tool_pos,A,initial_C+=step))[0]>p1[0])
				{
					//p1=t;
					result_pos=p1;
				}
			}else
			{
			}
		}
		result_C=initial_C;
	}

	void DetermineAnyCKinematics(std::vector<TPipelineElement> &pipe)
	{

		//здесь мы можем использовать различные критерии работы координат при any_C
		switch(any_C_criteria)
		{
		case 0:
			{
				//if(pipe[0].machine_orient.any_C)
				//{
				//	ToMachineToolKinematics(pipe[0].tool_orient.pos,pipe[0].tool_orient.dir,pipe[0].machine_orient);
				//}
				//for(int i=1;i<pipe.size();i++)
				//	if(pipe[i].mask&PrimitiveMask::CIRCLE
				//		||pipe[i].mask&PrimitiveMask::LINE)
				//		if(pipe[i].machine_orient.any_C)
				//		{
				//			ToMachineToolKinematics(pipe[i].tool_orient.pos,pipe[i].tool_orient.dir,pipe[i].machine_orient,true,pipe[i-1].machine_orient.variant[0].C);
				//		}
			}break;
		case 1://критерий максимальной координаты по X (для ТФЦ-600)
			{
				//T step=DegToRad(2.0);
				//if(pipe[0].machine_orient.any_C)
				//{
				//	ToMachineToolKinematics(pipe[0].tool_orient.pos,pipe[0].tool_orient.dir,pipe[0].machine_orient);
				//}
				//for(int i=1;i<pipe.size();i++)
				//	if(pipe[i].mask&PrimitiveMask::CIRCLE
				//		||pipe[i].mask&PrimitiveMask::LINE)
				//		if(pipe[i].machine_orient.any_C)
				//		{
				//			T initial_C=pipe[i-1].machine_orient.variant[0].C;
				//			T result_C;
				//			TVec<T,3> result_pos;

				//			if(true)
				//			{
				//				GetCWithMaxX(pipe[i].tool_orient.pos,pipe[i].machine_orient.variant[0].A,initial_C,result_C,result_pos);
				//				pipe[i].machine_orient.variant[0].pos=result_pos;
				//				pipe[i].machine_orient.variant[1].pos=result_pos;
				//				pipe[i].machine_orient.variant[0].C=result_C;
				//				pipe[i].machine_orient.variant[1].C=result_C;
				//			}else
				//			{
				//				TVec<T,3> pos0=ToMachineToolKinematics(pipe[i].tool_orient.pos,pipe[i].machine_orient.variant[0].A,initial_C);
				//				TVec<T,3> pos1=ToMachineToolKinematics(pipe[i].tool_orient.pos,pipe[i].machine_orient.variant[0].A,initial_C+step);
				//				TVec<T,3> pos2=ToMachineToolKinematics(pipe[i].tool_orient.pos,pipe[i].machine_orient.variant[0].A,initial_C-step);

				//				if(pos0[0]>pos1[0]&&pos0[0]>pos2[0])
				//				{
				//					pipe[i].machine_orient.variant[0].pos=pos0;
				//					pipe[i].machine_orient.variant[1].pos=pos0;
				//					pipe[i].machine_orient.variant[0].C=initial_C;
				//					pipe[i].machine_orient.variant[1].C=initial_C;
				//				}
				//				if(pos1[0]>pos0[0]&&pos1[0]>pos2[0])
				//				{
				//					pipe[i].machine_orient.variant[0].pos=pos1;
				//					pipe[i].machine_orient.variant[1].pos=pos1;
				//					pipe[i].machine_orient.variant[0].C=initial_C+step;
				//					pipe[i].machine_orient.variant[1].C=initial_C+step;
				//				}
				//				if(pos2[0]>pos0[0]&&pos2[0]>pos0[0])
				//				{
				//					pipe[i].machine_orient.variant[0].pos=pos2;
				//					pipe[i].machine_orient.variant[1].pos=pos2;
				//					pipe[i].machine_orient.variant[0].C=initial_C-step;
				//					pipe[i].machine_orient.variant[1].C=initial_C-step;
				//				}

				//			}
				//			//ToMachineToolKinematics(pipe[i].tool_orient.pos,pipe[i].tool_orient.dir,pipe[i].machine_orient,true,pipe[i-1].machine_orient.variant[0].C);
				//		}
			}break;
		case 2://критерий максимальной координаты по X (для ТФЦ-600)
			{
				
				//for(int i=1;i<pipe.size();i++)
				//	if(pipe[i].mask&PrimitiveMask::CIRCLE
				//		||pipe[i].mask&PrimitiveMask::LINE)
				//		if(pipe[i].machine_orient.any_C)
				//		{
				//			TKinematicsPair<T> pair;
				//			ToMachineToolKinematics(pipe[i].tool_orient.pos,pipe[i].tool_orient.pos.GetNormalized(),pair);//TODO а если позиция будет равна нулю?
				//			T result_C=pair.variant[0].C;
				//			TVec<T,3> pos=ToMachineToolKinematics(pipe[i].tool_orient.pos,pipe[i].machine_orient.variant[0].A,result_C);
				//			pipe[i].machine_orient.variant[0].pos=pos;
				//			pipe[i].machine_orient.variant[1].pos=pos;
				//			pipe[i].machine_orient.variant[0].C=result_C;
				//			pipe[i].machine_orient.variant[1].C=result_C;
				//		}
			}break;
		default:assert(false);
		}
	}

	void TraceLine(std::vector<TPipelineElement> &pipe,int line)
	{
		int curr_line=line;
		if(!pipe[0].machine_orient.valid[curr_line])curr_line=!line;
		T curr_A=AToMachineRange(pipe[0].machine_orient.variant[curr_line].A);
		T curr_C=CToMachineRange(pipe[0].machine_orient.variant[curr_line].C);

		for(int i=0;i<pipe.size();i++)
		{

			TKinematicsNode *curr=&pipe[i].machine_orient;

			switch(pipe[i].mask)
			{
			case PrimitiveMask::LINE:
				{
					if(i==pipe.size()-1)break;

					TKinematicsNode *next=&pipe[i+1].machine_orient;

					T delta_A,delta_C;

					//GetMovement(curr->variant[curr_line],next->variant[curr_line],delta_A,delta_C);
					IsCCWMove(curr_A,next->variant[curr_line].A,delta_A);
					IsCCWMove(curr_C,next->variant[curr_line].C,delta_C);

					curr->move_over_pole[line]=
						AIsMoveOverPole(curr_A,curr_A+delta_A)
						||CIsMoveOverPole(curr_C,curr_C+delta_C);

					curr->change_line[curr_line]=false;

					if(curr->move_over_pole[line]||(!next->valid[curr_line]))
					{
						if(curr->valid[!curr_line])
						{
							curr->change_line[curr_line]=true;
							T new_A=AToMachineRange(curr->variant[!curr_line].A);
							T new_C=CToMachineRange(curr->variant[!curr_line].C);

							curr->A_pole_change_delta[line]=new_A-curr_A;
							curr->C_pole_change_delta[line]=new_C-curr_C;

							curr_A+=curr->A_pole_change_delta[line];
							curr_C+=curr->C_pole_change_delta[line];

							curr_line=!curr_line;
							//GetMovement(curr->variant[curr_line],next->variant[curr_line],delta_A,delta_C);
							IsCCWMove(new_A,next->variant[curr_line].A,delta_A);
							IsCCWMove(new_C,next->variant[curr_line].C,delta_C);
						}else
						{
							curr->change_line[curr_line]=false;
							//TODO обработка случая когда сменить полюс нельза а надо раскручиваться по C
							T new_A=AToMachineRange(curr->variant[curr_line].A);
							T new_C=CToMachineRange(curr->variant[curr_line].C);

							curr->A_pole_change_delta[line]=new_A-curr_A;
							curr->C_pole_change_delta[line]=new_C-curr_C;

							curr_A+=curr->A_pole_change_delta[line];
							curr_C+=curr->C_pole_change_delta[line];

							IsCCWMove(new_A,curr->variant[curr_line].A,delta_A);
							IsCCWMove(new_C,curr->variant[curr_line].C,delta_C);
						}

						//delta_A+=new_A-curr_A;
						//delta_C+=new_C-curr_C;
						//
						if(!next->valid[curr_line])
							throw string("Kinematics hasn't valid variant");
					}

					curr->A_inc[line]=delta_A;
					curr->C_inc[line]=delta_C;

					assert(!(AIsMoveOverPole(curr_A,curr_A+delta_A)||CIsMoveOverPole(curr_C,curr_C+delta_C)));
					curr_A+=delta_A;
					curr_C+=delta_C;
					assert(!(AIsInPole(curr_A)||CIsInPole(curr_C)));
				}break;
			case PrimitiveMask::CIRCLE:
				{
				}break;
			default:assert(false);
			}
		}
	}

	void StandartRetractEngageChangePole(int i,int curr_line,int line,T curr_A,T curr_C,std::vector<TPipelineElement> &pipe,std::vector<TToolMovementElement<T>> &result_pipe)
	{

		//TVec<T,3> retract_pos=pipe[i].machine_orient.variant[curr_line].pos+pipe[i].tool_orient.dir*pole_change_height;

		TVec<T,3> retract_pos=ToMachineToolKinematics(pipe[i].tool_orient.pos+pipe[i].tool_orient.dir*pole_change_height,
			pipe[i].machine_orient.variant[curr_line].A,
			pipe[i].machine_orient.variant[curr_line].C);

		TToolMovementElement<T> t;
		t.tool_orient.dir=pipe[i].tool_orient.dir;
		t.tool_orient.pos=pipe[i].tool_orient.pos+pipe[i].tool_orient.dir*pole_change_height;
		*(TMachineState<T>*)&t=pipe[i];
		t.A=curr_A;
		t.C=curr_C;
		t.pos=retract_pos;
		t.rapid=true;
		result_pipe.push_back(t);

		curr_A+=pipe[i].machine_orient.A_pole_change_delta[line];
		curr_C+=pipe[i].machine_orient.C_pole_change_delta[line];

		//TVec<T,3> t0=pipe[i].machine_orient.variant[!curr_line].pos+pipe[i].tool_orient.dir*pole_change_height;

		TVec<T,3> t0=ToMachineToolKinematics(pipe[i].tool_orient.pos+pipe[i].tool_orient.dir*pole_change_height,
			pipe[i].machine_orient.variant[!curr_line].A,
			pipe[i].machine_orient.variant[!curr_line].C);

		t.tool_orient.dir=pipe[i].tool_orient.dir;
		t.tool_orient.pos=pipe[i].tool_orient.pos+pipe[i].tool_orient.dir*pole_change_height;
		*(TMachineState<T>*)&t=pipe[i];
		t.A=curr_A;
		t.C=curr_C;
		t.pos=t0;
		t.rapid=true;
		result_pipe.push_back(t);

		t0=pipe[i].machine_orient.variant[!curr_line].pos;
		t.tool_orient=pipe[i].tool_orient;
		*(TMachineState<T>*)&t=pipe[i];
		t.A=curr_A;
		t.C=curr_C;
		t.pos=t0;
		t.rapid=true;
		result_pipe.push_back(t);

		//TVec<T,3> t1=pipe[i].machine_orient.variant[!curr_line].pos+pipe[i].tool_orient.dir*pole_change_engage_height;
		//t.tool_orient=pipe[i].tool_orient;
		//*(TMachineState<T>*)&t=pipe[i];
		//t.A=curr_A;
		//t.C=curr_C;
		//t.pos=t1;
		//t.rapid=true;
		//result_pipe.push_back(t);

		//TVec<T,3> t2=pipe[i].machine_orient.variant[!curr_line].pos;
		//t.tool_orient=pipe[i].tool_orient;
		//*(TMachineState<T>*)&t=pipe[i];
		//t.A=curr_A;
		//t.C=curr_C;
		//t.pos=t2;
		//result_pipe.push_back(t);
	}

	void SelectBestLine(std::vector<TPipelineElement> &pipe,std::vector<TToolMovementElement<T>> &result_pipe)
	{
		int line=0;
		int curr_line=line;
		if(!pipe[0].machine_orient.valid[curr_line])curr_line=!line;
		T curr_A=AToMachineRange(pipe[0].machine_orient.variant[curr_line].A);
		T curr_C=CToMachineRange(pipe[0].machine_orient.variant[curr_line].C);
		for(int i=0;i<pipe.size();i++)
		{
			TKinematicsNode *curr=&pipe[i].machine_orient;
			switch(pipe[i].mask)
			{
			case PrimitiveMask::LINE:
				{
					TToolMovementElement<T> t;
					t.tool_orient=pipe[i].tool_orient;
					*(TMachineState<T>*)&t=pipe[i];
					*(TKinematics<T>*)&t=pipe[i].machine_orient.variant[curr_line];
					t.A=curr_A;
					t.C=curr_C;
					t.any_C=pipe[i].machine_orient.any_C;

					result_pipe.push_back(t);

					if(i==pipe.size()-1)break;
					TKinematicsNode *next=&pipe[i+1].machine_orient;
					if(curr->move_over_pole[line]||(!next->valid[curr_line]))
					{
						StandartRetractEngageChangePole(i,curr_line,line,curr_A,curr_C,pipe,result_pipe);

						curr_A+=curr->A_pole_change_delta[line];
						curr_C+=curr->C_pole_change_delta[line];
						if(curr->change_line[curr_line])
							curr_line=!curr_line;
					}
					curr_A+=curr->A_inc[line];
					curr_C+=curr->C_inc[line];
				}break;
			case PrimitiveMask::CIRCLE:
				{
					TToolMovementElement<T> t;
					t.tool_orient=pipe[i].tool_orient;
					*(TMachineState<T>*)&t=pipe[i];
					*(TKinematics<T>*)&t=pipe[i].machine_orient.variant[curr_line];
					t.A=curr_A;
					t.C=curr_C;
					result_pipe.push_back(t);
				}break;
			default:assert(false);
			}

		}
	}

	void GetGCode(std::vector<TToolMovementElement<T>> &pipe,std::string& result_code)
	{
		result_code="";
		if(pipe.size()==0)return;
		char* buff=new char[10000000];//TODO сюда записывается текст из auxfun котоорый может быть очень болььшим
		T curr_feed=0/*pipe[0].feed*/;
		int curr_cutcom=0;
		string curr_path_name;
		T curr_spndl_rpm=pipe[0].spndl_rpm;
		bool curr_clw=pipe[0].clw;
		//if(curr_feed==0)curr_feed=1500;//TODO
		int frame_number=1;

		bool oriented_from_goto_engage_done=false;
		
		T curr_coord[5];//текущие координаты
		T curr_coord_machine[5];//текущие координаты с учетом округления(используется при инкрементном режиме для устранения накапливающейся ошбки округления)
		bool need_print[5]={0,0,0,0,0};

		T* new_coord=&pipe[0].pos[0];
		for(int c=0;c<5;c++)
		{
			curr_coord[c]=new_coord[c]+2*gcode_axis_prop[c].repeat_tol;
			curr_coord_machine[c]=
				gcode_axis_prop[c].rad_to_deg
				?RadToDeg(new_coord[c])
				:new_coord[c];
		}

		for(int i=0;i<pipe.size();i++)
		{
			new_coord=&pipe[i].pos[0];
			//switch(pipe[i].mask)
			{
			//case PrimitiveMask::LINE:
			//	{
				bool is_path_begin=(curr_path_name!=pipe[i].path_name&&pipe[i].path_name!="");
				if(is_path_begin)
				{
					oriented_from_goto_engage_done=false;
					oriented_from_goto_engage_done=false;
				}

				curr_path_name=pipe[i].path_name;

				int temp_id=-1;
				for(int t=0;t<oriented_from_goto.size();t++)
				{
					if(oriented_from_goto[t].path_name==(curr_path_name))
					{
						temp_id=t;
						break;
					}
				}
				
				bool is_oriented_from_goto_engage_start=temp_id!=-1&&!oriented_from_goto_engage_done&&(((i==0||is_path_begin)&&pipe[i].color==186)||(i!=0&&pipe[i-1].color!=186&&pipe[i].color==186));
				bool is_oriented_from_goto_engage_end=temp_id!=-1&&!oriented_from_goto_engage_done&&(pipe[i].color==186&&pipe[i+1].color!=186)&&curr_path_name==pipe[i+1].path_name;

				bool is_oriented_from_goto_retract_start=
					temp_id!=-1&&
					oriented_from_goto_engage_done&&
					((i==0&&pipe[i].color==186)||(i!=0&&pipe[i-1].color!=186&&pipe[i].color==186));

				bool is_oriented_from_goto_retract_end=
					temp_id!=-1&&
					oriented_from_goto_engage_done&&
					((i!=pipe.size()&&pipe[i].color==186&&pipe[i+1].color!=186)||(i==pipe.size()&&pipe[i].color==186)||curr_path_name!=pipe[i+1].path_name);

				if(is_oriented_from_goto_engage_start)
				{
					sprintf(buff,"\nM5\nSPOS=%f;engage_start\n",oriented_from_goto[temp_id].orientation);
					result_code+=buff;
				}

				if(is_oriented_from_goto_retract_start)
				{
					sprintf(buff,"\nM5\nSPOS=%f;retract_start\n",oriented_from_goto[temp_id].orientation);
					result_code+=buff;
				}

				//if(oriented_from_goto&&!oriented_from_goto_engage_done)
				//{
				//	if((i==0&&pipe[i].color==186)||(pipe[i-1].color!=186&&pipe[i].color==186))//спец комманда для подхода и отхода
				//	{
				//		sprintf(buff,"\nSPOS=%f\n",oriented_from_goto_orientation);
				//		result_code+=buff;
				//		oriented_from_goto_engage_done=true;
				//	}
				//}

					if(!pipe[i].rapid&&pipe[i].feed!=curr_feed)
					{
						//sprintf(buff,"N%i G01 F%i\n",frame_number+=10,(int)pipe[i].feed);
						//result_code+=buff;
					}

					if((1/*coord_repeat_tol*/<abs(pipe[i].spndl_rpm-curr_spndl_rpm)||curr_clw!=pipe[i].clw||i==0)
						&&temp_id==-1)//если is_oriented_from_goto_engage то задается в подходе отходе
					{
						if(abs(curr_spndl_rpm-0)<0.001||i==0)
							sprintf(buff,"M3S%i\n",/*pipe[i].clw?3:4,*/int(pipe[i].spndl_rpm));
						else if(abs(pipe[i].spndl_rpm-0)<0.001)
							sprintf(buff,"M5\n");
						else sprintf(buff,"");
						result_code+=buff;
						curr_spndl_rpm=pipe[i].spndl_rpm;
						curr_clw=pipe[i].clw;
					}

					bool force_rapid_change=false;
					for(int c=0;c<5;c++)
					{
						bool last_need_print=need_print[c];
						T tol=gcode_axis_prop[c].rad_to_deg?DegToRad(gcode_axis_prop[c].repeat_tol):gcode_axis_prop[c].repeat_tol;
						need_print[c]=
							tol<=abs(new_coord[c]-curr_coord[c])||!gcode_axis_prop[c].remove_repeat;
						if(need_print[c]&&gcode_axis_prop[c].force_rapid_change)
							force_rapid_change=true;
						//если имеется функция разблокировки/блокировки оси то включаем ее
						if(!last_need_print&&need_print[c]&&gcode_axis_prop[c].lock_header!="")
						{
							result_code+=gcode_axis_prop[c].lock_header;
						}
						if(last_need_print&&!need_print[c]&&gcode_axis_prop[c].lock_header!="")
						{
							result_code+=gcode_axis_prop[c].lock_footer;
						}
					}

					sprintf(buff,"%s",pipe[i].auxfun.c_str());
					result_code+=buff;
					//если направление инструмента 0,0,-1 то надо сменить коррекцию на противоположную
					if(pipe[i].tool_orient.dir.Distance(TVec<T,3>(0,0,-1))<0.0001)
					{
						if(pipe[i].cutcom==1)pipe[i].cutcom=2;
						if(pipe[i].cutcom==2)pipe[i].cutcom=1;
					}
					if(pipe[i].cutcom!=curr_cutcom)
					{	
						sprintf(buff,"G%i",40+pipe[i].cutcom);
						result_code+=buff;
						curr_cutcom=pipe[i].cutcom;
					}
					//плоскость круговой/спиральной интерполяции
					if(pipe[i].mask==PrimitiveMask::CIRCLE)
					{
						int rot_axis;
						bool positive;
						IsOrthogonalVector(pipe[i].normal,rot_axis,positive);
						int rot_axis_to_plane[3]={19,18,17};
						sprintf(buff,"G%i",rot_axis_to_plane[rot_axis]);
						result_code+=buff;
					}

					{
						int g_move_mode=-1;
						if(pipe[i].mask==PrimitiveMask::CIRCLE)
						{
							g_move_mode=(pipe[i].normal*TVec<T,3>(1,1,1).GetNormalized()>0)?2:3;
						}else
						{
							g_move_mode=(pipe[i].rapid||force_rapid_change)?0:1;
						}
						sprintf(buff,/*"N%i"*/"G0%i "/*,frame_number+=10*/,g_move_mode);
						result_code+=buff;
					}

					//TODO ВАЖНООООО!!!! в настройках ини файла неправильно трактуются X_id=0
					//Y_id=1
					//Z_id=2
					//A_id=4
					//C_id=3
					// почему-то перепутываются оси при измененном порядке

					for(int c=0;c<5;c++)
					{
						if(need_print[c])
						{
							T new_coord_val=
								gcode_axis_prop[c].rad_to_deg
								?RadToDeg(new_coord[c])
								:new_coord[c];

							if(gcode_axis_prop[c].is_increment)
							{
								T inc_val=new_coord_val-curr_coord_machine[c];
								sprintf(buff,gcode_axis_prop[c].format.c_str(),inc_val);//TODO из-за округления будет накапливаться ошибка DONE
								inc_val=floor(inc_val/gcode_axis_prop[c].repeat_tol+0.5)*gcode_axis_prop[c].repeat_tol;

								curr_coord_machine[c]+=inc_val;
							}
							else
								sprintf(buff,gcode_axis_prop[c].format.c_str(),new_coord_val);
							result_code+=buff;
							curr_coord[c]=new_coord[c];
						}
					}
					if(pipe[i].mask==PrimitiveMask::CIRCLE)
					{
						TVec<T,3> center=pipe[i].center;
						TVec<T,3> machine_center=ToMachineToolKinematics(center,pipe[i].A,pipe[i].C);
						TVec<T,3> machine_last_pos=ToMachineToolKinematics(pipe[i].pos,pipe[i].A,pipe[i].C);

						int rot_axis;
						bool positive;
						IsOrthogonalVector(pipe[i].normal,rot_axis,positive);

						//печатаем координаты центра кроме координаты оси вращения
						for(int a=0;a<3;a++)
						{
							if(circle_interpolation_center_absol)
							{
								char* r[]={
									"I=AC(%f)","J=AC(%f)","K=AC(%f)"
								};
								if(rot_axis!=a)
								{
									sprintf(buff,r[a],machine_center[a]);
									result_code+=buff;
								}
							}else
							{
								char* r[]={
									"I%f","J%f","K%f"
								};
								if(rot_axis!=a)
								{
									sprintf(buff,r[a],machine_center[a]-machine_last_pos[a]);
									result_code+=buff;
								}
							}
						}

						if(pipe[i].spiral_times>0)
						{
							sprintf(buff," TURN=%i ",pipe[i].spiral_times);
							result_code+=buff;
						}
					}
				
					if(1/*coord_repeat_tol*/<abs(pipe[i].contour_correct_feed-curr_feed)||!remove_F_repeat)
					{
						if(!pipe[i].rapid)
						{
							sprintf(buff,"F%i",pipe[i].rapid?(int)rapid_feed:(int)pipe[i].contour_correct_feed);
							result_code+=buff;
							curr_feed=pipe[i].feed;
						}
					}

					result_code+="\n";

					if(is_oriented_from_goto_engage_end)
					{
						sprintf(buff,"\nM3S%i;engage end\n",int(pipe[i].spndl_rpm));
						result_code+=buff;
						oriented_from_goto_engage_done=true;
					}

					if(is_oriented_from_goto_retract_end)
					{
						sprintf(buff,"\nM3S%i;retract end\n",int(pipe[i].spndl_rpm));
						result_code+=buff;
						oriented_from_goto_engage_done=false;
					}

					//if(oriented_from_goto&&oriented_from_goto_engage_done)
					//{
					//	if((i==0&&pipe[i].color==186)||(pipe[i-1].color!=186&&pipe[i].color==186))//спец комманда для подхода и отхода
					//	{
					//		sprintf(buff,"\nM5\nSPOS=%f\n",oriented_from_goto_orientation);
					//		result_code+=buff;
					//		oriented_from_goto_engage_done=true;
					//	}
					//}

				//}break;
			//case PrimitiveMask::CIRCLE:
				/*{
					TVec<T,3> prev_pos=pipe[i-1].pos,
						next_pos=pipe[i].pos,
						center;
					if(circle_interpolation_center_absol)
						center=prev_pos+pipe[i].center-pipe[i-1].tool_orient.pos;
					else
						center=pipe[i].center-pipe[i-1].tool_orient.pos;
					sprintf(buff,"%sN%i G0%i X%.3f Y%.3f Z%.3f I%.3f J%.3f\n",
						pipe[i].auxfun.c_str(),
						frame_number+=10,
						pipe[i].normal[2]<0?3:2,
							next_pos[0],next_pos[1],next_pos[2],
							center[0],center[1],center[2]);
						result_code+=buff;

				}break;*/
			//default:assert(false);
			}
		}
		size_t start=0;
		int curr_frame=0;
		do{
			size_t curr_pos=result_code.find('\n',start);
			if(curr_pos==string::npos)break;
			char buff[100];
			sprintf(buff,"\nN%i ",curr_frame);
			curr_frame+=10;
			result_code.replace(curr_pos,1,buff);
			start=curr_pos+strlen(buff);
		}while(true);
		delete buff;
	}

	
	T MovementDistance(TKinematics<T> k0,TKinematics<T> k1)//в отличие от предыдущей версии считает перемещение по всем координатам в линейном виде
	{
		return sqrt(k0.pos.SqrDistance(k1.pos)+sqr(k0.A-k1.A)+sqr(k0.C-k1.C));
	}



	void CalculateMoveTime(std::vector<TToolMovementElement<T>> &pipe,double &fast_movement_time,double &work_movement_time)
	{
		fast_movement_time=0;
		work_movement_time=0;

		pipe[0].move_distance=0;
		pipe[0].move_time=0;
		for(int i=1;i<=pipe.size()-1;i++)
		{

			switch(pipe[i].mask)
			{
			case PrimitiveMask::LINE:
				{
					pipe[i].move_distance=MovementDistance(pipe[i-1],pipe[i]);
				}break;
			case PrimitiveMask::CIRCLE:
				{
					TVec<T,3> lx,ly,lz;
					TVec<T,3> center=pipe[i].center;
					T rad=pipe[i].radius;

					lx=(pipe[i-1].tool_orient.pos-center).GetNormalized();
					lz=pipe[i].normal;
					ly=lz.Cross(lx);
					
					pipe[i].move_distance=rad*abs(AngleFromDir(
						TVec<T,2>(
						(pipe[i].tool_orient.pos-center)*lx,
						(pipe[i].tool_orient.pos-center)*ly).GetNormalized()))+(pipe[i].spiral_times!=-1?2*M_PI*pipe[i].spiral_times:0);
				}break;
			default:assert(false);
			}
			pipe[i].move_time=pipe[i].move_distance/(pipe[i].rapid?rapid_feed:pipe[i].feed)*60.0;
			if(pipe[i].rapid)
				fast_movement_time+=pipe[i].move_time;
			else
				work_movement_time+=pipe[i].move_time;
		}
	}

	void CalculateContourSpeed(std::vector<TToolMovementElement<T>> &pipe)
	{
		pipe[0].contour_correct_feed=pipe[0].feed;
		for(int i=1;i<=pipe.size()-1;i++)
		{
			T dist=pipe[i].tool_orient.pos.Distance(pipe[i-1].tool_orient.pos);
			
			/*pipe[i].contour_correct_feed=ClampMax(contour_max_feed,
				pipe[i].feed*
				(dist<ortho_vec_epsilon?(1.0):(pipe[i].move_distance/dist)));*/
			pipe[i].contour_correct_feed=pipe[i].feed;
		}
	}

	bool MergeFrames(std::vector<TToolMovementElement<T>> &pipe,int window_start,int window_end)
		//return - произошло удаление элемента
	{
		int min_k=-1;
		T min_time=0;
		for(int k=window_start+1;k<=window_end-1;k++)
		{
			T temp=pipe[k].move_time+pipe[k+1].move_time;
			if((min_k==-1||temp<min_time)&&!pipe[k].base_element)
			{
				min_time=temp;
				min_k=k;
			}
		}
		if(min_k!=-1)
		{
			pipe[min_k+1].move_time+=pipe[min_k].move_time;
			pipe.Delete(min_k);
			return true;
		}else return false;
	}

	void Compress(std::vector<TToolMovementElement<T>> &pipe)
	{
		int i=0;
		while(true)
		{
			while(pipe[i].rapid&&i<pipe.size()-1)i++;
			if(i==pipe.size()-1)break;
			T time=0;
			int window_end=i;
			while(window_end-i<frames_on_1sec_max&&window_end<pipe.size()-1)
			{
				time+=pipe[window_end].move_time;
				window_end++;
			}
			while(window_end<=pipe.size()-1)
			{
				if(time<1.0&&MergeFrames(pipe,i,window_end))
				{
					if(window_end>pipe.size()-1)return;
					time+=pipe[window_end].move_time;//если было удаление кадр то на этом месте теперь новый кадр
				}else
				{
					time-=pipe[i].move_time;
					i++;
					window_end++;
					if(window_end>pipe.size()-1)break;
					time+=pipe[window_end].move_time;
				}
			}
		}
	}

	void TryAlignInvalidVariants(std::vector<TPipelineElement> &pipe)
	{
		for(int i=0;i<=pipe.size()-1;i++)
			if(!pipe[i].machine_orient.any_C)
			{
				for(int k=0;k<2;k++)
					if((!pipe[i].machine_orient.valid[k])&&(!AIsInPole(pipe[i].machine_orient.variant[k].A)))
					{
						try{
							pipe[i].machine_orient.variant[k].C=CToMachineRange(pipe[i].machine_orient.variant[k].C);
							pipe[i].machine_orient.valid[k]=!AIsInPole(pipe[i].machine_orient.variant[k].A);
						}catch(exception e)
						{}
						break;
					}
			}
	}

	void Subdivide(std::vector<TToolMovementElement<T>> &pipe)
	{
		std::vector<TVec<int,2>> count_after_pair;
		count_after_pair.reserve(pipe.size()/3);
		for(int i=0;i<pipe.size()-1;i++)
		{
			if(pipe[i].mask==PrimitiveMask::LINE)
			{
				T tol=1000000;
				//while(tol>tolerance)
				{
					TToolMovementElement<T> new_el;
					tol=GetInterpolationTolerance(pipe[i],pipe[i+1],new_el.tool_orient);
					bool need_subdivide=false;
					if(pipe[i].rapid)
						need_subdivide=tol>rapid_tolerance;
					else need_subdivide=tol>tolerance;
					if(need_subdivide&&((subdivide_only_any_C==pipe[i].any_C)||!subdivide_only_any_C))
					{		
						int new_el_high=(pipe[i].rapid?(tol/rapid_tolerance):(tol/tolerance))+1;
						if(new_el_high>1)
							count_after_pair.push_back(TVec<int,2>((new_el_high+1)-2,i));
						//(new_el_high+1)-2 уменьшается на 2 т.к. начальная и конечная точка уже имеется и надо добавить только промежуточные
					}
				}
			}
		}
		for (int i = 0; i < count_after_pair.size(); i++)
			pipe.push_back(TToolMovementElement<T>());
		int off=0,curr_pair=0;
		for(int i=0;i< pipe.size() - 1;i++)
		{
			if(curr_pair>count_after_pair.size()-1)break;
			switch(pipe[i].mask)
			{
			case PrimitiveMask::LINE:
				if(count_after_pair[curr_pair][1]+off==i)
				{
					int new_el_high=count_after_pair[curr_pair][0]+1;
					
					int count_inserted=count_after_pair[curr_pair][0];
					off+=count_inserted;


					TVec<T,3> p0,p1,d0,d1;
					T a0,a1,c0,c1;
					p0=pipe[i].tool_orient.pos;
					p1=pipe[i+1+count_inserted].tool_orient.pos;
					d0=pipe[i].tool_orient.dir.GetNormalized();
					d1=pipe[i+1+count_inserted].tool_orient.dir.GetNormalized();
					c0=pipe[i].C;
					c1=pipe[i+1+count_inserted].C;
					a0=pipe[i].A;
					a1=pipe[i+1+count_inserted].A;
					//pipe.InsertCount((new_el_high+1)-2,i);
					for(int t=1;t<new_el_high;t++)
					{
						TToolMovementElement<T> new_el;
						new_el.tool_orient.pos=p0*(new_el_high-t)/new_el_high+p1*t/new_el_high;
						new_el.feed=pipe[i].feed;
						new_el.mask=pipe[i].mask;
						new_el.rapid=pipe[i].rapid;
						new_el.color=pipe[i].color;
						new_el.cutcom=pipe[i].cutcom;
						new_el.base_element=false;
						new_el.A=a0*(new_el_high-t)/new_el_high+a1*t/new_el_high;
						new_el.C=c0*(new_el_high-t)/new_el_high+c1*t/new_el_high;
						new_el.tool_orient.dir=GetToolDirFromMachineToolKinematics(new_el.A,new_el.C);
						new_el.pos=ToMachineToolKinematics(new_el.tool_orient.pos,new_el.A,new_el.C);
						pipe[i+t]=new_el;
					}
					i+=new_el_high-1;

					curr_pair++;
				}break;
			case PrimitiveMask::CIRCLE:
				{
				}break;
			default:assert(false);
			}
		}
	}

	void ValidateKinematicsInverseAlgorithm(std::vector<TPipelineElement> &pipe)
	{
		T max_pos_tol=0;
		T max_dir_tol=0;

		for(int i=0;i<pipe.size()-1;i++)
			if(pipe[i].mask&PrimitiveMask::CIRCLE
				||pipe[i].mask&PrimitiveMask::LINE)
		{
			TVec<T,3> pos,dir;
			for(int k=0;k<(pipe[i].machine_orient.any_C?0:2);k++)
			{
				FromMachineToolKinematics(pipe[i].machine_orient.variant[k],pos,dir);
				T tol0=pipe[i].tool_orient.dir.Distance(dir);
				T tol1=pipe[i].tool_orient.pos.Distance(pos);

				if(tol0>max_dir_tol)max_dir_tol=tol0;
				if(tol1>max_pos_tol)max_pos_tol=tol1;

				//if(tol0>inverse_kinemtatics_tol||
				//	tol1>inverse_kinemtatics_tol)
				//	assert(false);
			}
		}
		printf("pos_tol = %.9lf \tdir_tol = %.9lf\n",max_pos_tol,max_dir_tol);
	}

	void MakePipe(std::vector<TPipelineElement> &pipe,std::vector<TCLSFToken> &atp_tokens)
	{
		pipe.reserve(atp_tokens.size());
		TVec<T,3> curr_dir(0,0,1);
		T curr_feed=100;
		T curr_spndl_rpm=0;
		int curr_cutcom=0;//0-off 1-left 2-right
		bool clw=true;
		string curr_aux;
		string curr_path_cs_name;
		string curr_path_name;
		bool rapid=false;
		int curr_color=0;
		if(!use_tool_length_correction)tool_length=0;
		for(int i=0;i<atp_tokens.size();i++)
		{
			try{
				if(
					atp_tokens[i].Name()=="GOTO"||
					atp_tokens[i].Name() =="FROM"||
					atp_tokens[i].Name() =="GOHOME")
				{
					TVec<T,3> pos,dir;
					if(atp_tokens[i].ParamsCount()!=3&&atp_tokens[i].ParamsCount() !=6)
						throw string("Not enough parameters in GOTO");
					for(int k=0;k<3;k++)pos[k]=atof(atp_tokens[i][k].c_str());
					if(atp_tokens[i].ParamsCount()<=3)
						dir=curr_dir;
					else
					{
						for(int k=0;k<3;k++)dir[k]= atof(atp_tokens[i][k+3].c_str());
						curr_dir=dir;
					}
					TPipelineElement t;
					t.auxfun=curr_aux;
					t.tool_orient.pos=pos;
					t.tool_orient.dir=dir;
					t.mask=PrimitiveMask::LINE;
					t.feed=curr_feed;
					t.spndl_rpm=curr_spndl_rpm;
					t.clw=clw;
					if(atp_tokens[i].Name() =="GOHOME")rapid=true;
					if(atp_tokens[i].Name() =="FROM")rapid=true;
					t.rapid=rapid;
					t.base_element=true;
					t.color=curr_color;
					t.cutcom=curr_cutcom;
					t.path_name=curr_path_name;
					pipe.push_back(t);
					if(rapid)rapid=false;
					curr_aux="";
				}
				else if(atp_tokens[i].Name() =="CIRCLE")
				{
					TVec<T,3> center,normal;
					T radius;
					TPipelineElement t;
					t.spiral_times=-1;
					if(atp_tokens[i].ParamsCount()<11)
						throw string("Not enough parameters in CIRCLE");
					for(int k=0;k<3;k++)center[k]=atof(atp_tokens[i][k].c_str());
					for(int k=0;k<3;k++)normal[k]= atof(atp_tokens[i][k+3].c_str());
					if(atp_tokens[i].ParamsCount()==13&&atp_tokens[i][11]=="TIMES")
					{
						t.spiral_times= atof(atp_tokens[i][12].c_str());
					}
					radius=atof(atp_tokens[i][6].c_str());

					t.auxfun=curr_aux;
					t.tool_orient.dir=curr_dir;
					t.center=center;
					t.normal=normal;
					t.radius=radius;
					t.mask=PrimitiveMask::CIRCLE;
					t.feed=curr_feed;
					t.spndl_rpm=curr_spndl_rpm;
					t.clw=clw;
					t.rapid=rapid;
					t.base_element=true;
					t.color=curr_color;
					t.cutcom=curr_cutcom;
					t.path_name=curr_path_name;
					pipe.push_back(t);
					curr_aux="";
				}else if(atp_tokens[i].Name()=="FEDRAT")
				{
					if(atp_tokens[i].ParamsCount()==1)
						curr_feed= atof(atp_tokens[i][0].c_str());
					else
						curr_feed= atof(atp_tokens[i][1].c_str());
					rapid=false;
				}else if(atp_tokens[i].Name()=="SPINDL")
				{
					if(atp_tokens[i].ParamsCount()==3)//TODO при смене инструмента у нас сбрасываются обороты шпинделя
					{
						curr_spndl_rpm= atof(atp_tokens[i][1].c_str());
						clw=(atp_tokens[i][2]=="CLW ");//TODO убирать пробельные символы
					}
				}
				else if(atp_tokens[i].Name()=="PAINT")
				{
					if(atp_tokens[i].ParamsCount()==2&&(atp_tokens[i][0]=="COLOR"))
						curr_color= atof(atp_tokens[i][1].c_str());
				}
				else if(atp_tokens[i].Name()=="RAPID")
				{
					rapid=true;
				}
				else if(atp_tokens[i].Name()=="TLDATA")
				{
					if(atp_tokens[i][0]=="MILL")
						tool_length= atof(atp_tokens[i][3].c_str());
					else if(atp_tokens[i][0]=="DRILL")
						tool_length= atof(atp_tokens[i][4].c_str());
					else if(atp_tokens[i][0]=="TCUTTER")
						tool_length= atof(atp_tokens[i][4].c_str());
					else throw string("unknown tool type");
					if(!use_tool_length_correction)tool_length=0;
				}
				else if(atp_tokens[i].Name()=="AUXFUN")
				{
					if(atp_tokens[i][0]=="0")//ф-я вывода текста
					{
						curr_aux+="\n";
						for(int k=1;k<atp_tokens[i].ParamsCount();k++)
						{
							curr_aux+=atp_tokens[i][k];
							if(k!=atp_tokens[i].ParamsCount()-1)
								curr_aux+=',';
						}
					}if(atp_tokens[i][0]=="1")//включение спец ф-и oriented_from_goto
					{
						//oriented_from_goto=true;
						//oriented_from_goto_orientation=lexical_cast<double>(atp_tokens[i][1].c_str());
						TOrientedFromGoto t;
						t.path_name=curr_path_name;
						t.orientation=atof(atp_tokens[i][1].c_str());
						oriented_from_goto.push_back(t);
					}
					
				}
				else if(atp_tokens[i].Name()=="CUTCOM")
				{
					if(atp_tokens[i][0]=="LEFT")
					{
						curr_cutcom=1;
					}else if(atp_tokens[i][0]=="RIGHT")
					{
						curr_cutcom=2;
					}else if(atp_tokens[i][0]=="OFF")
					{
						curr_cutcom=0;
					}
				}
				else if(atp_tokens[i].Name()=="NX_PROCESSOR_SET_CS_G")
				{
					local_CS_G_index=atoi(atp_tokens[i][0].c_str());
				}
				else if(atp_tokens[i].Name()=="NX_PROCESSOR_PATH_CS_NAME")
				{
					curr_path_cs_name=atp_tokens[i][0].c_str();
				}
				else if(atp_tokens[i].Name()=="MSYS")
				{
					int local_g_id=54;
					if(curr_path_cs_name=="")
					{
					}
					else
					{
						char* set_cs_template_start(".G");
						char* set_cs_template_end(".");
						size_t found=curr_path_cs_name.find(set_cs_template_start);
						if(found!=string::npos)
						{
							size_t found_last=curr_path_cs_name.find(set_cs_template_end,found+1);
							if(found_last>found)
							{
								string str_id=curr_path_cs_name.substr(found+strlen(set_cs_template_start),found_last-(found+strlen(set_cs_template_start)));
								local_g_id=atoi(str_id.c_str());
							}
						}
					}
					char buf[256];
					if(local_g_id>600)
						sprintf(buf,"\nzeros(%i)\n",local_g_id);
					else
						sprintf(buf,"\nG%i\n",local_g_id);
					curr_aux+=buf;
				}
				else if(atp_tokens[i].Name()=="TOOL PATH")
				{
					curr_path_name=atp_tokens[i][0].c_str();
					tool_name=atp_tokens[i][2].c_str();
				}
				else if(atp_tokens[i].Name()=="END-OF-PATH")
				{
					curr_path_name="";
				}
			//}catch(bad_cast& e)
			//{
			//	cout << (boost::format("ATP parser Error in line %i: %s\n")%i%e.what()).str();
			}
			catch(string& e)
			{
				//cout << (boost::format("ATP parser Error in line %i: %s\n")%i%e).str();
			}
		}
	}

	void PassThrough(std::vector<TCLSFToken> &atp_tokens,std::vector<TToolMovementElement<T>> &result_pipe,double &fast_movement_time,double &work_movement_time)
	{
		//using namespace boost::posix_time;
		//using namespace boost::gregorian;

		//ptime now;
		//ptime done_time;

		//now = microsec_clock::local_time();

		std::vector<TPipelineElement> pipe;



		MakePipe(pipe, atp_tokens);
		
		if(pipe.size()==0)return;

		//std::vector<TToolMovementElement<T>> result_pipe;
		bool need_rebuild=false;
		int max_rebuilds=1;
		int rebuild=0;

		Clear(pipe);

		CalcCircleParameters(pipe);
		CalcKinematics(pipe);
		ValidateKinematicsInverseAlgorithm(pipe);
		//FindAnyCBeetweenKinematics(pipe);
		DetermineAnyCKinematics(pipe);
		ValidateKinematics(pipe);
		TryAlignInvalidVariants(pipe);

		TraceLine(pipe,0);
		//TraceLine(pipe,1);
		SelectBestLine(pipe,result_pipe);
		if(use_subdivision)
			Subdivide(result_pipe);//TODO для ТФЦ600 при разбивке поворота по C нужное перемещение по дуге не совпадает с прямой между позициями поэтому скачет X

		CalculateMoveTime(result_pipe,fast_movement_time,work_movement_time);
		CalculateContourSpeed(result_pipe);
		//Compress(result_pipe); //пока что очень медленно

		//GetGCode(result_pipe,result_code);
		//InsertGCodeHead(result_code);

		//done_time = microsec_clock::local_time();
		//cout << " ("<< to_iso_string(done_time - now) <<")\n";
	}
	void GetCode(std::vector<TToolMovementElement<T>> &pipe,string &result_code,const char* ext_header,const char* prog_id)//только для standalone постов, далее убрать
	{
		//boost::format header_format(G_code_header.c_str());
		//header_format.exceptions( boost::io::all_error_bits ^ ( boost::io::too_many_args_bit | boost::io::too_few_args_bit )  );

		//boost::format footer_format(G_code_footer.c_str());
		//footer_format.exceptions( boost::io::all_error_bits ^ ( boost::io::too_many_args_bit | boost::io::too_few_args_bit )  );

		//GetGCode(pipe,result_code);
		//result_code=
		//	(
		//	header_format
		//	%local_CS_G_index
		//	%prog_id
		//	%head_name
		//	%tool_name
		//	).str()
		//	+
		//	result_code
		//	+
		//	(
		//	footer_format
		//	%prog_id
		//	).str();
		//result_code=G_code_header+result_code+G_code_footer;
		//InsertGCodeHead(result_code,TVec<T,3>(),"",false);
	}
	//void GetCode(std::vector<TToolMovementElement<T>> &pipe,string &result_code,
	//	TVec<T,3> use_offset, string use_machine_offset_string,bool use_machine_offset)//только для standalone постов, далее убрать
	//{
	//	GetGCode(pipe,result_code);
	//	result_code=(boost::format(G_code_header.c_str())%local_CS_G_index).str()+result_code+G_code_footer;
	//	//result_code=G_code_header+result_code+G_code_footer;
	//	//InsertGCodeHead(result_code,use_offset,use_machine_offset_string,use_machine_offset);
	//}
};
