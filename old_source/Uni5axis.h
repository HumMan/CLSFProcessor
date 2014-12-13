
#include "ATPConfiguration.h"
namespace PrimitiveMask
{
	enum Enum:unsigned int
	{
		LINE=1,
		CIRCLE=2,
	};
}

namespace TMovementMode
{
	enum Enum
	{
		Rapid,
		Linear,
		CircleCCW,
		CircleCW
	};
}

template<class T>
struct TToolOrientation
{
	TVec<T,3> pos,dir;
};

template<class T>
struct TKinematics
{
	TVec<T,3> pos;
	T 
		A,  //
		C;  //
	bool any_C;
};

template<class T>
struct TKinematicsPair
{
	TKinematics<T> variant[2];//варианты кинематики станка
	bool valid[2];			//вариант кинематики физически реализуем

	bool any_C;//ось А вертикальна -> значение C может быть любое
};

template<class T>
struct TMachineState
{
	PrimitiveMask::Enum mask;
	string auxfun;
	TVec<T,3> center,normal;
	T radius;
	int spiral_times;
	T feed;
	T spndl_rpm;
	bool clw;//TODO разобраться с парсингом а то глючит
	bool rapid;
	int color;
	bool base_element;//если true - элемент нельзя убрать при компрессии т.к. он исходный(существовавший до разбиения)
	int cutcom;
	string path_name;
};

template<class T>
struct TToolMovementElement:public TMachineState<T>,TKinematics<T>
{
	TToolOrientation<T> tool_orient;
	T move_time;
	T move_distance;
	T contour_correct_feed;
};

template<class T>
class TMovementNode
{
public:
	bool part_node;//иначе tool_node
	bool linear;//иначе поворотное звено
	TVec<T,3> axis_offset;
	TVec<T,3> axis;
	TVec<T,3> offset;//вектор смещения с направлением от текущего звена к следующему (привязан к текущему звену)

	TMovementNode(){}
	TMovementNode(bool use_part_node,bool use_linear,TVec<T,3> use_axis_offset,TVec<T,3> use_axis,TVec<T,3> use_offset=TVec<T,3>(0))
		:part_node(use_part_node)
		,linear(use_linear)
		,axis_offset(use_axis_offset)
		,axis(use_axis)
		,offset(use_offset){}
};

template<class T>
class TUniversal5axis:public TCommonProperties<T>
{
private:
	TMatrix<T,3> part_system;
	const static int nodes_count=5;//всегда такое для 5координатных станков
	TMovementNode<T> nodes[nodes_count];//0 - part  4 - tool
	TVec<T,3> Tool_dir;
	int X_id;
	int Y_id;
	int Z_id;
	int A_id;
	int C_id;
public:
	TUniversal5axis(std::map<string,string> &ini_params):TCommonProperties(ini_params)
	{
		//TODO добавить проверки правильности кинематик (1- количество линейных и угловых звеньев 2- ось инструмента должна лежать в плоскости поворота последней оси

		part_system=TMatrix<T,3>
			(
			ParseVec(ini_params["part_system0"]),
			ParseVec(ini_params["part_system1"]),
			ParseVec(ini_params["part_system2"])
			);
		//
		X_id=lexical_cast<int>(ini_params["X_id"]);
		Y_id=lexical_cast<int>(ini_params["Y_id"]);
		Z_id=lexical_cast<int>(ini_params["Z_id"]);
		A_id=lexical_cast<int>(ini_params["A_id"]);
		C_id=lexical_cast<int>(ini_params["C_id"]);

		for(int i=0;i<5;i++)
		{
			char buf[20];
			sprintf(buf,"node%i_part_node",i);
			nodes[i].part_node=lexical_cast<int>(ini_params[buf]);
			sprintf(buf,"node%i_linear",i);
			nodes[i].linear=lexical_cast<int>(ini_params[buf]);
			sprintf(buf,"node%i_axis_offset",i);
			nodes[i].axis_offset=ParseVec(ini_params[buf]);
			sprintf(buf,"node%i_axis",i);
			nodes[i].axis=ParseVec(ini_params[buf]);
			sprintf(buf,"node%i_offset",i);
			nodes[i].offset=ParseVec(ini_params[buf]);	
		}
		//tool
		Tool_dir=ParseVec(ini_params["Tool_dir"]);

	}

	TVec<T,3> ForwardLinearNode(TVec<T,3> p,TVec<T,3> axis, T value)
	{
		return p+axis*value;
	}
	TVec<T,3> InverseLinearNode(TVec<T,3> machine_p,TVec<T,3> axis, T value)
	{
		return machine_p-axis*value;
	}

	TVec<T,3> ForwardRotateNode(TVec<T,3> p,TVec<T,3> off,TVec<T,3> axis, T angle)
	{
		return (p-off).GetRotated(axis,angle)+off;
	}
	TVec<T,3> InverseRotateNode(TVec<T,3> machine_p,TVec<T,3> off,TVec<T,3> axis, T angle)
	{
		return (machine_p-off).GetRotated(axis,-angle)+off;
	}

	TVec<T,3> ForwardRotateNode(TVec<T,3> dir,TVec<T,3> axis, T angle)
	{
		return dir.GetRotated(axis,angle);
	}
	TVec<T,3> InverseRotateNode(TVec<T,3> machine_dir,TVec<T,3> axis, T angle)
	{
		return machine_dir.GetRotated(axis,-angle);
	}

	//a0,a1 - углы на которые надо повернуть вектор p вокруг оси curr чтобы он полностью оказался в плоскости перпенидкулярной next
	//result - false если такого поворота не существует
	//any_rotation - вектор curr лежит в плоскости перпенд. next и совпадает с вектором curr
	bool GetRotationToNextPlane(TVec<T,3> curr,TVec<T,3> next, TVec<T,3> p, T rotations[], bool &any_rotation)
	{
		//TODO надо математическки провверить а то что-то не сходится
		TVec<T,3> _z=curr;
		TVec<T,3> _y=curr.Cross(next).GetNormalized();
		TVec<T,3> _x=_y.Cross(curr);

		T gamma = TVec<T,3>::AngleBetween(next,_x);
		TVec<T,2> d(p*_x,p*_y);
		if(d.SqrLength()<sqr(0.00001))any_rotation=true;
		else any_rotation=false;
		T alpha = any_rotation?0:AngleFromDir(d.GetNormalized());
		if(alpha<0)alpha=2*M_PI+alpha;
		T cone_angle= TVec<T,3>::AngleBetween(curr,p);
		if(gamma>cone_angle)return false;
		if(!any_rotation)
		{
			T betta =  acos((curr*next>0?-1:1)*gamma/cone_angle);
			rotations[0] = betta - alpha;
			rotations[1] = 2*M_PI - betta - alpha;
		}
	}

	//на какой угол надо повернуть v0 вокруг axis чтобы совместить его с v1
	T AngleBetweenVectors(TVec<T,3> axis,TVec<T,3> v0,TVec<T,3> v1)
	{
		//T result = TVec<T,3>::AngleBetween(v0,v1);
		//if(v1*(axis.Cross(v0))<0)result=-result;
		//return result;
		return atan2(axis*(v0.Cross(v1)),v0*v1);
	}

	///////////////////


	//вектор направления инструмента в системе координат детали для заданных машинных угловых координат
	//направлен от фланца к концу инструмента
	TVec<T,3> GetToolDirFromMachineToolKinematics(T use_B,T use_C)
	{
		T angles[5];
		angles[A_id]=use_B;
		angles[C_id]=use_C;
		TVec<T,3> p(Tool_dir);
		//
		for(int i=nodes_count-1;i>=0;i--)
		{
			if(nodes[i].part_node)
			{
				if(!nodes[i].linear)
					p=InverseRotateNode(p,nodes[i].axis,angles[i]);
			}else
			{
				if(!nodes[i].linear)
					p=ForwardRotateNode(p,nodes[i].axis,angles[i]);
			}
		}
		//
		p=part_system.TransMul(p);
		//
		return p;
	}

	TVec<T,3> ToMachineToolKinematics(TVec<T,3> tool_pos,T use_A,T use_C)
	{
		//assert(false);//TODO проверить а то координаты косячные
		int curr_rot_node=0;
		int rotation_node_id[2];//TODO можно заранее задать
		for(int i=0;i<nodes_count;i++)
			if(!nodes[i].linear)
				rotation_node_id[curr_rot_node++]=i;

		TVec<T,3> tool_dir=GetToolDirFromMachineToolKinematics(use_A,use_C);
		//TODO починить везде
		tool_pos=part_system*(tool_pos-tool_dir*tool_length);

		TVec<T,3> lin_axis_rotated[5];
		TVec<T,3> p(0);
		for(int i=nodes_count-1;i>=0;i--)
		{
			if(nodes[i].part_node)
			{
				if(nodes[i].linear)
				{
					p-=nodes[i].offset;
				}
				else
				{
					p-=nodes[i].offset;
					p=InverseRotateNode(p,nodes[i].axis,(rotation_node_id[0]==i)?use_C:use_A);	
				}
			}else
			{
				if(nodes[i].linear)
				{
					p+=nodes[i].offset;
				}
				else
				{
					p+=nodes[i].offset;
					p=ForwardRotateNode(p,nodes[i].axis,(rotation_node_id[0]==i)?use_C:use_A);
				}
			}
		}
		for(int i=0;i<nodes_count;i++)
		{
			if(!nodes[i].linear)continue;
			lin_axis_rotated[i]=nodes[i].axis;
			for(int k=1;k>=0;k--)
				if(rotation_node_id[k]<i)
				{
					if(nodes[rotation_node_id[k]].part_node)
						lin_axis_rotated[i]=InverseRotateNode(lin_axis_rotated[i],nodes[rotation_node_id[k]].axis,k==0?use_C:use_A);
					else
						lin_axis_rotated[i]=ForwardRotateNode(lin_axis_rotated[i],nodes[rotation_node_id[k]].axis,k==0?use_C:use_A);
				}
		}
		TMatrix<T,3> m(lin_axis_rotated[X_id],lin_axis_rotated[Y_id],lin_axis_rotated[Z_id]);
		m.Invert();
		return m*(tool_pos-p);
	}

	void ToMachineToolKinematics(TVec<T,3> tool_pos,TVec<T,3> tool_dir,
		TKinematicsPair<T>& result)
		//use_fixed_C - используется для определения any_C, на выходе конкретная кинематика для данного fixed_C (должен быть от 0 до pi)
	{
		int curr_rot_node=0;
		int rotation_node_id[2];//TODO можно заранее задать
		for(int i=0;i<nodes_count;i++)
			if(!nodes[i].linear)
				rotation_node_id[curr_rot_node++]=i;

		tool_dir=part_system*tool_dir;
		//TODO починить везде
		tool_dir=-tool_dir;
		tool_pos=part_system*tool_pos-tool_dir*tool_length;
		//получаем 2 варианта положений поворотных осей

		T c[2]={0,0};
		GetRotationToNextPlane(nodes[rotation_node_id[0]].axis,nodes[rotation_node_id[1]].axis,tool_dir,c,result.any_C);

		TVec<T,3> _tool_dir[2];
		for(int i=0;i<2;i++)
		{
			_tool_dir[i]=tool_dir.GetRotated(nodes[rotation_node_id[0]].axis,c[i]);
		}

		//если поворотное звено находится в ветви инструмента то его надо будет поворачивать в противоположную сторону
		if(!nodes[rotation_node_id[0]].part_node)
		{
			c[0]=-c[0];
			c[1]=-c[1];
		}

		T b[2]={0,0};
		for(int i=0;i<2;i++)
			b[i]=AngleBetweenVectors(nodes[rotation_node_id[1]].axis,_tool_dir[i],Tool_dir);

		if(!nodes[rotation_node_id[1]].part_node)
		{
			b[0]=-b[0];
			b[1]=-b[1];
		}

		//составляем матрицу преобразования перемещений линейных осей для двух вариантов кинематики
		for(int v=0;v<2;v++)
		{
			TVec<T,3> lin_axis_rotated[5];
			TVec<T,3> p(0);
			for(int i=nodes_count-1;i>=0;i--)
			{
				if(nodes[i].part_node)
				{
					if(nodes[i].linear)
					{
						p-=nodes[i].offset;
					}
					else
					{
						p-=nodes[i].offset;
						p=InverseRotateNode(p,nodes[i].axis,(rotation_node_id[0]==i)?c[v]:b[v]);	
					}
				}else
				{
					if(nodes[i].linear)
					{
						p+=nodes[i].offset;
					}
					else
					{
						p+=nodes[i].offset;
						p=ForwardRotateNode(p,nodes[i].axis,(rotation_node_id[0]==i)?c[v]:b[v]);
					}
				}
			}
			for(int i=0;i<nodes_count;i++)
			{
				if(!nodes[i].linear)continue;
				lin_axis_rotated[i]=nodes[i].axis;
				for(int k=1;k>=0;k--)
					if(rotation_node_id[k]<i)
					{
						if(nodes[rotation_node_id[k]].part_node)
							lin_axis_rotated[i]=InverseRotateNode(lin_axis_rotated[i],nodes[rotation_node_id[k]].axis,k==0?c[v]:b[v]);
						else
							lin_axis_rotated[i]=ForwardRotateNode(lin_axis_rotated[i],nodes[rotation_node_id[k]].axis,k==0?c[v]:b[v]);
					}
			}
			TMatrix<T,3> m(lin_axis_rotated[X_id],lin_axis_rotated[Y_id],lin_axis_rotated[Z_id]);
			m.Invert();
			result.variant[v].pos=m*(tool_pos-p);
			result.variant[v].A=C_id<A_id?b[v]:c[v];
			result.variant[v].C=C_id>A_id?b[v]:c[v];
		}
	}

	void FromMachineToolKinematics(TKinematics<T> source,
		TVec<T,3> &tool_pos,TVec<T,3> &tool_dir)
	{
		T coords[5];
		coords[A_id]=source.A;
		coords[C_id]=source.C;
		coords[X_id]=source.pos[0];
		coords[Y_id]=source.pos[1];
		coords[Z_id]=source.pos[2];

		TVec<T,3> p(Tool_dir*tool_length);
		//
		for(int i=nodes_count-1;i>=0;i--)
		{
			if(nodes[i].part_node)
			{
				if(nodes[i].linear)
				{
					p-=nodes[i].offset;
					p=InverseLinearNode(p,nodes[i].axis,coords[i]);
				}
				else
				{
					p-=nodes[i].offset;
					p=InverseRotateNode(p,nodes[i].axis_offset,nodes[i].axis,coords[i]);
				}
			}else
			{
				if(nodes[i].linear)
				{
					p+=nodes[i].offset;
					p=ForwardLinearNode(p,nodes[i].axis,coords[i]);
				}
				else
				{
					p+=nodes[i].offset;
					p=ForwardRotateNode(p,nodes[i].axis_offset,nodes[i].axis,coords[i]);
				}
			}
		}
		//
		tool_pos=part_system.TransMul(p);
		//
		tool_dir=GetToolDirFromMachineToolKinematics(source.A,source.C);
		tool_dir=-tool_dir;
	}

	T AToMachineRange(T a)
	{
		return a;
	}

	T CToMachineRange(T c)
	{
		while(c<C_pole_min)c+=M_PI*2.0;
		while(c>C_pole_max)c-=M_PI*2.0;
		if(CIsInPole(c))throw exception("Coordinate 'C' is out of range");
		return c;
	}
	//void InsertGCodeHead(std::string& result_code,TVec<T,3> use_offset, string use_machine_offset_string,bool use_machine_offset)
	//{
	//	//result_code=(boost::format("N1 G54 T1M6\r\nN2 G09\r\nN3 G01F1000\r\n")).str()+result_code;
	//	//TODO переделать более универсально для разных станков
	//	//result_code=(boost::format("N1 G54 T1M6 G01F1000\r\nN2 TRANS Z%.3f\r\n\r\n")%(-k0[2])).str()+result_code;
	//	result_code=(boost::format(
	//		"N1 G01F1000\r\n"
	//		//"N2 M62\r\n"
	//		//"N3 M61\r\n"
	//		"N4 \r\n\r\n")).str()+result_code;
	//	//result_code=(boost::format("%%\r\nN1 G56 G01F1000\r\nN2 G09\r\n\r\n")).str()+result_code+"N10000000 M2\r\nN10000001 M30";
	//}
};