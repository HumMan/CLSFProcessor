

#include "refactored\kinematicsConf.h"

namespace PrimitiveMask
{
	enum Enum:unsigned int
	{
		LINE=1,
		CIRCLE=2,
	};
}

struct TToolOrientation
{
	TVec<double,3> pos,dir;
};

struct TKinematics
{
	TVec<double,3> pos;
	double 
		A,  //
		C;  //
	bool any_C;
};

struct TKinematicsPair
{
	TKinematics variant[2];//варианты кинематики станка
	bool valid[2];			//вариант кинематики физически реализуем

	bool any_C;//ось А вертикальна -> значение C может быть любое
};

struct TMachineState
{
	PrimitiveMask::Enum mask;
	string auxfun;
	TVec<double,3> center,normal;
	double radius;
	int spiral_times;
	double feed;
	double spndl_rpm;
	bool clw;//TODO разобраться с парсингом а то глючит
	bool rapid;
	int color;
	bool base_element;//если true - элемент нельзя убрать при компрессии т.к. он исходный(существовавший до разбиения)
	int cutcom;
	string path_name;
};

struct TToolMovementElement:public TMachineState,TKinematics
{
	TToolOrientation tool_orient;
	double move_time;
	double move_distance;
	double contour_correct_feed;
};


class TUniversal5axis
{
private:
	CLSFProcessor::Conf::TFiveAxis conf;
	double tool_length;
public:
	TUniversal5axis(CLSFProcessor::Conf::TFiveAxis use_conf, double use_tool_length = 0)
	{
		tool_length = use_tool_length;
		conf = use_conf;
		//TODO добавить проверки правильности кинематик (1- количество линейных и угловых звеньев 2- ось инструмента должна лежать в плоскости поворота последней оси
	}

	TVec<double,3> ForwardLinearNode(TVec<double,3> p,TVec<double,3> axis, double value)
	{
		return p+axis*value;
	}
	TVec<double,3> InverseLinearNode(TVec<double,3> machine_p,TVec<double,3> axis, double value)
	{
		return machine_p-axis*value;
	}

	TVec<double,3> ForwardRotateNode(TVec<double,3> p,TVec<double,3> off,TVec<double,3> axis, double angle)
	{
		return (p-off).GetRotated(axis,angle)+off;
	}
	TVec<double,3> InverseRotateNode(TVec<double,3> machine_p,TVec<double,3> off,TVec<double,3> axis, double angle)
	{
		return (machine_p-off).GetRotated(axis,-angle)+off;
	}

	TVec<double,3> ForwardRotateNode(TVec<double,3> dir,TVec<double,3> axis, double angle)
	{
		return dir.GetRotated(axis,angle);
	}
	TVec<double,3> InverseRotateNode(TVec<double,3> machine_dir,TVec<double,3> axis, double angle)
	{
		return machine_dir.GetRotated(axis,-angle);
	}

	//a0,a1 - углы на которые надо повернуть вектор p вокруг оси curr чтобы он полностью оказался в плоскости перпенидкулярной next
	//result - false если такого поворота не существует
	//any_rotation - вектор curr лежит в плоскости перпенд. next и совпадает с вектором curr
	bool GetRotationToNextPlane(TVec<double,3> curr,TVec<double,3> next, TVec<double,3> p, double rotations[], bool &any_rotation)
	{
		//TODO надо математическки провверить а то что-то не сходится
		TVec<double,3> _z=curr;
		TVec<double,3> _y=curr.Cross(next).GetNormalized();
		TVec<double,3> _x=_y.Cross(curr);

		double gamma = TVec<double,3>::AngleBetween(next,_x);
		TVec<double,2> d(p*_x,p*_y);
		if(d.SqrLength()<sqr(0.00001))any_rotation=true;
		else any_rotation=false;
		double alpha = any_rotation?0:AngleFromDir(d.GetNormalized());
		if(alpha<0)alpha=2*M_PI+alpha;
		double cone_angle= TVec<double,3>::AngleBetween(curr,p);
		if(gamma>cone_angle)return false;
		if(!any_rotation)
		{
			double betta =  acos((curr*next>0?-1:1)*gamma/cone_angle);
			rotations[0] = betta - alpha;
			rotations[1] = 2*M_PI - betta - alpha;
		}
	}

	//на какой угол надо повернуть v0 вокруг axis чтобы совместить его с v1
	double AngleBetweenVectors(TVec<double,3> axis,TVec<double,3> v0,TVec<double,3> v1)
	{
		//double result = TVec<double,3>::AngleBetween(v0,v1);
		//if(v1*(axis.Cross(v0))<0)result=-result;
		//return result;
		return atan2(axis*(v0.Cross(v1)),v0*v1);
	}

	///////////////////


	//вектор направления инструмента в системе координат детали для заданных машинных угловых координат
	//направлен от фланца к концу инструмента
	TVec<double,3> GetToolDirFromMachineToolKinematics(double use_B,double use_C)
	{
		double angles[5];
		angles[A_id]=use_B;
		angles[C_id]=use_C;
		TVec<double,3> p(Tool_dir);
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

	TVec<double,3> ToMachineToolKinematics(TVec<double,3> tool_pos,double use_A,double use_C)
	{
		//assert(false);//TODO проверить а то координаты косячные
		int curr_rot_node=0;
		int rotation_node_id[2];//TODO можно заранее задать
		for(int i=0;i<nodes_count;i++)
			if(!nodes[i].linear)
				rotation_node_id[curr_rot_node++]=i;

		TVec<double,3> tool_dir=GetToolDirFromMachineToolKinematics(use_A,use_C);
		//TODO починить везде
		tool_pos=part_system*(tool_pos-tool_dir*tool_length);

		TVec<double,3> lin_axis_rotated[5];
		TVec<double,3> p(0);
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
		TMatrix<double,3> m(lin_axis_rotated[X_id],lin_axis_rotated[Y_id],lin_axis_rotated[Z_id]);
		m.Invert();
		return m*(tool_pos-p);
	}

	void ToMachineToolKinematics(TVec<double,3> tool_pos,TVec<double,3> tool_dir,
		TKinematicsPair<double>& result)
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

		double c[2]={0,0};
		GetRotationToNextPlane(nodes[rotation_node_id[0]].axis,nodes[rotation_node_id[1]].axis,tool_dir,c,result.any_C);

		TVec<double,3> _tool_dir[2];
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

		double b[2]={0,0};
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
			TVec<double,3> lin_axis_rotated[5];
			TVec<double,3> p(0);
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
			TMatrix<double,3> m(lin_axis_rotated[X_id],lin_axis_rotated[Y_id],lin_axis_rotated[Z_id]);
			m.Invert();
			result.variant[v].pos=m*(tool_pos-p);
			result.variant[v].A=C_id<A_id?b[v]:c[v];
			result.variant[v].C=C_id>A_id?b[v]:c[v];
		}
	}

	void FromMachineToolKinematics(TKinematics<double> source,
		TVec<double,3> &tool_pos,TVec<double,3> &tool_dir)
	{
		double coords[5];
		coords[A_id]=source.A;
		coords[C_id]=source.C;
		coords[X_id]=source.pos[0];
		coords[Y_id]=source.pos[1];
		coords[Z_id]=source.pos[2];

		TVec<double,3> p(Tool_dir*tool_length);
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

	double AToMachineRange(double a)
	{
		return a;
	}

	double CToMachineRange(double c)
	{
		while(c<C_pole_min)c+=M_PI*2.0;
		while(c>C_pole_max)c-=M_PI*2.0;
		if(CIsInPole(c))throw exception("Coordinate 'C' is out of range");
		return c;
	}
	//void InsertGCodeHead(std::string& result_code,TVec<double,3> use_offset, string use_machine_offset_string,bool use_machine_offset)
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