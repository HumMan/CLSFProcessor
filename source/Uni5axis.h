#pragma once

#include <Eigen/Dense>
#include <vector>
#include <pugixml.hpp>

using namespace Eigen;
using namespace std;

class TMovementNode
{
public:
	bool is_part_node;//иначе tool_node
	bool is_linear;//иначе поворотное звено
	Vector3d axis_offset;
	Vector3d axis;
	Vector3d offset;//вектор смещения с направлением от текущего звена к следующему (привязан к текущему звену)

	TMovementNode(){}
	TMovementNode(bool use_part_node,bool use_linear,Vector3d use_axis_offset,Vector3d use_axis,Vector3d use_offset=Vector3d(0.0))
		:is_part_node(use_part_node)
		,is_linear(use_linear)
		,axis_offset(use_axis_offset)
		,axis(use_axis)
		,offset(use_offset){}
};

typedef double TKinematics[5];

struct TKinematicsPair
{
	TKinematics variant[2];//варианты кинематики станка
	bool has_undet_coord;//ось А вертикальна -> значение C может быть любое
	int undet_coord_id;
};

Vector3d ExtractV3d(pugi::xml_node node)
{
	return Vector3d(node.attribute("x").as_double(),node.attribute("y").as_double(),node.attribute("z").as_double());
}

void PackV3d(const Vector3d& v,pugi::xml_node node)
{
	node.append_attribute("x")=v[0];
	node.append_attribute("y")=v[1];
	node.append_attribute("z")=v[2];
}

double AngleBetween(Vector3d v0, Vector3d v1)
{
	return acos((v0.norm()*v1.norm())/(v0.dot(v1)));
}

double AngleFromDir(const Vector2d& v)
{
	return atan2(v[1],v[0]);
}

class TUniversal5axis
{
private:
	Matrix3d part_system;
	vector<TMovementNode> nodes;//from part to tool
	Vector3d mach_tool_dir;
	double tool_length;
public:
	TUniversal5axis(pugi::xml_node ini_params, double use_tool_length=0)
	{
		//TODO добавить проверки правильности кинематик (1- количество линейных и угловых звеньев 2- ось инструмента должна лежать в плоскости поворота последней оси
		tool_length=use_tool_length;
		int version=ini_params.child("version").attribute("id").as_int();
		ini_params=ini_params.child("params");
		if(version==1)
		{
			pugi::xml_node 
				part_x=ini_params.child("part_system_x"),
				part_y=ini_params.child("part_system_y"),
				part_z=ini_params.child("part_system_z");
			part_system.col(0)=ExtractV3d(part_x);
			part_system.col(1)=ExtractV3d(part_y);
			part_system.col(2)=ExtractV3d(part_z);

			pugi::xml_node nodes=ini_params.child("nodes");
			for (pugi::xml_node_iterator it = nodes.begin(); it != nodes.end(); ++it)
			{
				TMovementNode n;
				n.is_part_node=it->child("is_part_node").attribute("value").as_bool();
				n.is_linear=it->child("is_linear").attribute("value").as_bool();
				n.axis_offset=ExtractV3d(it->child("axis_offset"));
				n.axis=ExtractV3d(it->child("axis"));
				n.offset=ExtractV3d(it->child("offset"));
			}
			mach_tool_dir=ExtractV3d(ini_params.child("tool_dir"));
		}else 
			throw "Not supported ini_params.xml version!";
	}
	void SetToolLength(double use_tool_length)
	{
		tool_length=use_tool_length;
	}
	Vector3d ForwardLinearNode(Vector3d p,Vector3d axis, double value)
	{
		return p+axis*value;
	}
	Vector3d InverseLinearNode(Vector3d machine_p,Vector3d axis, double value)
	{
		return machine_p-axis*value;
	}

	Vector3d ForwardRotateNode(Vector3d p,Vector3d off,Vector3d axis, double angle)
	{
		AngleAxis<double> aa(angle, axis);
		return aa*(p-off)+off;
	}
	Vector3d InverseRotateNode(Vector3d machine_p,Vector3d off,Vector3d axis, double angle)
	{
		AngleAxis<double> aa(-angle, axis);
		return aa*(machine_p-off)+off;
	}

	Vector3d ForwardRotateNode(Vector3d dir,Vector3d axis, double angle)
	{
		AngleAxis<double> aa(angle, axis);
		return aa*dir;
	}
	Vector3d InverseRotateNode(Vector3d machine_dir,Vector3d axis, double angle)
	{
		AngleAxis<double> aa(-angle, axis);
		return aa*machine_dir;
	}

	//a0,a1 - углы на которые надо повернуть вектор p вокруг оси curr чтобы он полностью оказался в плоскости перпенидкулярной next
	//result - false если такого поворота не существует
	//any_rotation - вектор curr лежит в плоскости перпенд. next и совпадает с вектором curr
	bool GetRotationToNextPlane(Vector3d curr,Vector3d next, Vector3d p, double rotations[], bool &any_rotation)
	{
		//TODO надо математическки провверить а то что-то не сходится
		Vector3d _z=curr;
		Vector3d _y=(curr.cross(next)).normalized();
		Vector3d _x=_y.cross(curr);

		double gamma = AngleBetween(next,_x);
		Vector2d d(p.dot(_x),p.dot(_y));
		if(d.squaredNorm()<0.0000001)any_rotation=true;
		else any_rotation=false;
		double alpha = any_rotation?0:AngleFromDir(d.normalized());
		if(alpha<0)alpha=2*M_PI+alpha;
		double cone_angle= AngleBetween(curr,p);
		if(gamma>cone_angle)return false;
		if(!any_rotation)
		{
			double betta =  acos((curr.dot(next)>0?-1:1)*gamma/cone_angle);
			rotations[0] = betta - alpha;
			rotations[1] = 2*M_PI - betta - alpha;
		}
	}

	//на какой угол надо повернуть v0 вокруг axis чтобы совместить его с v1
	double AngleBetweenVectors(Vector3d axis,Vector3d v0,Vector3d v1)
	{
		//double result = Vector3d::AngleBetween(v0,v1);
		//if(v1*(axis.cross(v0))<0)result=-result;
		//return result;
		return atan2(axis.dot(v0.cross(v1)),v0.dot(v1));
	}

	///////////////////


	//вектор направления инструмента в системе координат детали для заданных машинных угловых координат
	//направлен от фланца к концу инструмента
	Vector3d GetToolDirFromMachineToolKinematics(double* coords)
	{
		Vector3d p(mach_tool_dir);
		//
		for(int i=nodes.size()-1;i>=0;i--)
		{
			if(nodes[i].is_part_node)
			{
				if(!nodes[i].is_linear)
					p=InverseRotateNode(p,nodes[i].axis,coords[i]);
			}else
			{
				if(!nodes[i].is_linear)
					p=ForwardRotateNode(p,nodes[i].axis,coords[i]);
			}
		}
		//
		p=part_system.transpose()*p;
		//
		return p;
	}

	Vector3d ToMachineToolKinematics(Vector3d tool_pos,double* coord)
	{
		Vector3d tool_dir=GetToolDirFromMachineToolKinematics(coord);
		//TODO починить везде
		tool_pos=part_system*(tool_pos-tool_dir*tool_length);
		
		int curr_rot_node=0;
		int curr_lin_node=0;
		int rotation_node_id[2];//TODO можно заранее задать
		int linear_node_id[3];
		for(int i=0;i<nodes.size();i++)
			if(!nodes[i].is_linear)
				rotation_node_id[curr_rot_node++]=i;
			else 
				linear_node_id[curr_lin_node++]=i;

		Vector3d lin_axis_rotated[5];
		Vector3d p(0.0);
		for(int i=nodes.size()-1;i>=0;i--)
		{
			if(nodes[i].is_part_node)
			{
				if(nodes[i].is_linear)
				{
					p-=nodes[i].offset;
				}
				else
				{
					p-=nodes[i].offset;
					p=InverseRotateNode(p,nodes[i].axis,coord[i]);	
				}
			}else
			{
				if(nodes[i].is_linear)
				{
					p+=nodes[i].offset;
				}
				else
				{
					p+=nodes[i].offset;
					p=ForwardRotateNode(p,nodes[i].axis,coord[i]);
				}
			}
		}
		for(int i=0;i<nodes.size();i++)
		{
			if(!nodes[i].is_linear)continue;
			lin_axis_rotated[i]=nodes[i].axis;
			for(int k=1;k>=0;k--)
				if(rotation_node_id[k]<i)
				{
					if(nodes[rotation_node_id[k]].is_part_node)
						lin_axis_rotated[i]=InverseRotateNode(lin_axis_rotated[i],nodes[rotation_node_id[k]].axis,coord[i]);
					else
						lin_axis_rotated[i]=ForwardRotateNode(lin_axis_rotated[i],nodes[rotation_node_id[k]].axis,coord[i]);
				}
		}
		Matrix3d m;
		m.col(0)=lin_axis_rotated[linear_node_id[0]];
		m.col(1)=lin_axis_rotated[linear_node_id[1]];
		m.col(2)=lin_axis_rotated[linear_node_id[2]];
		m.inverse();
		return m*(tool_pos-p);
	}

	void ToMachineToolKinematics(Vector3d tool_pos,Vector3d tool_dir,
		TKinematicsPair& result)
		//use_fixed_C - используется для определения any_C, на выходе конкретная кинематика для данного fixed_C (должен быть от 0 до pi)
	{
		int curr_rot_node=0;
		int rotation_node_id[2];//TODO можно заранее задать
		for(int i=0;i<nodes.size();i++)
			if(!nodes[i].is_linear)
				rotation_node_id[curr_rot_node++]=i;

		tool_dir=part_system*tool_dir;
		//TODO починить везде
		tool_dir=-tool_dir;
		tool_pos=part_system*tool_pos-tool_dir*tool_length;
		//получаем 2 варианта положений поворотных осей

		double c[2]={0,0};
		GetRotationToNextPlane(nodes[rotation_node_id[0]].axis,nodes[rotation_node_id[1]].axis,tool_dir,c,result.has_undet_coord);

		Vector3d _tool_dir[2];
		for(int i=0;i<2;i++)
		{
			AngleAxis<double> aa(c[i], nodes[rotation_node_id[0]].axis);
			_tool_dir[i]=aa*tool_dir;
		}

		//если поворотное звено находится в ветви инструмента то его надо будет поворачивать в противоположную сторону
		if(!nodes[rotation_node_id[0]].is_part_node)
		{
			c[0]=-c[0];
			c[1]=-c[1];
		}

		double b[2]={0,0};
		for(int i=0;i<2;i++)
			b[i]=AngleBetweenVectors(nodes[rotation_node_id[1]].axis,_tool_dir[i],tool_dir);

		if(!nodes[rotation_node_id[1]].is_part_node)
		{
			b[0]=-b[0];
			b[1]=-b[1];
		}

		//составляем матрицу преобразования перемещений линейных осей для двух вариантов кинематики
		for(int v=0;v<2;v++)
		{
			Vector3d lin_axis_rotated[5];
			Vector3d p(0.0);
			for(int i=nodes.size()-1;i>=0;i--)
			{
				if(nodes[i].is_part_node)
				{
					if(nodes[i].is_linear)
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
					if(nodes[i].is_linear)
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
			for(int i=0;i<nodes.size();i++)
			{
				if(!nodes[i].is_linear)continue;
				lin_axis_rotated[i]=nodes[i].axis;
				for(int k=1;k>=0;k--)
					if(rotation_node_id[k]<i)
					{
						if(nodes[rotation_node_id[k]].is_part_node)
							lin_axis_rotated[i]=InverseRotateNode(lin_axis_rotated[i],nodes[rotation_node_id[k]].axis,k==0?c[v]:b[v]);
						else
							lin_axis_rotated[i]=ForwardRotateNode(lin_axis_rotated[i],nodes[rotation_node_id[k]].axis,k==0?c[v]:b[v]);
					}
			}

			int linear_node_id[3];
			for(int i=0;i<nodes.size();i++)
				if(nodes[i].is_linear)
					rotation_node_id[curr_rot_node++]=i;
			Matrix3d m;
			m.col(0)=lin_axis_rotated[linear_node_id[0]];
			m.col(1)=lin_axis_rotated[linear_node_id[1]];
			m.col(2)=lin_axis_rotated[linear_node_id[2]];
			m.inverse();

			Vector3d pos=m*(tool_pos-p);
			result.variant[v][linear_node_id[0]]=pos[0];
			result.variant[v][linear_node_id[0]]=pos[1];
			result.variant[v][linear_node_id[0]]=pos[2];

			result.variant[v][rotation_node_id[0]]=c[v];
			result.variant[v][rotation_node_id[1]]=b[v];
		}
	}

	void FromMachineToolKinematics(TKinematics source,
		Vector3d &tool_pos,Vector3d &tool_dir)
	{
		Vector3d p(mach_tool_dir*tool_length);
		//
		for(int i=nodes.size()-1;i>=0;i--)
		{
			if(nodes[i].is_part_node)
			{
				if(nodes[i].is_linear)
				{
					p-=nodes[i].offset;
					p=InverseLinearNode(p,nodes[i].axis,source[i]);
				}
				else
				{
					p-=nodes[i].offset;
					p=InverseRotateNode(p,nodes[i].axis_offset,nodes[i].axis,source[i]);
				}
			}else
			{
				if(nodes[i].is_linear)
				{
					p+=nodes[i].offset;
					p=ForwardLinearNode(p,nodes[i].axis,source[i]);
				}
				else
				{
					p+=nodes[i].offset;
					p=ForwardRotateNode(p,nodes[i].axis_offset,nodes[i].axis,source[i]);
				}
			}
		}
		//
		Matrix3d m(part_system);
		m.transpose();
		tool_pos=m*p;
		//
		tool_dir=GetToolDirFromMachineToolKinematics(source);
		tool_dir=-tool_dir;
	}
};