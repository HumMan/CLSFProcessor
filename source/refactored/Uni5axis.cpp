#include "Uni5Axis.h"

#define M_PI 3.14159265358979323846

using namespace CLSFProcessor;

TAngle CLSFProcessor::AngleBetween(Eigen::Vector3d v0, Eigen::Vector3d v1)
{
	//return TAngle(acos((v0.norm()*v1.norm()) / (v0.dot(v1))));
	return TAngle(acos((v0.dot(v1))));
}

TAngle CLSFProcessor::AngleFromDir(const Eigen::Vector2d& v)
{
	return TAngle(atan2(v[1], v[0]));
}

TUniversal5axis::TUniversal5axis(CLSFProcessor::Conf::TFiveAxis use_conf, double use_tool_length)
{

	tool_length = use_tool_length;
	conf = use_conf;

	this->ids = GetNodesIndex();

	//TODO добавить проверки правильности кинематик (ось инструмента должна лежать в плоскости поворота последней оси
}
void TUniversal5axis::SetToolLength(double use_tool_length)
{
	tool_length = use_tool_length;
}
Eigen::Vector3d TUniversal5axis::ForwardLinearNode(Eigen::Vector3d p, Eigen::Vector3d axis, double value)
{
	return p + axis * value;
}
Eigen::Vector3d TUniversal5axis::InverseLinearNode(Eigen::Vector3d machine_p, Eigen::Vector3d axis, double value)
{
	return machine_p - axis * value;
}

Eigen::Vector3d TUniversal5axis::ForwardRotateNode(Eigen::Vector3d p, Eigen::Vector3d off, Eigen::Vector3d axis, TAngle angle)
{
	Eigen::AngleAxis<double> aa(angle.AsRad(), axis);
	return aa * (p - off) + off;
}
Eigen::Vector3d TUniversal5axis::InverseRotateNode(Eigen::Vector3d machine_p, Eigen::Vector3d off, Eigen::Vector3d axis, TAngle angle)
{
	Eigen::AngleAxis<double> aa(-angle.AsRad(), axis);
	return aa * (machine_p - off) + off;
}

Eigen::Vector3d TUniversal5axis::ForwardRotateNode(Eigen::Vector3d dir, Eigen::Vector3d axis, TAngle angle)
{
	Eigen::AngleAxis<double> aa(angle.AsRad(), axis);
	return aa * dir;
}
Eigen::Vector3d TUniversal5axis::InverseRotateNode(Eigen::Vector3d machine_dir, Eigen::Vector3d axis, TAngle angle)
{
	Eigen::AngleAxis<double> aa(-angle.AsRad(), axis);
	return aa * machine_dir;
}

//rotations - углы на которые надо повернуть вектор p вокруг оси curr чтобы он полностью оказался в плоскости перпенидкулярной next
//result - false если такого поворота не существует
//any_rotation - вектор curr лежит в плоскости перпенд. next и совпадает с вектором curr
bool TUniversal5axis::GetRotationToNextPlane(Eigen::Vector3d curr, Eigen::Vector3d next, Eigen::Vector3d p, TRotations& rotations, bool &any_rotation)
{
	//TODO надо математическки провверить а то что-то не сходится
	Eigen::Vector3d _z = curr;
	Eigen::Vector3d _y = (curr.cross(next)).normalized();
	Eigen::Vector3d _x = _y.cross(curr);

	TAngle gamma = AngleBetween(next, _x);
	Eigen::Vector2d d(p.dot(_x), p.dot(_y));
	if (d.squaredNorm() < 0.0000001)
		any_rotation = true;
	else
		any_rotation = false;
	TAngle alpha = any_rotation ? TAngle(0) : AngleFromDir(d.normalized());
	if (alpha.AsRad() < 0)
		alpha = TAngle::FromRad(2 * M_PI + alpha.AsRad());
	TAngle cone_angle = AngleBetween(curr, p);
	if (gamma.AsRad() > cone_angle.AsRad())
		return false;

	double betta = acos((curr.dot(next) > 0 ? -1 : 1)*gamma.AsRad() / cone_angle.AsRad());
	rotations.v[0] = TAngle(betta - alpha.AsRad());
	rotations.v[1] = TAngle(2 * M_PI - betta - alpha.AsRad());
}

//на какой угол надо повернуть v0 вокруг axis чтобы совместить его с v1
double TUniversal5axis::AngleBetweenVectors(Eigen::Vector3d axis, Eigen::Vector3d v0, Eigen::Vector3d v1)
{
	//double result = Eigen::Vector3d::AngleBetween(v0,v1);
	//if(v1*(axis.cross(v0))<0)result=-result;
	//return result;
	return atan2(axis.dot(v0.cross(v1)), v0.dot(v1));
}

///////////////////


//вектор направления инструмента в системе координат детали для заданных машинных угловых координат
//направлен от фланца к концу инструмента
Eigen::Vector3d TUniversal5axis::GetToolDirFromMachineToolKinematics(TKinematics coords)
{
	Eigen::Vector3d p(conf.mach_tool_dir);
	//
	for (int i = conf.nodes.size() - 1; i >= 0; i--)
	{
		if (conf.nodes[i].is_part_node)
		{
			if (!conf.nodes[i].is_linear)
				p = InverseRotateNode(p, conf.nodes[i].axis, TAngle(coords.v[i]));
		}
		else
		{
			if (!conf.nodes[i].is_linear)
				p = ForwardRotateNode(p, conf.nodes[i].axis, TAngle(coords.v[i]));
		}
	}
	//
	p = conf.part_system.transpose()*p;
	//
	return p;
}

Eigen::Vector3d TUniversal5axis::ToMachineToolKinematics(Eigen::Vector3d tool_pos, TKinematics coord)
{
	Eigen::Vector3d tool_dir = GetToolDirFromMachineToolKinematics(coord);
	//TODO починить везде
	tool_pos = conf.part_system * (tool_pos - tool_dir * tool_length);

	int curr_rot_node = 0;
	int curr_lin_node = 0;
	int rotation_node_id[2];//TODO можно заранее задать
	int linear_node_id[3];
	for (int i = 0; i < conf.nodes.size(); i++)
		if (!conf.nodes[i].is_linear)
			rotation_node_id[curr_rot_node++] = i;
		else
			linear_node_id[curr_lin_node++] = i;

	Eigen::Vector3d lin_axis_rotated[5];
	Eigen::Vector3d p(0.0, 0.0, 0.0);
	for (int i = conf.nodes.size() - 1; i >= 0; i--)
	{
		if (conf.nodes[i].is_part_node)
		{
			if (conf.nodes[i].is_linear)
			{
				p -= conf.nodes[i].offset;
			}
			else
			{
				p -= conf.nodes[i].offset;
				p = InverseRotateNode(p, conf.nodes[i].axis, TAngle(coord.v[i]));
			}
		}
		else
		{
			if (conf.nodes[i].is_linear)
			{
				p += conf.nodes[i].offset;
			}
			else
			{
				p += conf.nodes[i].offset;
				p = ForwardRotateNode(p, conf.nodes[i].axis, TAngle(coord.v[i]));
			}
		}
	}
	for (int i = 0; i < conf.nodes.size(); i++)
	{
		if (!conf.nodes[i].is_linear)continue;
		lin_axis_rotated[i] = conf.nodes[i].axis;
		for (int k = 1; k >= 0; k--)
			if (rotation_node_id[k] < i)
			{
				if (conf.nodes[rotation_node_id[k]].is_part_node)
					lin_axis_rotated[i] = InverseRotateNode(lin_axis_rotated[i], conf.nodes[rotation_node_id[k]].axis, TAngle(coord.v[i]));
				else
					lin_axis_rotated[i] = ForwardRotateNode(lin_axis_rotated[i], conf.nodes[rotation_node_id[k]].axis, TAngle(coord.v[i]));
			}
	}
	Eigen::Matrix3d m;
	m.col(0) = lin_axis_rotated[linear_node_id[0]];
	m.col(1) = lin_axis_rotated[linear_node_id[1]];
	m.col(2) = lin_axis_rotated[linear_node_id[2]];
	m.inverse();
	return m * (tool_pos - p);
}

//use_fixed_C - используется для определения any_C, на выходе конкретная кинематика для данного fixed_C (должен быть от 0 до pi)
void TUniversal5axis::ToMachineToolKinematics(Eigen::Vector3d tool_pos, Eigen::Vector3d tool_dir,
	TKinematicsPair& result)
{

	tool_dir = conf.part_system * tool_dir;
	tool_pos = conf.part_system * tool_pos - tool_dir * tool_length;
	//получаем 2 варианта положений поворотных осей

	TRotations c;
	GetRotationToNextPlane(conf.nodes[ids.rot_node[0]].axis, conf.nodes[ids.rot_node[1]].axis, tool_dir, c, result.has_undet_coord);

	Eigen::Vector3d _tool_dir[2];
	for (int i = 0; i < 2; i++)
	{
		Eigen::AngleAxis<double> aa(c.v[i].AsRad(), conf.nodes[ids.rot_node[0]].axis);
		_tool_dir[i] = aa * tool_dir;
	}

	//если поворотное звено находится в ветви инструмента то его надо будет поворачивать в противоположную сторону
	if (!conf.nodes[ids.rot_node[0]].is_part_node)
	{
		c.v[0] = TAngle(-(c.v[0].AsRad()));
		c.v[1] = TAngle(-(c.v[1].AsRad()));
	}

	double b[2] = { 0,0 };
	for (int i = 0; i < 2; i++)
		b[i] = AngleBetweenVectors(conf.nodes[ids.rot_node[1]].axis, _tool_dir[i], conf.mach_tool_dir);

	if (!conf.nodes[ids.rot_node[1]].is_part_node)
	{
		b[0] = -b[0];
		b[1] = -b[1];
	}

	//составляем матрицу преобразования перемещений линейных осей для двух вариантов кинематики
	for (int v = 0; v < 2; v++)
	{
		Eigen::Vector3d lin_axis_rotated[5];
		Eigen::Vector3d p(0, 0, 0);
		for (int i = conf.nodes.size() - 1; i >= 0; i--)
		{
			if (conf.nodes[i].is_part_node)
			{
				if (conf.nodes[i].is_linear)
				{
					p -= conf.nodes[i].offset;
				}
				else
				{
					p -= conf.nodes[i].offset;
					p = InverseRotateNode(p, conf.nodes[i].axis, (ids.rot_node[0] == i) ? c.v[v] : TAngle(b[v]));
				}
			}
			else
			{
				if (conf.nodes[i].is_linear)
				{
					p += conf.nodes[i].offset;
				}
				else
				{
					p += conf.nodes[i].offset;
					p = ForwardRotateNode(p, conf.nodes[i].axis, (ids.rot_node[0] == i) ? c.v[v] : TAngle(b[v]));
				}
			}
		}
		for (int i = 0; i < conf.nodes.size(); i++)
		{
			if (!conf.nodes[i].is_linear)continue;
			lin_axis_rotated[i] = conf.nodes[i].axis;
			for (int k = 1; k >= 0; k--)
				if (ids.rot_node[k] < i)
				{
					if (conf.nodes[ids.rot_node[k]].is_part_node)
						lin_axis_rotated[i] = InverseRotateNode(lin_axis_rotated[i], conf.nodes[ids.rot_node[k]].axis, k == 0 ? c.v[v] : TAngle(b[v]));
					else
						lin_axis_rotated[i] = ForwardRotateNode(lin_axis_rotated[i], conf.nodes[ids.rot_node[k]].axis, k == 0 ? c.v[v] : TAngle(b[v]));
				}
		}

		Eigen::Matrix3d m;
		m.col(0) = lin_axis_rotated[ids.linear_node[0]];
		m.col(1) = lin_axis_rotated[ids.linear_node[1]];
		m.col(2) = lin_axis_rotated[ids.linear_node[2]];
		m.inverse();


		Eigen::Vector3d pos = m * (tool_pos - p);
		result.variant[v].v[ids.linear_node[0]] = pos[0];
		result.variant[v].v[ids.linear_node[1]] = pos[1];
		result.variant[v].v[ids.linear_node[2]] = pos[2];

		result.variant[v].v[ids.rot_node[0]] = c.v[v].AsRad();
		result.variant[v].v[ids.rot_node[1]] = b[v];
	}
}

void TUniversal5axis::FromMachineToolKinematics(TKinematics source,
	Eigen::Vector3d &tool_pos, Eigen::Vector3d &tool_dir)
{
	Eigen::Vector3d p(conf.mach_tool_dir*tool_length);
	//
	for (int i = conf.nodes.size() - 1; i >= 0; i--)
	{
		if (conf.nodes[i].is_part_node)
		{
			if (conf.nodes[i].is_linear)
			{
				p -= conf.nodes[i].offset;
				p = InverseLinearNode(p, conf.nodes[i].axis, source.v[i]);
			}
			else
			{
				p -= conf.nodes[i].offset;
				p = InverseRotateNode(p, conf.nodes[i].axis_offset, conf.nodes[i].axis, TAngle(source.v[i]));
			}
		}
		else
		{
			if (conf.nodes[i].is_linear)
			{
				p += conf.nodes[i].offset;
				p = ForwardLinearNode(p, conf.nodes[i].axis, source.v[i]);
			}
			else
			{
				p += conf.nodes[i].offset;
				p = ForwardRotateNode(p, conf.nodes[i].axis_offset, conf.nodes[i].axis, TAngle(source.v[i]));
			}
		}
	}
	//
	Eigen::Matrix3d m(conf.part_system);
	m.transpose();
	tool_pos = m * p;
	//
	tool_dir = GetToolDirFromMachineToolKinematics(source);
}

TNodesIndex TUniversal5axis::GetNodesIndex()
{
	int curr_linear = 0, curr_rot = 0;

	TNodesIndex result;

	//TODO добавить проверки поворотных осей
	//ось инструмента параллельная оси C
	//угол между осью инструмента и A
	//угол между C и A 90 гр.

	for (int i = 0; i < conf.nodes.size(); i++)
	{
		auto& node = conf.nodes[i];
		if (node.is_linear)
		{
			if (curr_linear > 2)
				throw std::exception("неверная конфигурация");
			result.linear_node[curr_linear] = i;
			curr_linear++;
		}
		else if (!node.is_linear)
		{
			if (curr_rot > 1)
				throw std::exception("неверная конфигурация");
			result.rot_node[curr_rot] = i;
			curr_rot++;
		}
	}

	if (curr_rot != 2 || curr_linear != 3)
		throw std::exception("неверная конфигурация");

	return result;
}