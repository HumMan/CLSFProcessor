#pragma once

#include "kinematicsConf.h"

#include "commonConf.h"

#include <array>

namespace CLSFProcessor
{
	struct TKinematics
	{
		//5 координат, соответствующих звеньям станка (2 поворотных и 3 линейных в любой комбинации)
		std::array<double,5> v;
	};

	struct TKinematicsPair
	{
		std::array<TKinematics,2> variant;//варианты кинематики станка
		bool has_undet_coord;//ось А вертикальна -> значение C может быть любое
		int undet_coord_id;
	};

	struct TNodesIndex
	{
		std::array<int,3> linear_node; //индексы линейных звеньев
		std::array<int,2> rot_node; //индексы поворотных звеньев
	};
	

	TAngle AngleBetween(Eigen::Vector3d v0, Eigen::Vector3d v1);

	TAngle AngleFromDir(const Eigen::Vector2d& v);

	struct TRotations
	{
		std::array<TAngle,2> v;
	};

	class TUniversal5axis
	{
	private:
		CLSFProcessor::Conf::TFiveAxis conf;
		double tool_length;
		TNodesIndex ids;
	public:
		TUniversal5axis(CLSFProcessor::Conf::TFiveAxis use_conf, double use_tool_length = 0);
		void SetToolLength(double use_tool_length);
		Eigen::Vector3d ForwardLinearNode(Eigen::Vector3d p, Eigen::Vector3d axis, double value);
		Eigen::Vector3d InverseLinearNode(Eigen::Vector3d machine_p, Eigen::Vector3d axis, double value);
		Eigen::Vector3d ForwardRotateNode(Eigen::Vector3d p, Eigen::Vector3d off, Eigen::Vector3d axis, TAngle angle);
		Eigen::Vector3d InverseRotateNode(Eigen::Vector3d machine_p, Eigen::Vector3d off, Eigen::Vector3d axis, TAngle angle);
		Eigen::Vector3d ForwardRotateNode(Eigen::Vector3d dir, Eigen::Vector3d axis, TAngle angle);
		Eigen::Vector3d InverseRotateNode(Eigen::Vector3d machine_dir, Eigen::Vector3d axis, TAngle angle);
		//a0,a1 - углы на которые надо повернуть вектор p вокруг оси curr чтобы он полностью оказался в плоскости перпенидкулярной next
		//result - false если такого поворота не существует
		//any_rotation - вектор curr лежит в плоскости перпенд. next и совпадает с вектором curr
		bool GetRotationToNextPlane(Eigen::Vector3d curr, Eigen::Vector3d next, Eigen::Vector3d p, TRotations& rotations, bool &any_rotation);

		//на какой угол надо повернуть v0 вокруг axis чтобы совместить его с v1
		double AngleBetweenVectors(Eigen::Vector3d axis, Eigen::Vector3d v0, Eigen::Vector3d v1);

		//вектор направления инструмента в системе координат детали для заданных машинных угловых координат
		//направлен от фланца к концу инструмента
		Eigen::Vector3d GetToolDirFromMachineToolKinematics(TKinematics coords);
		Eigen::Vector3d ToMachineToolKinematics(Eigen::Vector3d tool_pos, TKinematics coord);

		//use_fixed_C - используется для определения any_C, на выходе конкретная кинематика для данного fixed_C (должен быть от 0 до pi)
		void ToMachineToolKinematics(Eigen::Vector3d tool_pos, Eigen::Vector3d tool_dir,
			TKinematicsPair& result);
			

		void FromMachineToolKinematics(TKinematics source,
			Eigen::Vector3d &tool_pos, Eigen::Vector3d &tool_dir);

		TNodesIndex GetNodesIndex();
	};
}