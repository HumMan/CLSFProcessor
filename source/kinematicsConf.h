#pragma once

#include <Eigen/Dense>
#include <vector>

namespace CLSFProcessor
{
	namespace Conf
	{
		class TMovementNode
		{
		public:
			bool is_part_node;//иначе tool_node
			bool is_linear;//иначе поворотное звено
			Eigen::Vector3d axis_offset;
			Eigen::Vector3d axis;
			Eigen::Vector3d offset;//вектор смещения с направлением от текущего звена к следующему (привязан к текущему звену)

			TMovementNode() {}
			TMovementNode(bool use_part_node, bool use_linear, Eigen::Vector3d use_axis_offset, Eigen::Vector3d use_axis, Eigen::Vector3d use_offset = Eigen::Vector3d(0.0,0.0,0.0))
				:is_part_node(use_part_node)
				, is_linear(use_linear)
				, axis_offset(use_axis_offset)
				, axis(use_axis)
				, offset(use_offset) {}
		};

		class TFiveAxis
		{
		public:
			Eigen::Matrix3d part_system;
			std::vector<TMovementNode> nodes;//from part to tool
			Eigen::Vector3d mach_tool_dir;
		};
	}
}