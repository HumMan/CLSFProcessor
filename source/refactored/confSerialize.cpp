#include "confSerialize.h"

#include <pugixml.hpp>

using namespace CLSFProcessor;

Eigen::Vector3d ExtractV3d(pugi::xml_node node)
{
	return Eigen::Vector3d(node.attribute("x").as_double(), node.attribute("y").as_double(), node.attribute("z").as_double());
}

void CLSFProcessor::ParseConfig(const std::string& config_path, Conf::TCommon& common_conf, Conf::TFiveAxis& five_axis_conf)
{
	setlocale(LC_ALL, "C");
	pugi::xml_document doc;
	doc.load_file(pugi::as_wide(config_path.c_str()).c_str());
	
	int version = doc.child("version").attribute("id").as_int();

	pugi::xml_node ini_params = doc.child("kinematics");
	if (version == 1)
	{
		pugi::xml_node
			part_x = ini_params.child("part_system_x"),
			part_y = ini_params.child("part_system_y"),
			part_z = ini_params.child("part_system_z");
		five_axis_conf.part_system.col(0) = ExtractV3d(part_x);
		five_axis_conf.part_system.col(1) = ExtractV3d(part_y);
		five_axis_conf.part_system.col(2) = ExtractV3d(part_z);

		pugi::xml_node nodes = ini_params.child("nodes");
		for (pugi::xml_node_iterator it = nodes.begin(); it != nodes.end(); ++it)
		{
			Conf::TMovementNode n;
			n.is_part_node = it->child("is_part_node").attribute("value").as_bool();
			n.is_linear = it->child("is_linear").attribute("value").as_bool();
			n.axis_offset = ExtractV3d(it->child("axis_offset"));
			n.axis = ExtractV3d(it->child("axis"));
			n.offset = ExtractV3d(it->child("offset"));

			five_axis_conf.nodes.push_back(n);
		}
		five_axis_conf.mach_tool_dir = ExtractV3d(ini_params.child("tool_dir"));
	}
	else
		throw "Not supported ini_params.xml version!";
}
