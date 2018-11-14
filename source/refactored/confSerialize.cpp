#include "confSerialize.h"

#include <pugixml.hpp>

using namespace CLSFProcessor;

Eigen::Vector3d ExtractV3d(pugi::xml_node node)
{
	return Eigen::Vector3d(node.attribute("x").as_double(), node.attribute("y").as_double(), node.attribute("z").as_double());
}

void ParseProcessor(pugi::xml_node& ini_params, Conf::TProcessor& conf)
{
	conf.tolerance = ini_params.child("tolerance").attribute("value").as_double();
	conf.rapid_tolerance = ini_params.child("rapid_tolerance").attribute("value").as_double();
	conf.rapid_feed = ini_params.child("rapid_feed").attribute("value").as_double();
	conf.contour_max_feed = ini_params.child("contour_max_feed").attribute("value").as_double();
	conf.frames_on_1sec_max = ini_params.child("frames_on_1sec_max").attribute("value").as_int();
	conf.circle_interpolation_center_absol = ini_params.child("circle_interpolation_center_absol").attribute("value").as_bool();
}

void ParseCommon(pugi::xml_node& ini_params, Conf::TCommon& conf)
{
	//oriented_from_goto=false;
	//oriented_from_goto_orientation=0;

	conf.local_CS_G_index = 54;  //меняется коммандой NX_PROCESSOR_SET_CS_G(формируемой в NXProcessor в зависимости от названия СК - _G(%i) )

	conf.tool_length = ini_params.child("tool_length").attribute("value").as_double();

	conf.any_C_epsilon = ini_params.child("any_C_epsilon").attribute("value").as_double();
	conf.ortho_vec_epsilon = ini_params.child("ortho_vec_epsilon").attribute("value").as_double();

	conf.C_pole_min = TAngle::FromDeg(ini_params.child("C_pole_min").attribute("value").as_double());
	conf.C_pole_max = TAngle::FromDeg(ini_params.child("C_pole_max").attribute("value").as_double());

	conf.A_pole_min = TAngle::FromDeg(ini_params.child("A_pole_min").attribute("value").as_double());
	conf.A_pole_max = TAngle::FromDeg(ini_params.child("A_pole_max").attribute("value").as_double());
	//inverse_kinemtatics_tol=0.001;//максимально допустимое различие между прямой и обратной функцией кинематики

	conf.remove_F_repeat = ini_params.child("remove_F_repeat").attribute("value").as_bool();

	conf.G_code_header = ini_params.child("G_code_header").attribute("value").as_string();
	//boost::replace_all(G_code_header, "\\n", "\n");
	conf.G_code_footer = ini_params.child("G_code_footer").attribute("value").as_string();
	//boost::replace_all(G_code_footer, "\\n", "\n");

	conf.any_C_criteria = ini_params.child("any_C_criteria").attribute("value").as_int();

	conf.use_subdivision = ini_params.child("use_subdivision").attribute("value").as_bool();

	conf.subdivide_only_any_C = ini_params.child("subdivide_only_any_C").attribute("value").as_bool();

	conf.use_circles = ini_params.child("use_circles").attribute("value").as_bool();

	conf.use_tool_length_correction = ini_params.child("use_tool_length_correction").attribute("value").as_bool();

	conf.head_name = ini_params.child("head_name").attribute("value").as_string();

	pugi::xml_node nodes = ini_params.child("coords");
	for (pugi::xml_node_iterator it = nodes.begin(); it != nodes.end(); ++it)
	{
		Conf::TCommon::TGCodeAxisProperties n;

		n.remove_repeat = it->child("remove_repeat").attribute("value").as_bool();
		n.force_rapid_change = it->child("force_rapid_change").attribute("value").as_bool();
		n.lock_header = it->child("lock_header").attribute("value").as_string();
		//boost::replace_all(n.lock_header, "\\n", "\n");
		n.lock_footer = it->child("lock_footer").attribute("value").as_string();
		//boost::replace_all(n.lock_footer, "\\n", "\n");
		n.format = it->child("format").attribute("value").as_string();
		n.is_increment = it->child("is_increment").attribute("value").as_bool();
		n.repeat_tol = it->child("repeat_tol").attribute("value").as_double();
		n.rad_to_deg = it->child("rad_to_deg").attribute("value").as_bool();

		conf.gcode_axis_prop.push_back(n);
	}
}

void ParseKinematics(pugi::xml_node& ini_params, Conf::TFiveAxis& five_axis_conf)
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

void CLSFProcessor::ParseConfig(const std::string& config_path, Conf::TCommon& common_conf, Conf::TProcessor& processor_conf, Conf::TFiveAxis& five_axis_conf)
{
	setlocale(LC_ALL, "C");
	pugi::xml_document doc;
	auto result = doc.load_file(pugi::as_wide(config_path.c_str()).c_str());

	int version = doc.child("version").attribute("id").as_int();

	ParseKinematics(doc.child("kinematics"), five_axis_conf);
	ParseProcessor(doc.child("processor"), processor_conf);
	ParseCommon(doc.child("common"), common_conf);
}
