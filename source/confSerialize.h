#pragma once


#include "commonConf.h"
#include "kinematicsConf.h"

namespace CLSFProcessor
{
	void ParseConfig(const std::string& config_path, Conf::TCommon& common_conf, Conf::TProcessor& processor_conf, Conf::TFiveAxis& five_axis_conf);
}