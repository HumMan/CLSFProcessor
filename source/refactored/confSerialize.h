#pragma once


#include "commonConf.h"
#include "kinematicsConf.h"

namespace CLSFProcessor
{
	void ParseConfig(const char* config_path, Conf::TCommon& common_conf, Conf::TFiveAxis& five_axis_conf);
}