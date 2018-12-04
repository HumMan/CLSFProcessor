#pragma once

#include "FiveAxisKinematics.h"

namespace CLSFProcessor
{

	//void GetCode(std::vector<TToolMovementElement> &pipe, std::string &result_code, const char* ext_header, const char* prog_id);
	void GetGCode(CLSFProcessor::Conf::TCommon common_conf, CLSFProcessor::Conf::TProcessor processor_conf, std::vector<TToolMovementElement> &pipe, const char* result_path);
}