#pragma once

#include "FiveAxisKinematics.h"

namespace CLSFProcessor
{

	//void GetCode(std::vector<TToolMovementElement> &pipe, std::string &result_code, const char* ext_header, const char* prog_id);
	void GetGCode(std::vector<TToolMovementElement> &pipe, std::string& result_code);
}