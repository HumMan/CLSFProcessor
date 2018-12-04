// NXProcessorTest.cpp : Defines the entry point for the console application.
//

#include <stdio.h>
#include <tchar.h>

#include "refactored\atptokenizer.h"
#include "refactored\confSerialize.h"
#include "refactored\Uni5axis.h"
#include "refactored/FiveAxisKinematics.h"
#include "refactored\formatter.h"

int _tmain(int argc, _TCHAR* argv[])
{
	_TCHAR* config_path = argv[1];
	_TCHAR* source_path = argv[2];
	_TCHAR* result_path = argv[3];
	auto tokens = CLSFProcessor::ParsePath(source_path);

	CLSFProcessor::Conf::TCommon common_conf;
	CLSFProcessor::Conf::TFiveAxis five_axis_conf;
	CLSFProcessor::Conf::TProcessor processor_conf;

	CLSFProcessor::ParseConfig(config_path, common_conf, processor_conf, five_axis_conf);

	CLSFProcessor::TATPProcessor processor(common_conf, five_axis_conf, processor_conf);
	std::vector<CLSFProcessor::TToolMovementElement> result;
	double fast_movement_time;
	double work_movement_time;
	processor.PassThrough(tokens, result, fast_movement_time, work_movement_time);

	std::string code;
	CLSFProcessor::GetGCode(common_conf, processor_conf, result, result_path);

	return 0;
}

