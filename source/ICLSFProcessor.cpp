
#include <ICLSFProcessor.h>

#include <stdlib.h>
#include <malloc.h>
#include <memory.h>
#include <tchar.h>
#include <stdio.h>
#include <conio.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <map>
using namespace std;

#include "FiveAxisKinematics.h"
#include "config.h"

#include "CLSFParser/ExpressionParser.h"

#include <pugixml.hpp>

class ICLSFProcessorPrivate
{
	friend class ICLSFProcessor;
	string gcode;
};

ICLSFProcessor::ICLSFProcessor():p(new ICLSFProcessorPrivate)
{
}

ICLSFProcessor::~ICLSFProcessor(void)
{
	delete p;
	p=NULL;
}

int ICLSFProcessor::CLSFToGCode(
								const char* use_machine_config,
								char* const use_clsf,
								const char** result_gcode,
								const char* ext_header,
								const char* prog_id,
								double &fast_movement_time,
								double &work_movement_time
								)
{
	std::map<string,string> ini_params;
	char config_fname[256];
	sprintf(config_fname,"%s.ini",use_machine_config);
	ParseIniFile(config_fname,ini_params);
	TATPProcessor<double,TUniversal5axis<double>> atp_processor(ini_params);
	TATPTokenizer atp_tokenenizer;
	auto atp_tokens = atp_tokenenizer.Parse(use_clsf);
	std::vector<TToolMovementElement<double>> result_pipe;
	atp_processor.PassThrough(atp_tokens,result_pipe,fast_movement_time,work_movement_time);
	atp_processor.GetCode(result_pipe,p->gcode,ext_header,prog_id);
	(*result_gcode)=p->gcode.c_str();
	return 0;

}

int ICLSFProcessor::CLSFFileToGCodeFile(
										const char* use_machine_config,
										const wchar_t* use_clsf_path,
										const wchar_t* result_gcode_path,
										const char* ext_header,
										const char* prog_id,
										double &fast_movement_time,
										double &work_movement_time
										)
{
	std::ifstream clsf_file(use_clsf_path,std::ios::binary);
	clsf_file.seekg( 0, ios_base::end );
	size_t size=clsf_file.tellg();
	clsf_file.seekg( 0, ios_base::beg );

	char* clsf=new char[size+1];
	clsf_file.read(clsf,size);
	clsf[size]=0;

	const char* result_gcode=0;
	int err_code=CLSFToGCode(use_machine_config,clsf,&result_gcode,ext_header,prog_id,fast_movement_time,work_movement_time);
	if(err_code!=0)return err_code;
	std::ofstream result_file(result_gcode_path,std::ios_base::binary);
	result_file.write(result_gcode,strlen(result_gcode));
	return 0;

}
