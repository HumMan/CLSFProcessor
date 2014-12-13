
#include <ICLSFProcessor.h>

#include <stdlib.h>
#include <malloc.h>
#include <memory.h>
#include <tchar.h>
#include <stdio.h>
#include <conio.h>
#include <iostream>
#include <map>
using namespace std;

#include <tinyxml.h>

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
	TATPTokenizer atp_tokens(use_clsf);
	TVector<TToolMovementElement<double>> result_pipe;
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

void PutMeasCommand(string &program, char buf[],TVec3d pos,TVec3d dir,TVec3d contact,double ball_diameter)
{
	const double min_delta=2.0; //2 мм максимум можем войти в тело
	const double pre_collide_delta=2.0; //расстояние на котором включаем контроль столкновений
	const double max_delta=4;

	//TVec3d 
	//	normal,
	//	ball_center;

	//ball_center=pos+dir*ball_diameter*0.5;
	//normal=(ball_center-contact).GetNormalized();

	//TODO сразу давать отклонение

	//sprintf(buf,
	//	"normal[0]=set(%.5f,%.5f,%.5f)\n"
	//	"pos[0]=set(%.5f,%.5f,%.5f)\n"
	//	"ball_center[0]=set(%.5f,%.5f,%.5f)\n"
	//	"dir[0]=set(%.5f,%.5f,%.5f)\n"
	//	"contact[0]=set(%.5f,%.5f,%.5f)\n"
	//	"GMO_MPOINT(ball_center,normal,measpos)\n"
	//	"GMO_MADD(measpos,dir,ball_rad,measpos)\n"
	//	"GMO_WRCONTACT(pos,normal,meaball_centerspos,dir,contact,measpos)\n",
	//	(float)normal[0],(float)normal[1],(float)normal[2],
	//	(float)pos[0],(float)pos[1],(float)pos[2],
	//	(float)ball_center[0],(float)ball_center[1],(float)ball_center[2],
	//	(float)dir[0],(float)dir[1],(float)dir[2],
	//	(float)contact[0],(float)contact[1],(float)contact[2]);

	sprintf(buf,
		"pos[0]=set(%.5f,%.5f,%.5f)\n"
		"dir[0]=set(%.5f,%.5f,%.5f)\n"
		"contact[0]=set(%.5f,%.5f,%.5f)\n"
		"GMO_BALLPROBE(pos,dir,contact,ball_diameter*0.5)\n",
		(float)pos[0],(float)pos[1],(float)pos[2],
		(float)dir[0],(float)dir[1],(float)dir[2],
		(float)contact[0],(float)contact[1],(float)contact[2]);

	program+=buf;
}

int ICLSFProcessor::CLSFToMeasCode(
								   char* const use_clsf,
								   const char** result_gcode,
								   double min_step
								   )
{
	try
	{
		pugi::xml_document doc;
		pugi::xml_parse_result result = doc.load_file("1.xml");

		std::string s(use_clsf);
		TProgram clsf_program;
		int error_line=0;
		parse_clsf(use_clsf,clsf_program,error_line);

		TToolPath& path=clsf_program.tool_paths.front();
		string program("");
		clsf_program.GetGCode(program);
		char buf[10000];	

		if(path.tool_data.tool_type=="MILL")
		{
			if(path.tool_data.params.size()<1)return 3;
		}
		else return 2;

		sprintf(buf,
			"extern GMO_MPOINT(var real[3],var real[3],var real[3])\n"
			"extern GMO_WRCONTACT(var real[3],var real[3],var real[3])\n"
			"def real pos[3]=set(0,0,0)\n"
			"def real dir[3]=set(0,0,1)\n"
			"def real contact[3]=set(0,0,0)\n"
			"def real ball_diameter=%.5f\n"
			"G54\n"
			"G01 F2000\n",
			path.tool_data.params[0]);
		program+=buf;

		TVec3d last_pos;
		bool last_pos_initialized=false;

		TVec3d dir;

		//расскоментировать после рефакторинга
		//for(int i=0;i<=path.movement.GetHigh();i++)
		//{
		//	for(int k=0;k<=path.movement[i].commands.GetHigh();k++)
		//	{
		//		TMovementCommand& comm=path.movement[i].commands[k];
		//		if(comm.has_dir)
		//			dir=comm.dir;
		//		if(path.movement[i].type==TMovement::CUT||path.movement[i].type==TMovement::RAPID)
		//		{
		//			if(comm.has_contact_point&&((!last_pos_initialized)||(last_pos_initialized&&last_pos.Distance(comm.pos)>=min_step)))
		//			{
		//				last_pos_initialized=true;
		//				PutMeasCommand(program,buf,comm.pos,dir,comm.contact_point,path.tool_data.params[0]);
		//				last_pos=comm.pos;
		//			}
		//		}
		//	}
		//}

		program+="G500\nZ0 F4000\n";
		program+="M2\nM30\n";

		{
			stringstream str(program);
			string temp;
			stringstream ostr(temp);
			int curr_frame=0;
			while(!str.eof())
			{
				char buff[100];
				sprintf(buff,"N%i ",curr_frame);
				curr_frame+=10;

				char line[1024];
				str.getline(line,1023);
				ostr << buff << line <<'\n';
			}

			program=ostr.str();
		}

		p->gcode=program;
		*result_gcode=p->gcode.c_str();
	}
	catch(exception e)
	{
		return 1;
	}
	return 0;

}

int ICLSFProcessor::CLSFFileToMeasCodeFile(
	const wchar_t* use_clsf_path,
	const wchar_t* result_gcode_path,
	double min_step
	)
{
	try
	{
		std::ifstream clsf_file(use_clsf_path,std::ios::binary);
		if(clsf_file.fail())return 2;
		clsf_file.seekg( 0, ios_base::end );
		size_t size=clsf_file.tellg();
		clsf_file.seekg( 0, ios_base::beg );

		char* clsf=new char[size+1];
		clsf_file.read(clsf,size);
		clsf[size]=0;

		const char* result_gcode=0;
		CLSFToMeasCode(clsf,&result_gcode,min_step);
		std::ofstream result_file(result_gcode_path,std::ios_base::binary);
		result_file.write(result_gcode,strlen(result_gcode));
	}
	catch(exception e)
	{
		return 1;
	}
	return 0;

}