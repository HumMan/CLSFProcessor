
#include <ICLSFProcessor.h>

#include <stdlib.h>
#include <malloc.h>
#include <memory.h>
//#include <tchar.h>
#include <stdio.h>
//#include <conio.h>
#include <iostream>
#include <fstream>
#include <map>
using namespace std;

#include "CLSFParser/ExpressionParser.h"
#include "Uni5axis.h"

class ICLSFProcessorPrivate
{
	friend class ICLSFProcessor;
	string gcode;
};

ICLSFProcessor::ICLSFProcessor():p(new ICLSFProcessorPrivate)
{
}

ICLSFProcessor::~ICLSFProcessor()
{
	delete p;
	p=NULL;
}

void PutMeasCommand(string &program, char buf[],Vector3d pos,Vector3d dir,Vector3d contact,double ball_diameter)
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
		"mpos[0]=set(%.5f,%.5f,%.5f)\n"
		"dir[0]=set(%.5f,%.5f,%.5f)\n"
		"contact[0]=set(%.5f,%.5f,%.5f)\n"
		"GMO_BALLPROBE(mpos,dir,contact,ball_diameter*0.5)\n",
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

                //clsf_program.Save(doc);

		TToolPath& path=clsf_program.tool_paths.front();
		string program("");
		//clsf_program.GetGCode(program);
		char buf[10000];	

		if(path.tool_data.tool_type=="MILL")
		{
			if(path.tool_data.params.size()<1)return 3;
		}
		else return 2;

		sprintf(buf,
			"extern GMO_MPOINT(var real[3],var real[3],var real[3])\n"
			"extern GMO_WRCONTACT(var real[3],var real[3],var real[3])\n"
			"def real mpos[3]=set(0,0,0)\n"
			"def real dir[3]=set(0,0,1)\n"
			"def real contact[3]=set(0,0,0)\n"
			"def real ball_diameter=%.5f\n"
			"G54\n"
			"G01 F2000\n",
			path.tool_data.params[0]);
		program+=buf;

		bool last_pos_initialized=false;
		Vector3d last_pos;

		std::list<TMovement>::iterator movement;
		std::list<TMovementContour>::iterator contour;

		for(movement=path.movements.begin();movement!=path.movements.end();movement++)
		{
			
			for(contour=movement->contours.begin();contour!=movement->contours.end();contour++)
			{
				boost::ptr_vector<TMovementCommand>::iterator comm;
				for(comm=contour->commands.begin();comm!=contour->commands.end();comm++)
				{
					Vector3d dir;
					if(comm->HasDir())
						dir=comm->GetDir();
					if(movement->type==TMovement::CUT)
					{
						if(comm->HasContactPoint()&&((!last_pos_initialized)||(last_pos_initialized&&(last_pos-comm->pos).norm()>=min_step)))
						{
							last_pos_initialized=true;
							PutMeasCommand(program,buf,comm->pos,dir,comm->GetContactPoint(),path.tool_data.params[0]);
							last_pos=comm->pos;
						}
					}
				}
			}
		}

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
#ifdef WIN
		std::ifstream clsf_file(use_clsf_path,std::ios::binary);
#else
		std::ifstream clsf_file(pugi::as_utf8(use_clsf_path).c_str(),std::ios::binary);
#endif
		if(clsf_file.fail())return 2;
		clsf_file.seekg( 0, ios_base::end );
		size_t size=clsf_file.tellg();
		clsf_file.seekg( 0, ios_base::beg );

		char* clsf=new char[size+1];
		clsf_file.read(clsf,size);
		clsf[size]=0;

		const char* result_gcode=0;
		CLSFToMeasCode(clsf,&result_gcode,min_step);
#ifdef WIN
		std::ofstream result_file(result_gcode_path,std::ios_base::binary);
#else
		std::ofstream result_file(pugi::as_utf8(result_gcode_path).c_str(),std::ios_base::binary);
#endif
		result_file.write(result_gcode,strlen(result_gcode));
	}
	catch(exception e)
	{
		return 1;
	}
	return 0;

}

struct TMotionPrivate
{
	TProgram program;
};

TMotion::TMotion(pugi::xml_node from_xml)
{
	p->program.Save(from_xml);
}

TMotion::TMotion(const std::wstring& use_path,TMotionFormat format)
{
	Load(use_path,format);
}

void TMotion::Save(const std::wstring& use_path,TMotionFormat format)
{
	switch(format)
	{
	case FORMAT_XML:
		{
			pugi::xml_document doc;
			Save(doc);
			bool result = doc.save_file(use_path.c_str());
		}
	case FORMAT_CLSF:
		{
			//TODO
		}
	}
}

void TMotion::Save(pugi::xml_node to_xml)
{
	p->program.Save(to_xml);
}

void TMotion::Load(const std::wstring& use_path,TMotionFormat format)
{
	switch(format)
	{
	case FORMAT_XML:
		{
			pugi::xml_document doc;
			pugi::xml_parse_result result = doc.load_file(use_path.c_str());

			Load(doc);
		}
	case FORMAT_CLSF:
		{
#ifdef WIN
			std::ifstream clsf_file(use_path.c_str(),std::ios::binary);
#else
			std::ifstream clsf_file(pugi::as_utf8(use_path.c_str()).c_str(),std::ios::binary);
#endif
			//if(clsf_file.fail())return 2;
			clsf_file.seekg( 0, ios_base::end );
			size_t size=clsf_file.tellg();
			clsf_file.seekg( 0, ios_base::beg );

			char* clsf=new char[size+1];
			clsf_file.read(clsf,size);
			clsf[size]=0;

			string s(clsf);
			delete clsf;
			int error_line=0;
			parse_clsf(s,p->program,error_line);
		}
	}
}

void TMotion::Load(pugi::xml_node from_xml)
{
	p->program.Load(from_xml);
}

TMotion::~TMotion()
{
}
