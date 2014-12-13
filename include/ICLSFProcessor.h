#pragma once

#include <stdlib.h>
#include <vector>
#include <string>
#include <pugixml.hpp>

#ifdef ICLSFPROCESSOR_EXPORTS
	#if _WINDOWS
	#define ICLSFPROCESSOR_API __declspec(dllexport)
	#else
	#define ICLSFPROCESSOR_API
	#endif
#else
#define ICLSFPROCESSOR_API
#endif

//!  Класс обрабатывающий CLSF
/*!
  ICLSFProcessor занимается преобразованием CLSF в промежуточный формат и процессингом траектории.
*/

class TMotionPrivate;

class ICLSFPROCESSOR_API TMotion
{
	TMotionPrivate *p;
	enum TMotionFormat
	{
		FORMAT_XML,
		FORMAT_CLSF
	};
public:
	TMotion(pugi::xml_node from_xml);
	TMotion(const std::wstring& use_path,TMotionFormat format);
	void Save(const std::wstring& use_path,TMotionFormat format);
	void Save(pugi::xml_node to_xml);
	void Load(const std::wstring& use_path,TMotionFormat format);
	void Load(pugi::xml_node from_xml);
    ~TMotion();
};

class ICLSFProcessorPrivate;

class ICLSFPROCESSOR_API ICLSFProcessor
{
	ICLSFProcessorPrivate* p;
public:
	ICLSFProcessor();
	~ICLSFProcessor();

	//int CLSFToGCode(const char* use_machine_config,char*const use_clsf,const char** result_gcode,const char* ext_header,const char* prog_id,double &fast_movement_time,double &work_movement_time);
	//int CLSFFileToGCodeFile(const char* use_machine_config,const wchar_t* use_clsf_path,const wchar_t* result_gcode_path,const char* ext_header,const char* prog_id,double &fast_movement_time,double &work_movement_time);
	
	int CLSFToMeasCode(char*const use_clsf,const char** result_gcode,double min_step);
	int CLSFFileToMeasCodeFile(const wchar_t* use_clsf_path,const wchar_t* result_gcode_path,double min_step);

};
