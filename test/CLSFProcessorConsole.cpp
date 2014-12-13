// NXProcessorTest.cpp : Defines the entry point for the console application.
//

#include <stdio.h>
#include <tchar.h>
#include <ICLSFProcessor.h>
#include <fstream>


int _tmain(int argc, _TCHAR* argv[])
{
	//while(true)
	//{
		ICLSFProcessor p;
		const char* ss=NULL;
		double fast_moment_time,work_movement_time;
		/*if(argc<2)return 0;
		std::ifstream clsf_file(argv[1],std::ios_base::binary);
		clsf_file.seekg(0,std::ios_base::end);
		size_t size=clsf_file.tellg();
		char* source=new char[size+1];
		clsf_file.seekg(0);
		clsf_file.read(source,size);
		source[size]=0;
		p.CLSFToGCode("jomax3dhead",source,&ss,";header test",";test_prog_id",fast_moment_time,work_movement_time);*/
		//p.CLSFFileToGCodeFile("jomax3dhead",L"TestPaths//_3_PAZ_125_N2.cls",L"test.cnc",";header test",";test_prog_id",fast_moment_time,work_movement_time);
		//p.CLSFFileToGCodeFile("jomax3dhead",L"test.cls",L"test.cnc",";header test",";test_prog_id",fast_moment_time,work_movement_time);
		p.CLSFFileToMeasCodeFile(argv[1],argv[2],_wtof(argv[3]));
	//}
	return 0;
}

