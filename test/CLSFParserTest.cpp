// NXProcessorTest.cpp : Defines the entry point for the console application.

//


#include "ICLSFProcessor.h"

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE Main

#include <boost/test/unit_test.hpp>

//#define BOOST_TEST_MODULE CLSFProcessorTest 
//#include <boost/test/unit_test.hpp>
 

 
 
 class Calculator
{
public:
	explicit Calculator(int value)
		: Value_(value)
	{
	}
 
	void Divide(int value)
	{
		if (value == 0)
		{
			throw std::invalid_argument("Деление на ноль!");
		}
		Value_ /= value;
	}
 
	void Multiply(int value)
	{
		Value_ *= value;
	}
 
	int Result() const
	{
		return Value_;
	}
 
private:
	int Value_;
};
 
BOOST_AUTO_TEST_CASE(CLSFParserTest)
{
		ICLSFProcessor p;
		const char* ss=NULL;
		double fast_moment_time,work_movement_time;
		//p.CLSFFileToGCodeFile("jomax3dhead",L"test.cls",L"test.cnc",";header test",";test_prog_id",fast_moment_time,work_movement_time);
}
BOOST_AUTO_TEST_CASE(testCalculator2)
{
	Calculator calculator(12);
	BOOST_CHECK_EQUAL(calculator.Result(), 12);
	calculator.Divide(3);
	BOOST_CHECK_EQUAL(calculator.Result(), 4);
	calculator.Divide(2);
	BOOST_CHECK_EQUAL(calculator.Result(), 2);
	calculator.Multiply(2);
	BOOST_CHECK_EQUAL(calculator.Result(), 4);
	calculator.Multiply(3);
	BOOST_CHECK_EQUAL(calculator.Result(), 12);
}
// #include <stdio.h>
// #include <tchar.h>
// #include <ICLSFProcessor.h>
// #include <fstream>

// int _tmain(int argc, _TCHAR* argv[])
// {
	// //while(true)
	// //{
		// ICLSFProcessor p;
		// const char* ss=NULL;
		// double fast_moment_time,work_movement_time;
		// /*if(argc<2)return 0;
		// std::ifstream clsf_file(argv[1],std::ios_base::binary);
		// clsf_file.seekg(0,std::ios_base::end);
		// size_t size=clsf_file.tellg();
		// char* source=new char[size+1];
		// clsf_file.seekg(0);
		// clsf_file.read(source,size);
		// source[size]=0;
		// p.CLSFToGCode("jomax3dhead",source,&ss,";header test",";test_prog_id",fast_moment_time,work_movement_time);*/
		// //p.CLSFFileToGCodeFile("jomax3dhead",L"TestPaths//_3_PAZ_125_N2.cls",L"test.cnc",";header test",";test_prog_id",fast_moment_time,work_movement_time);
		// p.CLSFFileToGCodeFile("jomax3dhead",L"test.cls",L"test.cnc",";header test",";test_prog_id",fast_moment_time,work_movement_time);
	// //}
	// return 0;
// }

