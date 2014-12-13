#pragma once

#include <boost/filesystem.hpp>
#include <boost/tokenizer.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/format.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/program_options/detail/config_file.hpp>
#include <boost/program_options/parsers.hpp>

inline void ParseIniFile(const char* fname,std::map<string,string> &ini_params)
{
	ifstream ini_file(fname);
	std::set<string> options;
	options.insert("*");
	try
	{
		for(boost::program_options::detail::config_file_iterator i(ini_file,options),e;i!=e;i++)
		{
			ini_params[i->string_key]=i->value[0];
		}
	}catch(exception e)
	{
		cout << "Error parsing ini file";
	}
}