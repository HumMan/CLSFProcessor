#pragma once

#include <map>
#include <string>

inline void ParseIniFile(const char* fname,std::map<string,string> &ini_params)
{
	ifstream ini_file(fname);

	ini_file.seekg(0, std::ios_base::end);
	size_t size = ini_file.tellg();
	char* source = new char[size + 1];
	ini_file.seekg(0);
	ini_file.read(source, size);
	source[size] = 0;
	
	string currKey, currValue;
	bool isKey = true;
	for (int i = 0; i < size; i++)
	{
		if (source[i] == '=')
		{
			isKey = false;			
		}
		else if (source[i] == '\n')
		{
			ini_params[currKey] = currValue;

			currKey = "";
			currValue = "";
			isKey = true;
		}
		else if (source[i] == '\t'|| source[i] == '\r')
		{
		}
		else if (source[i] == '#')
		{

		}
		else
		{
			if(isKey)
				currKey += source[i];
			else
				currValue += source[i];
		}

	}

	//std::set<string> options;
	//options.insert("*");
	//try
	//{
	//	for(boost::program_options::detail::config_file_iterator i(ini_file,options),e;i!=e;i++)
	//	{
	//		ini_params[i->string_key]=i->value[0];
	//	}
	//}catch(exception e)
	//{
	//	cout << "Error parsing ini file";
	//}
}