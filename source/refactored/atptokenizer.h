#pragma once


#include <string.h>
#include <vector>
#include <Eigen/Dense>

namespace CLSFProcessor
{
	class TCLSFToken
	{
	private:
		std::string name;
		std::vector<std::string> params;
	public:
		void AddParam(std::string value)
		{
			params.push_back(value);
		}
		void SetName(std::string value)
		{
			this->name = value;
		}
		std::string Name()
		{
			return name;
		}
		size_t ParamsCount()
		{
			return params.size();
		}
		std::string& operator[](size_t i)
		{
			return params[i];
		}
	};

	std::vector<TCLSFToken> Parse(const char* use_source);
	std::vector<TCLSFToken> ParsePath(const char* use_file_path);
}