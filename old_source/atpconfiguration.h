#pragma once

#include <boost/lexical_cast.hpp>
#include <boost/program_options/detail/config_file.hpp>
#include <boost/program_options/parsers.hpp>
#include <map>
#include <set>
using boost::lexical_cast;
using boost::bad_lexical_cast;

TVec<double,3> ParseVec(const string& s)
{
	int c0=s.find(",");
	int c1=s.find(",",c0+1);
	TVec<double,3> r;
	string temp=s.substr(1,c0-1);
	r[0]= lexical_cast<double>(temp);
	temp=s.substr(c0+1,c1-c0-1);
	r[1]= lexical_cast<double>(temp);
	temp=s.substr(c1+1,s.length()-c1-2);
	r[2]= lexical_cast<double>(temp);
	return r;
}