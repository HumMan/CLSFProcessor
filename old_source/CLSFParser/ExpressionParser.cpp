
#include "ExpressionCompiler.h"

#include "CLSFNodesAdapt.h"

#include <boost/config/warning_disable.hpp>
#include <boost/spirit/include/qi.hpp>
#include <boost/spirit/include/phoenix_core.hpp>
#include <boost/spirit/include/phoenix_operator.hpp>
#include <boost/spirit/include/phoenix_fusion.hpp>
#include <boost/spirit/include/phoenix_stl.hpp>
#include <boost/foreach.hpp>

using namespace boost::spirit;
using namespace boost::spirit::iso8859_1;
using boost::spirit::qi::rule;
using boost::spirit::qi::grammar;

template <typename Iterator>
struct expression_grammar : grammar<Iterator, void(), space_type>
{
	expression_grammar() : expression_grammar::base_type(expression)
	{
		using qi::eol;
		using boost::ref;
		expression = *(
			comment	|
			tool_path [op(ref(nodes), _1, compiler::TCLSF_TOOL_PATH())]	|
			tool_data [op(ref(nodes), _1, compiler::TCLSF_TOOL_DATA())]	|
			load |
			select[op(ref(nodes), _1,compiler::TCLSF_SELECT())]	| 
			msys [op(ref(nodes), _1, compiler::TCLSF_MSYS())] |
			paint |
			movement_commands |
			movement_params	| 
			auxfun [op(ref(nodes), _1, compiler::TCLSF_AUXFUN())] |
			end_of_path [op(ref(nodes), compiler::TCLSF_END_OF_PATH())] |
			aux_funcs
			)
			;

		movement_params = 
			rapid [op(ref(nodes), compiler::TCLSF_RAPID())]
		| feedrat [op(ref(nodes), _1, compiler::TCLSF_FEEDRAT())]
		| spindl [op(ref(nodes), _1, compiler::TCLSF_SPINDL())]
		| cutcom [op(ref(nodes), _1, compiler::TCLSF_CUTCOM())];

		movement_commands = 
			( goto_expression [op(ref(nodes), _1, compiler::TCLSF_GOTO())] >> 
			-(',' >> dir [op(ref(nodes), _1, compiler::TCLSF_DIR())]) >> 
			-(contact_point [op(ref(nodes), _1, compiler::TCLSF_CONTACT_POINT())])
			)
			| (from [op(ref(nodes), _1, compiler::TCLSF_FROM())] >>
			-(',' >> dir [op(ref(nodes), _1, compiler::TCLSF_DIR())]))
			|( gohome [op(ref(nodes), _1, compiler::TCLSF_GOHOME())] >>
			-(',' >> dir [op(ref(nodes), _1, compiler::TCLSF_DIR())])
			)
			| (circle [op(ref(nodes), _1, compiler::TCLSF_CIRCLE())] >> 
			goto_expression [op(ref(nodes), _1, compiler::TCLSF_CIRCLE_GOTO())] //конечная точка дуги присутствует в обязательном порядке
		);

		aux_funcs = 
			nx_processor_path_cs_name [op(ref(nodes), _1, compiler::TCLSF_NX_PROCESSOR_SET_CS_G())]	|
			nx_processor_set_cs_g [op(ref(nodes), _1, compiler::TCLSF_NX_PROCESSOR_PATH_CS_NAME())];

		comment = "$$" >> lexeme[+(char_ - eol)];

		tool_path = 
			"TOOL PATH/" >> 
			lexeme[+(char_ - ',')] >> 
			',' >> lexeme[+(char_ - ',')] >> 
			',' >> lexeme[+(char_ - eol)];

		tool_data = "TLDATA/" >>
			lexeme[+(char_ - ',')] >>
			+(',' >> double_ );

		load =	"LOAD/" >> (
			(
			(qi::omit["TOOL"] >> ',' >> int_ )[op(ref(nodes), _1, compiler::TCLSF_LOAD_TOOL())]|
			(qi::omit["ADJUST"] >> ',' >> int_ )[op(ref(nodes), _1, compiler::TCLSF_LOAD_ADJUST())]
		)
			% ',');

		select = "SELECT/" >> lexeme[+(char_ - ',')] >>',' >> double_ ;

		msys =  "MSYS/" >> 
			(double_  % ',');

		paint =	"PAINT/" >> (
			(qi::omit["PATH"])[op(ref(nodes), compiler::TCLSF_PAINT_PATH())]|
			(qi::omit["TOOL"] >> ',' >> "NOMORE")[op(ref(nodes), compiler::TCLSF_PAINT_TOOL())]|
			(qi::omit["COLOR"] >> ',' >> int_ )[op(ref(nodes), _1, compiler::TCLSF_PAINT_COLOR())]|
			(qi::omit["SPEED"] >> ',' >> int_ )[op(ref(nodes), _1, compiler::TCLSF_PAINT_SPEED())]
		);

		rapid = "RAPID";

		goto_expression = 
			"GOTO/" >> 
			double_ >> 
			',' >> double_ >> 
			',' >> double_ ;

		dir = 
			double_ >> 
			',' >> double_ >> 
			',' >> double_ ;

		contact_point = "$$" >> 
			double_ >> 
			',' >> double_ >> 
			',' >> double_ ;

		from = 
			"FROM/" >> 
			double_ >> 
			',' >> double_ >> 
			',' >> double_ ;

		gohome = 
			"GOHOME/" >> 
			double_ >> 
			',' >> double_ >> 
			',' >> double_ ;

		circle = "CIRCLE/" >> 
			double_ >> ',' >> 
			double_ >> ',' >> 
			double_ >> ',' >> 
			double_ >> ',' >> 
			double_ >> ',' >> 
			double_ >> ',' >> 
			(double_ % ',');  

		feedrat = "FEDRAT/" >> 
			qi::omit[+(char_ - ',')] >> //TODO может исчезать
			',' >> double_ ;

		spindl = "SPINDL/" >> 
			lexeme[+(char_ - ',')] >>
			',' >> double_ >> ',' >>
			lexeme[+(char_ - eol)];

		cutcom = "CUTCOM/" >> 
			lexeme[+(char_ - ',' - eol)] >> - (',' >> lexeme[+(char_ - eol)]);

		auxfun = "AUXFUN/" >> 
			int_ >> 
			lexeme[*(char_ - eol)];

		end_of_path = "END-OF-PATH";

		//nx connector generated events

		nx_processor_path_cs_name = "NX_PROCESSOR_PATH_CS_NAME/" >> 
			lexeme[+(char_ - eol)];

		nx_processor_set_cs_g = "NX_PROCESSOR_SET_CS_G/" >> 
			lexeme[+(char_ - eol)];

	}
	rule<Iterator, void(), space_type> expression,load,paint,rapid,aux_funcs,movement_commands,movement_params;

	rule<Iterator, TCLSF_VEC3(), space_type>  contact_point;
	rule<Iterator, TCLSF_TOOL_PATH(), space_type> tool_path;
	rule<Iterator, TCLSF_TOOL_DATA(), space_type> tool_data;
	rule<Iterator, TCLSF_SELECT(), space_type> select;
	rule<Iterator, TCLSF_MSYS(), space_type> msys;
	rule<Iterator, TCLSF_VEC3(), space_type>  goto_expression,dir,from,gohome;
	rule<Iterator, TCLSF_CIRCLE(), space_type> circle;
	rule<Iterator, TCLSF_FEEDRAT(), space_type> feedrat;
	rule<Iterator, TCLSF_SPINDL(), space_type> spindl;
	rule<Iterator, TCLSF_CUTCOM(), space_type> cutcom;
	rule<Iterator, TCLSF_AUXFUN(), space_type> auxfun;
	rule<Iterator, void(), space_type> end_of_path;

	//nx connector generated events
	rule<Iterator, std::string(), space_type> 
		nx_processor_path_cs_name,
		nx_processor_set_cs_g,
		comment;
	//

	boost::phoenix::function<compiler> op;
	CLSFNodes nodes;
};

int parse_clsf(const std::string &use_clsf,TProgram &destination,int &error_line)
{
	std::string::const_iterator iter=use_clsf.begin(),end=use_clsf.end();
	expression_grammar<std::string::const_iterator> grammar;//TODO куда нибудь чтобы часто не инициализировался парсер грамматики
	grammar.nodes.program=&destination;
	bool result=phrase_parse(iter, end, grammar, space);
	if(iter==end)
		error_line=-1;
	else
	{
		std::string::const_iterator temp=use_clsf.begin();
		error_line=1;
		for(;temp!=iter;temp++)
		{
			if(*temp=='\n')error_line++;
		}
	}
	return  !(result && (iter == end));
}

int TProgram::ToXMLFile(const std::wstring &fname)
{
	//TODO
	return 0;
}

int TProgram::FromXMLFile(const std::wstring &fname)
{
	//TODO
	return 0;
}