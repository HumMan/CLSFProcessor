#include "CLSFNodes.h"

#include <boost/spirit/include/qi.hpp>

struct compiler
{
	template <typename A, typename B = boost::spirit::unused_type, typename C = boost::spirit::unused_type, typename D = boost::spirit::unused_type>
	struct result { typedef void type; };

	void PrepareCommandInsertion(CLSFNodes &nodes,TVector<TMovement>* &movements,TVector<TMovementContour>* &contours,boost::ptr_vector<TMovementCommand>* &commands)const;

	struct TCLSF_COMMENT{};
	void operator()(CLSFNodes &nodes, ::TCLSF_COMMENT const &params,compiler::TCLSF_COMMENT) const;
	
	struct TCLSF_TOOL_PATH{};
	void operator()(CLSFNodes &nodes, ::TCLSF_TOOL_PATH const &params,compiler::TCLSF_TOOL_PATH) const;

	struct TCLSF_TOOL_DATA{};
	void operator()(CLSFNodes &nodes, ::TCLSF_TOOL_DATA const &params,compiler::TCLSF_TOOL_DATA) const;

	struct TCLSF_LOAD_TOOL{};
	void operator()(CLSFNodes &nodes, int const &params,compiler::TCLSF_LOAD_TOOL) const;

	struct TCLSF_LOAD_ADJUST{};
	void operator()(CLSFNodes &nodes, int const &params,compiler::TCLSF_LOAD_ADJUST) const;

	struct TCLSF_SELECT{};
	void operator()(CLSFNodes &nodes, ::TCLSF_SELECT const &params,compiler::TCLSF_SELECT) const;

	struct TCLSF_MSYS{};
	void operator()(CLSFNodes &nodes, ::TCLSF_MSYS const &params,compiler::TCLSF_MSYS) const;

	struct TCLSF_PAINT_COLOR{};
	void operator()(CLSFNodes &nodes, int const &params,compiler::TCLSF_PAINT_COLOR) const;

	struct TCLSF_PAINT_PATH{};
	void operator()(CLSFNodes &nodes,compiler::TCLSF_PAINT_PATH) const;

	struct TCLSF_PAINT_TOOL{};
	void operator()(CLSFNodes &nodes,compiler::TCLSF_PAINT_TOOL) const;

	struct TCLSF_PAINT_SPEED{};
	void operator()(CLSFNodes &nodes, int const &params,compiler::TCLSF_PAINT_SPEED) const;

	struct TCLSF_RAPID{};
	void operator()(CLSFNodes &nodes,compiler::TCLSF_RAPID) const;

	struct TCLSF_GOTO{};
	void operator()(CLSFNodes &nodes, ::TCLSF_VEC3 const &params,compiler::TCLSF_GOTO) const;

	struct TCLSF_CIRCLE_GOTO{};
	void operator()(CLSFNodes &nodes, ::TCLSF_VEC3 const &params,compiler::TCLSF_CIRCLE_GOTO) const;

	struct TCLSF_DIR{};
	void operator()(CLSFNodes &nodes, ::TCLSF_VEC3 const &params,compiler::TCLSF_DIR) const;

	struct TCLSF_CONTACT_POINT{};
	void operator()(CLSFNodes &nodes, ::TCLSF_VEC3 const &params,compiler::TCLSF_CONTACT_POINT) const;

	struct TCLSF_FROM{};
	void operator()(CLSFNodes &nodes, ::TCLSF_VEC3 const &params,compiler::TCLSF_FROM) const;

	struct TCLSF_GOHOME{};
	void operator()(CLSFNodes &nodes, ::TCLSF_VEC3 const &params,compiler::TCLSF_GOHOME) const;

	struct TCLSF_CIRCLE{};
	void operator()(CLSFNodes &nodes, ::TCLSF_CIRCLE const &params,compiler::TCLSF_CIRCLE) const;

	struct TCLSF_FEEDRAT{};
	void operator()(CLSFNodes &nodes, ::TCLSF_FEEDRAT const &params,compiler::TCLSF_FEEDRAT) const;

	struct TCLSF_SPINDL{};
	void operator()(CLSFNodes &nodes, ::TCLSF_SPINDL const &params,compiler::TCLSF_SPINDL) const;

	struct TCLSF_CUTCOM{};
	void operator()(CLSFNodes &nodes, ::TCLSF_CUTCOM const &params,compiler::TCLSF_CUTCOM) const;

	struct TCLSF_AUXFUN{};
	void operator()(CLSFNodes &nodes, ::TCLSF_AUXFUN const &params,compiler::TCLSF_AUXFUN) const;

	struct TCLSF_END_OF_PATH{};
	void operator()(CLSFNodes &nodes, compiler::TCLSF_END_OF_PATH) const;

	//nx connector generated events
	struct TCLSF_NX_PROCESSOR_SET_CS_G{};
	void operator()(CLSFNodes &nodes, std::string const &params,compiler::TCLSF_NX_PROCESSOR_SET_CS_G) const;

	struct TCLSF_NX_PROCESSOR_PATH_CS_NAME{};
	void operator()(CLSFNodes &nodes, std::string const &params,compiler::TCLSF_NX_PROCESSOR_PATH_CS_NAME) const;	
};