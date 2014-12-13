#pragma once

#include <BaluLib.h>
#include <string>
#include <vector>
#include <list>
#include <boost/ptr_container/ptr_vector.hpp>

struct TCLSF_COMMENT
{
	std::string text;
};

struct TCLSF_TOOL_PATH
{
	std::string path_name;
	std::string tool_type;
	std::string tool_name;
};

struct TCLSF_TOOL_DATA
{
	std::string tool_type;
	std::vector<double> params;
};

struct TCLSF_SELECT
{
	std::string name;
	int id;
};

struct TCLSF_MSYS
{
	std::vector<double> params;
};

struct TCLSF_GOTO
{
	TVec3d pos;
	bool has_dir;
	TVec3d dir;
	bool has_contact_point;
	TVec3d contact_point;
};

struct TCLSF_VEC3
{
	TVec3d v;
};

struct TCLSF_FROM
{
	TVec3d pos;
};

struct TCLSF_GOHOME
{
	TVec3d pos;
};

struct TCLSF_CIRCLE
{
	TVec3d 
		center,
		normal,
		pos;
	std::vector<double> params;
	bool has_spiral_times;
	int spiral_times;
};

struct TCLSF_FEEDRAT
{
	//std::string type;
	double feed_rate;
};

struct TCLSF_SPINDL
{
	std::string type;
	double rate;
	std::string dir;
};

struct TCLSF_CUTCOM
{
	std::string type;
	std::string plane;
};

struct TCLSF_AUXFUN
{
	int id;
	std::string text;
};

struct TCLSF_END_OF_PATH
{
};

class TMovementCommand : protected boost::noncopyable
{
private:
	virtual TMovementCommand* do_clone() const = 0;
protected:
    TMovementCommand( const TMovementCommand& r ) { }
    void operator=( const TMovementCommand& );
public:
	bool rapid;
	bool feed_change;
	double cut_feed;
	TVec3 pos;
	double rot[2];
	TMovementCommand()
		:rapid(false)
		,cut_feed(0)
	{}
	virtual ~TMovementCommand(){}
	virtual void SetDir(const TVec3d& use_dir)=0;
	virtual void SetEndPos(const TVec3d& use_pos)=0;
	virtual void SetContact(const TVec3d& use_contact)=0;
	virtual void GetGCode(std::string& program)=0;
	
	TMovementCommand* clone() const
    {
        return do_clone();
    }
};

inline TMovementCommand* new_clone( const TMovementCommand& a )
{
    return a.clone();
}

struct TMovementContourParams
{
	bool has_compensation;
	bool compensation_type;
	int compensation_plane;
	bool contour_params_change;
	TMovementContourParams()
		:has_compensation(false)
		,compensation_type(false)
		,compensation_plane(0)
		,contour_params_change(false)
	{}
	bool operator!=(const TMovementContourParams& use_right)const
	{
		if(has_compensation)
			return 
			(compensation_type!=use_right.compensation_type)||
			(compensation_plane!=use_right.compensation_plane);
		else
			return has_compensation!=use_right.has_compensation;

	}
};

struct TMovementContour: public TMovementContourParams
{
	boost::ptr_vector<TMovementCommand> commands;
};

struct TMovement
{
	enum TMovementType
	{
		RAPID,
		APPROACH,
		ENGAGE,
		CUT,
		RETRACT,
		DEPARTURE
	};
	TMovementType type;
	TVector<TMovementContour> contours;
};

struct TToolPath
{
	TCLSF_TOOL_PATH path_data;
	TCLSF_TOOL_DATA tool_data;
	TCLSF_MSYS msys;
	TVector<TMovement> movements;
	bool has_spindl_rpm;
	double spindl_rpm;
	bool has_tool_id;
	int tool_id;
	bool has_tool_adjust;
	int tool_adjust;
	TToolPath()
		:has_spindl_rpm(false)
		,spindl_rpm(0)
		,has_tool_id(false)
		,tool_id(0)
		,has_tool_adjust(false)
		,tool_adjust(0)
	{}
};

struct TProgram
{
	std::list<TToolPath> tool_paths;
	void GetGCode(std::string& program);
	int ToXMLFile(const std::wstring &fname);
	int FromXMLFile(const std::wstring &fname);
};

struct CLSFNodes
{
	enum ProgramParseStates
	{
		STATE_PROGRAM,
		STATE_UDE,
		STATE_TOOL_PATH,
		STATE_TOOL_PATH_UDE
	}curr_state;

	bool rapid;
	TMovement::TMovementType curr_move_type;
	int curr_color;
	TMovementContourParams curr_contour_params;
	double curr_feed;

	TMovementCommand* curr_movement_command;
	TToolPath* curr_tool_path;

	CLSFNodes()
		:curr_state(STATE_PROGRAM)
		,rapid(true)
		,curr_move_type(TMovement::RAPID)
		,curr_color(186) 
		,curr_movement_command(NULL)
		,curr_tool_path(NULL)
		,curr_feed(0)
	{}

	TProgram* program;
};


