#pragma once

#include <Eigen/Dense>
#include <stdio.h>
#include <string>
#include <vector>
#include <list>
#include <boost/ptr_container/ptr_vector.hpp>
#include <pugixml.hpp>

using namespace Eigen;
using namespace std;

void Save(pugi::xml_node to_xml,const char* name,vector<double>& v);
void Load(pugi::xml_node from_xml,const char* name,vector<double>& v);

void Save(pugi::xml_node to_xml,const char* name,string& v);
void Load(pugi::xml_node from_xml,const char* name,string& v);

void Save(pugi::xml_node to_xml,const char* name,Vector3d& v);
void Load(pugi::xml_node from_xml,const char* name,Vector3d& v);

struct TCLSF_COMMENT
{
	string text;
        void Save(pugi::xml_node to_xml){to_xml.append_attribute("text").set_value(text.c_str());}
        void Load(pugi::xml_node from_xml){text=from_xml.attribute("text").value();}
};

struct TCLSF_TOOL_PATH
{
	string path_name;
	string tool_type;
	string tool_name;
        void Save(pugi::xml_node to_xml){
            to_xml.append_attribute("path_name").set_value(path_name.c_str());
            to_xml.append_attribute("tool_type").set_value(tool_type.c_str());
            to_xml.append_attribute("tool_name").set_value(tool_name.c_str());
        }
        void Load(pugi::xml_node from_xml){
            path_name=from_xml.attribute("path_name").value();
            tool_type=from_xml.attribute("tool_type").value();
            tool_name=from_xml.attribute("tool_name").value();
        }
};

struct TCLSF_TOOL_DATA
{
	string tool_type;
	vector<double> params;
        void Save(pugi::xml_node to_xml){
            to_xml.append_attribute("tool_type").set_value(tool_type.c_str());
            ::Save(to_xml,"params",params);
        }
        void Load(pugi::xml_node from_xml){
            tool_type=from_xml.attribute("tool_type").value();
            ::Load(from_xml,"params",params);
        }
};

struct TCLSF_SELECT
{
	string name;
	int id;
        void Save(pugi::xml_node to_xml){
            to_xml.append_attribute("name").set_value(name.c_str());
            to_xml.append_attribute("id").set_value(id);
        }
        void Load(pugi::xml_node from_xml){
            name=from_xml.attribute("name").value();
            id=from_xml.attribute("id").as_int();
        }
};

struct TCLSF_MSYS
{
	vector<double> params;
        void Save(pugi::xml_node to_xml){
            ::Save(to_xml,"params",params);
        }
        void Load(pugi::xml_node from_xml){
            ::Load(from_xml,"params",params);
        }
};

struct TCLSF_GOTO
{
	Vector3d pos;
	bool has_dir;
	Vector3d dir;
	bool has_contact_point;
	Vector3d contact_point;
        void Save(pugi::xml_node to_xml){
            ::Save(to_xml,"pos",pos);
            if(has_dir)
                ::Save(to_xml,"dir",dir);
            if(has_contact_point)
                ::Save(to_xml,"contact",contact_point);
        }
        void Load(pugi::xml_node from_xml){
            ::Load(from_xml,"pos",pos);
            if(!from_xml.attribute("dir").empty())
            {
                has_dir=true;
                ::Load(from_xml,"dir",dir);
            }else has_dir=false;
            if(!from_xml.attribute("contact").empty())
            {
                has_contact_point=true;
                ::Load(from_xml,"contact",contact_point);
            }else has_contact_point=false;
        }
};

struct TCLSF_VEC3
{
	Vector3d v;
};

struct TCLSF_FROM
{
	Vector3d pos;
};

struct TCLSF_GOHOME
{
	Vector3d pos;
};

struct TCLSF_CIRCLE
{
	Vector3d 
		center,
		normal,
		pos;
	vector<double> params;
	bool has_spiral_times;
	int spiral_times;
};

struct TCLSF_FEEDRAT
{
	//string type;
	double feed_rate;
};

struct TCLSF_SPINDL
{
	string type;
	double rate;
	string dir;
};

struct TCLSF_CUTCOM
{
	string type;
	string plane;
};

struct TCLSF_AUXFUN
{
	int id;
	string text;
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
	Vector3d pos;
	double rot[2];
	TMovementCommand()
		:rapid(false)
		,cut_feed(0)
	{}
	virtual ~TMovementCommand(){}
	virtual void SetDir(const Vector3d& use_dir)=0;
	virtual void SetEndPos(const Vector3d& use_pos)=0;
	virtual void SetContact(const Vector3d& use_contact)=0;
	virtual void GetGCode(string& program)=0;
    virtual void Save(pugi::xml_node to_xml)=0;
    virtual void Load(pugi::xml_node from_xml)=0;
	virtual bool HasContactPoint()=0;
	virtual Vector3d GetContactPoint()=0;
	virtual bool HasDir()=0;
	virtual Vector3d GetDir()=0;
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
    void Save(pugi::xml_node to_xml);
    void Load(pugi::xml_node from_xml);
};

struct TMovementContour: public TMovementContourParams
{
	boost::ptr_vector<TMovementCommand> commands;
    void Save(pugi::xml_node to_xml);
    void Load(pugi::xml_node from_xml);
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
	list<TMovementContour> contours;
    void Save(pugi::xml_node to_xml);
    void Load(pugi::xml_node from_xml);
};

struct TToolPath
{
	TCLSF_TOOL_PATH path_data;
	TCLSF_TOOL_DATA tool_data;
	TCLSF_MSYS msys;
	list<TMovement> movements;
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
        void Save(pugi::xml_node to_xml);
	void Load(pugi::xml_node from_xml);
};

struct TProgram
{
	list<TToolPath> tool_paths;
	void GetGCode(string& program);
	void Save(pugi::xml_node to_xml);
	void Load(pugi::xml_node from_xml);
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


