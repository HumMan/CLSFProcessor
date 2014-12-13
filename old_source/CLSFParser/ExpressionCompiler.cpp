#include "ExpressionCompiler.h"

#include "MovementCommands.h"

void compiler::PrepareCommandInsertion(CLSFNodes &nodes,TVector<TMovement>* &movements,TVector<TMovementContour>* &contours,boost::ptr_vector<TMovementCommand>* &commands)const
{
	movements=&nodes.curr_tool_path->movements;

	bool curr_contour_params_change=false;
	
	if(movements->GetCount()==0||(movements->GetTop(0).type!=nodes.curr_move_type))
	{
		if(movements->GetCount()==0)
			curr_contour_params_change=true;
		else
			curr_contour_params_change=(movements->GetTop(0).contours.GetTop(0)!=nodes.curr_contour_params);

		movements->Push();
		movements->GetTop(0).type=nodes.curr_move_type;
	}

	contours=&movements->GetTop(0).contours;
	if(contours->GetCount()==0||(contours->GetTop(0)!=nodes.curr_contour_params))
	{
		if(contours->GetCount()!=0)
			curr_contour_params_change=contours->GetTop(0)!=nodes.curr_contour_params;
		contours->Push();
		*(TMovementContourParams*)(&contours->GetTop(0))=nodes.curr_contour_params;
		contours->GetTop(0).contour_params_change=curr_contour_params_change;
	}

	commands=&contours->GetTop(0).commands;
}

void compiler::operator()(CLSFNodes &nodes, ::TCLSF_TOOL_PATH const &params,compiler::TCLSF_TOOL_PATH) const
{
	if(nodes.curr_state!=CLSFNodes::STATE_PROGRAM)throw std::string("Error: TOOL_PATH must be in program!");
	nodes.curr_state=CLSFNodes::STATE_TOOL_PATH;
	nodes.program->tool_paths.push_back(TToolPath());
	nodes.program->tool_paths.front().path_data=params;
	nodes.curr_tool_path=&nodes.program->tool_paths.front();
}

void compiler::operator()(CLSFNodes &nodes, ::TCLSF_TOOL_DATA const &params,compiler::TCLSF_TOOL_DATA) const
{
	if(nodes.curr_state!=CLSFNodes::STATE_TOOL_PATH)throw std::string("Error: TOOL_DATA must be in tool path!");
	nodes.curr_tool_path->tool_data=params;
}

void compiler::operator()(CLSFNodes &nodes, int const &params,compiler::TCLSF_LOAD_TOOL) const
{
	if(nodes.curr_state!=CLSFNodes::STATE_TOOL_PATH)throw std::string("Error: LOAD TOOL must be in tool path!");
	nodes.curr_tool_path->has_tool_id=true;
	nodes.curr_tool_path->tool_id=params;
}

void compiler::operator()(CLSFNodes &nodes, int const &params,compiler::TCLSF_LOAD_ADJUST) const
{
	if(nodes.curr_state!=CLSFNodes::STATE_TOOL_PATH)throw std::string("Error: LOAD TOOL ADJUST must be in tool path!");
	nodes.curr_tool_path->has_tool_adjust=true;
	nodes.curr_tool_path->tool_adjust=params;
}

void compiler::operator()(CLSFNodes &nodes, ::TCLSF_SELECT const &params,compiler::TCLSF_SELECT) const
{
	if(nodes.curr_state!=CLSFNodes::STATE_TOOL_PATH)throw std::string("Error: SELECT must be in tool path!");
}

void compiler::operator()(CLSFNodes &nodes, ::TCLSF_MSYS const &params,compiler::TCLSF_MSYS) const
{
	if(nodes.curr_state!=CLSFNodes::STATE_TOOL_PATH)throw std::string("Error: MSYS must be in tool path!");
	nodes.program->tool_paths.front().msys=params;
}

void compiler::operator()(CLSFNodes &nodes, int const &params,compiler::TCLSF_PAINT_COLOR) const
{
	switch(params)
	{
	case 186:
		nodes.curr_move_type=TMovement::RAPID;
		break;
	case 211:
		if(nodes.curr_move_type==TMovement::RAPID)
			nodes.curr_move_type=TMovement::APPROACH;
		else
			nodes.curr_move_type=TMovement::DEPARTURE;
		break;
	case 42:
		nodes.curr_move_type=TMovement::ENGAGE;
		break;
	case 31:
		nodes.curr_move_type=TMovement::CUT;
		break;
	case 37:
		nodes.curr_move_type=TMovement::RETRACT;
		break;
	default:
		nodes.curr_move_type=TMovement::CUT;
	}
	nodes.curr_color=params;
}

void compiler::operator()(CLSFNodes &nodes,compiler::TCLSF_PAINT_PATH) const
{
}

void compiler::operator()(CLSFNodes &nodes,compiler::TCLSF_PAINT_TOOL) const
{
}

void compiler::operator()(CLSFNodes &nodes, int const &params,compiler::TCLSF_PAINT_SPEED) const
{
}

void compiler::operator()(CLSFNodes &nodes, compiler::TCLSF_RAPID) const
{
	if(nodes.curr_state!=CLSFNodes::STATE_TOOL_PATH)throw std::string("Error: RAPID must be in tool path!");
	nodes.rapid=true;
}

void compiler::operator()(CLSFNodes &nodes, ::TCLSF_VEC3 const &params,compiler::TCLSF_GOTO) const
{
	if(nodes.curr_state!=CLSFNodes::STATE_TOOL_PATH)throw std::string("Error: GOTO must be in tool path!");
	TVector<TMovement>* movements;
	TVector<TMovementContour>* contours;
	boost::ptr_vector<TMovementCommand>* commands;
	PrepareCommandInsertion(nodes,movements,contours,commands);

	TLinearCommand* c=new TLinearCommand();
	c->has_dir=false;
	c->has_contact_point=false;
	c->pos=params.v;
	c->rapid=nodes.rapid;
	if(nodes.rapid)nodes.rapid=false;
	c->cut_feed=nodes.curr_feed;
	commands->push_back(c);
	nodes.curr_movement_command=c;
}

void compiler::operator()(CLSFNodes &nodes, ::TCLSF_VEC3 const &params,compiler::TCLSF_CIRCLE_GOTO) const
{
	if(nodes.curr_state!=CLSFNodes::STATE_TOOL_PATH)throw std::string("Error: CIRCLE_GOTO must be in tool path!");
	nodes.curr_movement_command->SetEndPos(params.v);
}

void compiler::operator()(CLSFNodes &nodes, ::TCLSF_VEC3 const &params,compiler::TCLSF_DIR) const
{
	nodes.curr_movement_command->SetDir(params.v);
}

void compiler::operator()(CLSFNodes &nodes, ::TCLSF_VEC3 const &params,compiler::TCLSF_CONTACT_POINT) const
{
	nodes.curr_movement_command->SetContact(params.v);
}

void compiler::operator()(CLSFNodes &nodes, ::TCLSF_VEC3 const &params,compiler::TCLSF_FROM) const
{
	if(nodes.curr_state!=CLSFNodes::STATE_TOOL_PATH)throw std::string("Error: FROM must be in tool path!");

	TVector<TMovement>* movements;
	TVector<TMovementContour>* contours;
	boost::ptr_vector<TMovementCommand>* commands;
	PrepareCommandInsertion(nodes,movements,contours,commands);

	TFromCommand* c=new TFromCommand();
	c->pos=params.v;
	c->rapid=nodes.rapid;
	if(nodes.rapid)nodes.rapid=false;
	c->cut_feed=nodes.curr_feed;
	commands->push_back(c);
	nodes.curr_movement_command=c;
}

void compiler::operator()(CLSFNodes &nodes, ::TCLSF_VEC3 const &params,compiler::TCLSF_GOHOME) const
{
	if(nodes.curr_state!=CLSFNodes::STATE_TOOL_PATH)throw std::string("Error: GOHOME must be in tool path!");
	
	TVector<TMovement>* movements;
	TVector<TMovementContour>* contours;
	boost::ptr_vector<TMovementCommand>* commands;
	PrepareCommandInsertion(nodes,movements,contours,commands);

	TGoHomeCommand* c=new TGoHomeCommand();
	c->pos=params.v;
	c->rapid=nodes.rapid;
	if(nodes.rapid)nodes.rapid=false;
	c->cut_feed=nodes.curr_feed;
	commands->push_back(c);
	nodes.curr_movement_command=c;
}

void compiler::operator()(CLSFNodes &nodes, ::TCLSF_CIRCLE const &params,compiler::TCLSF_CIRCLE) const
{
	if(nodes.curr_state!=CLSFNodes::STATE_TOOL_PATH)throw std::string("Error: CIRCLE must be in tool path!");
	
	TVector<TMovement>* movements;
	TVector<TMovementContour>* contours;
	boost::ptr_vector<TMovementCommand>* commands;
	PrepareCommandInsertion(nodes,movements,contours,commands);

	TCircleCommand* c=new TCircleCommand();
	c->has_spiral_times=params.has_spiral_times;
	c->spiral_times=params.spiral_times;
	c->circle_params=params.params;
	c->center=params.center;
	c->normal=params.normal;
	c->rapid=nodes.rapid;
	if(nodes.rapid)nodes.rapid=false;
	c->cut_feed=nodes.curr_feed;
	commands->push_back(c);
	nodes.curr_movement_command=c;
}

void compiler::operator()(CLSFNodes &nodes, ::TCLSF_FEEDRAT const &params,compiler::TCLSF_FEEDRAT) const
{
	if(nodes.curr_state!=CLSFNodes::STATE_TOOL_PATH)throw std::string("Error: FEEDRAT must be in tool path!");
	nodes.rapid=false;
	nodes.curr_feed=params.feed_rate;
}

void compiler::operator()(CLSFNodes &nodes, ::TCLSF_SPINDL const &params,compiler::TCLSF_SPINDL) const
{
	if(nodes.curr_tool_path->has_spindl_rpm)throw std::string("Error: SPINDL used more than one time in tool path!");
	nodes.curr_tool_path->has_spindl_rpm=true;
	nodes.curr_tool_path->spindl_rpm=params.rate;
}

void compiler::operator()(CLSFNodes &nodes, ::TCLSF_CUTCOM const &params,compiler::TCLSF_CUTCOM) const
{
	if(params.type=="OFF")
	{
			nodes.curr_contour_params.has_compensation=false;
			nodes.curr_contour_params.compensation_type=false;
			nodes.curr_contour_params.compensation_plane=0;
	}
	else
	{
		nodes.curr_contour_params.has_compensation=true;
		nodes.curr_contour_params.compensation_type=(params.type=="LEFT");
		if(params.plane=="XYPLAN")
			nodes.curr_contour_params.compensation_plane=0;
		else if(params.plane=="YZPLAN")
			nodes.curr_contour_params.compensation_plane=1;
		else if(params.plane=="ZXPLAN")
			nodes.curr_contour_params.compensation_plane=2;
		else throw std::string("Error: CUTCOM unknown compensation plane type!");
	}
}


void compiler::operator()(CLSFNodes &nodes, ::TCLSF_AUXFUN const &params,compiler::TCLSF_AUXFUN) const
{
}

void compiler::operator()(CLSFNodes &nodes,compiler::TCLSF_END_OF_PATH) const
{
	if(nodes.curr_state!=CLSFNodes::STATE_TOOL_PATH)throw std::string("Error: end of path must be in tool path!");
	nodes.curr_state=CLSFNodes::STATE_PROGRAM;
}

//nx connector generated events
void compiler::operator()(CLSFNodes &nodes, std::string const &params,compiler::TCLSF_NX_PROCESSOR_SET_CS_G) const
{
}

void compiler::operator()(CLSFNodes &nodes, std::string const &params,compiler::TCLSF_NX_PROCESSOR_PATH_CS_NAME) const
{
}

char* movement_type_string[]=
{
	"RAPID",
	"APPROACH",
	"ENGAGE",
	"CUT",
	"RETRACT",
	"DEPARTURE"
};

void TProgram::GetGCode(std::string& program)
{
	char buf[1000];
	program+=";PROGRAM START\n";
	std::list<TToolPath>::iterator path;
	for(path=tool_paths.begin();path!=tool_paths.end();path++)
	{
		program+=";PATH START\nPATH_NAME=";
		program+=path->path_data.path_name+"\n";
		for(int i=0;i<=path->movements.GetHigh();i++)
		{
			TMovement& movement=path->movements[i];
			program+="MOVEMENT_TYPE=";
			program+=movement_type_string[movement.type];
			program+="\n";
			for(int c=0;c<=movement.contours.GetHigh();c++)
			{
				TMovementContour& contour=movement.contours[c];
				if(contour.contour_params_change)
				{
				program+="CONTOUR= comp(";
				program+=std::string(contour.has_compensation?"on":"off")+")\n";
				}
				boost::ptr_vector<TMovementCommand>::iterator com;
				for(com=contour.commands.begin();com!=contour.commands.end();com++)
				{
					com->GetGCode(program);
				}
			}
		}
		program+=";PATH END\n";
	}
	program+=";PROGRAM END\n";
}