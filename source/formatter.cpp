#include "formatter.h"


#include <fstream>
#include <iostream>

using namespace CLSFProcessor;

struct State
{
	double curr_feed = 0/*pipe[0].feed*/;
	int curr_cutcom = 0;
	std::string curr_path_name;
	double curr_spndl_rpm;
	bool curr_clw;

	int frame_number = 1;
	//текущие координаты
	double curr_coord[5];
	//текущие координаты с учетом округления(используется при инкрементном режиме для устранения накапливающейся ошбки округления)
	double curr_coord_machine[5];
	bool need_print[5] = { 0,0,0,0,0 };

	std::array<double, 5> new_coord;
};

std::string ProcessLine(State& state, bool is_first, CLSFProcessor::TToolMovementElement& element, CLSFProcessor::Conf::TCommon conf, CLSFProcessor::Conf::TProcessor processor_conf)
{
	char* buff = new char[1000];

	std::string result_code("");

	state.new_coord = element.kinematics.v;
	//switch(element.mask)
	{
		//case PrimitiveMask::LINE:
		//	{
		bool is_path_begin = (state.curr_path_name != element.state.path_name&&element.state.path_name != "");

		state.curr_path_name = element.state.path_name;


		if (!element.state.rapid&&element.state.feed != state.curr_feed)
		{
			//sprintf(buff,"N%i G01 F%i\n",frame_number+=10,(int)element.feed);
			//result_code+=buff;
		}

		if ((1/*coord_repeat_tol*/ < abs(element.state.spndl_rpm - state.curr_spndl_rpm) || state.curr_clw != element.state.clw || is_first))
		{
			if (abs(state.curr_spndl_rpm - 0) < 0.001 || is_first)
				sprintf(buff, "M3S%i\n",/*element.clw?3:4,*/int(element.state.spndl_rpm));
			else if (abs(element.state.spndl_rpm - 0) < 0.001)
				sprintf(buff, "M5\n");
			else sprintf(buff, "");
			result_code += buff;
			state.curr_spndl_rpm = element.state.spndl_rpm;
			state.curr_clw = element.state.clw;
		}

		bool force_rapid_change = false;
		for (int c = 0; c < 5; c++)
		{
			bool last_need_print = state.need_print[c];
			double tol = conf.gcode_axis_prop[c].rad_to_deg ? TAngle::FromDeg(conf.gcode_axis_prop[c].repeat_tol).AsRad() : conf.gcode_axis_prop[c].repeat_tol;
			state.need_print[c] =
				tol <= abs(state.new_coord[c] - state.curr_coord[c]) || !conf.gcode_axis_prop[c].remove_repeat;
			if (state.need_print[c] && conf.gcode_axis_prop[c].force_rapid_change)
				force_rapid_change = true;
			//если имеется функция разблокировки/блокировки оси то включаем ее
			if (!last_need_print&&state.need_print[c] && conf.gcode_axis_prop[c].lock_header != "")
			{
				result_code += conf.gcode_axis_prop[c].lock_header;
			}
			if (last_need_print && !state.need_print[c] && conf.gcode_axis_prop[c].lock_header != "")
			{
				result_code += conf.gcode_axis_prop[c].lock_footer;
			}
		}

		sprintf(buff, "%s", element.state.auxfun.c_str());
		result_code += buff;
		//если направление инструмента 0,0,-1 то надо сменить коррекцию на противоположную
		if ((element.tool_orient.dir - (Eigen::Vector3d(0, 0, -1))).norm() < 0.0001)
		{
			if (element.state.cutcom == 1)element.state.cutcom = 2;
			if (element.state.cutcom == 2)element.state.cutcom = 1;
		}
		if (element.state.cutcom != state.curr_cutcom)
		{
			sprintf(buff, "G%i", 40 + element.state.cutcom);
			result_code += buff;
			state.curr_cutcom = element.state.cutcom;
		}
		//плоскость круговой/спиральной интерполяции
		//if (element.state.mask == PrimitiveMask::CIRCLE)
		//{
		//	int rot_axis;
		//	bool positive;
		//	IsOrthogonalVector(element.state.normal, rot_axis, positive);
		//	int rot_axis_to_plane[3] = { 19,18,17 };
		//	sprintf(buff, "G%i", rot_axis_to_plane[rot_axis]);
		//	result_code += buff;
		//}

		{
			int g_move_mode = -1;
			if (element.state.mask == PrimitiveMask::CIRCLE)
			{
				g_move_mode = (element.state.normal.dot(Eigen::Vector3d(1, 1, 1).normalized()) > 0) ? 2 : 3;
			}
			else
			{
				g_move_mode = (element.state.rapid || force_rapid_change) ? 0 : 1;
			}
			sprintf(buff,/*"N%i"*/"G0%i "/*,frame_number+=10*/, g_move_mode);
			result_code += buff;
		}

		for (int c = 0; c < 5; c++)
		{
			if (state.need_print[c])
			{
				double new_coord_val =
					conf.gcode_axis_prop[c].rad_to_deg
					? TAngle::FromRad(state.new_coord[c]).AsDeg()
					: state.new_coord[c];

				if (conf.gcode_axis_prop[c].is_increment)
				{
					double inc_val = new_coord_val - state.curr_coord_machine[c];
					sprintf(buff, conf.gcode_axis_prop[c].format.c_str(), inc_val);
					inc_val = floor(inc_val / conf.gcode_axis_prop[c].repeat_tol + 0.5)*conf.gcode_axis_prop[c].repeat_tol;

					state.curr_coord_machine[c] += inc_val;
				}
				else
					sprintf(buff, conf.gcode_axis_prop[c].format.c_str(), new_coord_val);
				result_code += buff;
				state.curr_coord[c] = state.new_coord[c];
			}
		}
		//if (element.state.mask == PrimitiveMask::CIRCLE)
		//{
		//	TVec<double, 3> center = element.state.center;
		//	TVec<double, 3> machine_center = ToMachineToolKinematics(center, element.A, element.C);
		//	TVec<double, 3> machine_last_pos = ToMachineToolKinematics(element.pos, element.A, element.C);

		//	int rot_axis;
		//	bool positive;
		//	IsOrthogonalVector(element.state.normal, rot_axis, positive);

		//	//печатаем координаты центра кроме координаты оси вращения
		//	for (int a = 0; a < 3; a++)
		//	{
		//		if (circle_interpolation_center_absol)
		//		{
		//			char* r[] = {
		//				"I=AC(%f)","J=AC(%f)","K=AC(%f)"
		//			};
		//			if (rot_axis != a)
		//			{
		//				sprintf(buff, r[a], machine_center[a]);
		//				result_code += buff;
		//			}
		//		}
		//		else
		//		{
		//			char* r[] = {
		//				"I%f","J%f","K%f"
		//			};
		//			if (rot_axis != a)
		//			{
		//				sprintf(buff, r[a], machine_center[a] - machine_last_pos[a]);
		//				result_code += buff;
		//			}
		//		}
		//	}

		//	if (element.state.spiral_times > 0)
		//	{
		//		sprintf(buff, " TURN=%i ", element.state.spiral_times);
		//		result_code += buff;
		//	}
		//}

		if (1/*coord_repeat_tol*/ < abs(element.contour_correct_feed - state.curr_feed) || !conf.remove_F_repeat)
		{
			if (!element.state.rapid)
			{
				sprintf(buff, "F%i", element.state.rapid ? (int)processor_conf.rapid_feed : (int)element.contour_correct_feed);
				result_code += buff;
				state.curr_feed = element.state.feed;
			}
		}

		result_code += "\n";

		//}break;
	//case PrimitiveMask::CIRCLE:
		/*{
			TVec<double,3> prev_pos=pipe[i-1].pos,
				next_pos=element.pos,
				center;
			if(circle_interpolation_center_absol)
				center=prev_pos+element.center-pipe[i-1].tool_orient.pos;
			else
				center=element.center-pipe[i-1].tool_orient.pos;
			sprintf(buff,"%sN%i G0%i X%.3f Y%.3f Z%.3f I%.3f J%.3f\n",
				element.auxfun.c_str(),
				frame_number+=10,
				element.normal[2]<0?3:2,
					next_pos[0],next_pos[1],next_pos[2],
					center[0],center[1],center[2]);
				result_code+=buff;

		}break;*/
		//default:assert(false);

		return result_code;
	}
}


//void TATPProcessor::GetCode(std::vector<TToolMovementElement> &pipe, string &result_code, const char* ext_header, const char* prog_id)//только для standalone постов, далее убрать
//{
//	//boost::format header_format(G_code_header.c_str());
//	//header_format.exceptions( boost::io::all_error_bits ^ ( boost::io::too_many_args_bit | boost::io::too_few_args_bit )  );
//
//	//boost::format footer_format(G_code_footer.c_str());
//	//footer_format.exceptions( boost::io::all_error_bits ^ ( boost::io::too_many_args_bit | boost::io::too_few_args_bit )  );
//
//	//GetGCode(pipe,result_code);
//	//result_code=
//	//	(
//	//	header_format
//	//	%local_CS_G_index
//	//	%prog_id
//	//	%head_name
//	//	%tool_name
//	//	).str()
//	//	+
//	//	result_code
//	//	+
//	//	(
//	//	footer_format
//	//	%prog_id
//	//	).str();
//	//result_code=G_code_header+result_code+G_code_footer;
//	//InsertGCodeHead(result_code,TVec<double,3>(),"",false);
//}
////void GetCode(std::vector<TToolMovementElement> &pipe,string &result_code,
////	TVec<double,3> use_offset, string use_machine_offset_string,bool use_machine_offset)//только для standalone постов, далее убрать
////{
////	GetGCode(pipe,result_code);
////	result_code=(boost::format(G_code_header.c_str())%local_CS_G_index).str()+result_code+G_code_footer;
////	//result_code=G_code_header+result_code+G_code_footer;
////	//InsertGCodeHead(result_code,use_offset,use_machine_offset_string,use_machine_offset);
////}


void CLSFProcessor::GetGCode(CLSFProcessor::Conf::TCommon conf, CLSFProcessor::Conf::TProcessor processor_conf, std::vector<TToolMovementElement> &pipe, const char* result_path)
{
	State state;

	//if(curr_feed==0)curr_feed=1500;//TODO

	state.curr_spndl_rpm = pipe[0].state.spndl_rpm;
	state.curr_clw = pipe[0].state.clw;
	state.new_coord = pipe[0].kinematics.v;

	if (pipe.size() == 0)
		return;

	for (int c = 0; c < 5; c++)
	{
		state.curr_coord[c] = state.new_coord[c] + 2 * conf.gcode_axis_prop[c].repeat_tol;
		state.curr_coord_machine[c] =
			conf.gcode_axis_prop[c].rad_to_deg
			? TAngle::FromRad(state.new_coord[c]).AsDeg()
			: state.new_coord[c];
	}

	std::ofstream result_file(result_path, std::ios::binary);

	for (int i = 0; i < pipe.size(); i++)
	{
		std::string line = ProcessLine(state, i == 0, pipe[i], conf, processor_conf);
		result_file << line;
	}

	result_file.close();
}