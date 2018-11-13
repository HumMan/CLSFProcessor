#include <string>
#include <vector>
#include <map>

namespace CLSFProcessor
{
	namespace Conf
	{
		struct TCommon
		{
			struct TGCodeAxisProperties
			{
				bool remove_repeat;
				bool force_rapid_change;
				std::string lock_header;
				std::string lock_footer;
				std::string format;
				bool is_increment;
				double repeat_tol;
				bool rad_to_deg;
			};
		public:
			double C_pole_min;
			double C_pole_max;

			double A_pole_min;
			double A_pole_max;
		public:
			double tool_length;
			std::string tool_name;

			double any_C_epsilon;
			double ortho_vec_epsilon;
			//T inverse_kinemtatics_tol;


			bool remove_F_repeat;
			int any_C_criteria;

			std::string G_code_header;
			std::string G_code_footer;
			int local_CS_G_index;

			struct TOrientedFromGoto
			{
				std::string path_name;
				double orientation;
			};
			std::vector<TOrientedFromGoto> oriented_from_goto;
			//bool oriented_from_goto;//спец ф-я для джомакса - ориентированный поход и отход
			//double oriented_from_goto_orientation;


			bool use_subdivision;
			bool subdivide_only_any_C;
			bool use_circles;
			bool use_tool_length_correction;

			std::string head_name;

			TGCodeAxisProperties gcode_axis_prop[5];
		};
	}
}