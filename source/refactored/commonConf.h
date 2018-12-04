#pragma once

#include <string>
#include <vector>
#include <map>

#define M_PI 3.14159265358979323846

namespace CLSFProcessor
{
	class TAngle
	{
		double value;//в радианах;

	public:
		TAngle()
		{
			value = 0;
		}
		explicit TAngle(double angle_in_radians)
		{
			value = angle_in_radians;
		}
		static TAngle FromDeg(double angle)
		{
			return TAngle(DegToRad(angle));
		}
		static TAngle FromRad(double angle)
		{
			return TAngle(angle);
		}

		double AsRad()
		{
			return value;
		}
		double AsDeg()
		{
			return value / 180.0 * M_PI;
		}
		double To0_360Space()
		{
			auto angle = fmod(value, double(2 * M_PI));
			if (angle < 0)angle = 2 * M_PI + angle;
			return angle;
		}
		static double DegToRad(double value)
		{
			return value / 180.0 * M_PI;
		}
	};

	namespace Conf
	{

		struct TProcessor
		{
			double tolerance;
			double rapid_tolerance;
			double rapid_feed;
			double contour_max_feed;
			int frames_on_1sec_max;
			bool circle_interpolation_center_absol;
		};

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

			TAngle C_pole_min;
			TAngle C_pole_max;

			TAngle A_pole_min;
			TAngle A_pole_max;

			double tool_length;
			std::string tool_name;

			double any_C_epsilon;
			double ortho_vec_epsilon;
			double inverse_kinemtatics_tol;

			bool remove_F_repeat;
			int any_C_criteria;

			std::string G_code_header;
			std::string G_code_footer;
			int local_CS_G_index;

			bool use_subdivision;
			bool subdivide_only_any_C;
			bool use_circles;
			bool use_tool_length_correction;

			std::string head_name;

			std::vector<TGCodeAxisProperties> gcode_axis_prop;
		};
	}
}