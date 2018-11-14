#pragma once


#include <map>
#include <set>

#include "commonConf.h"

#include "Uni5axis.h"

#include "atptokenizer.h"

namespace CLSFProcessor
{
	namespace Funcs
	{
		inline double sqr(double value)
		{
			return value * value;
		}
		//inline double AngleFromDir(Eigen::Vector2d vec)
		//{
		//	auto c = acos(vec[0]);
		//	if (asin(vec[1]) > 0)
		//		return c;
		//	else
		//		return c + M_PI;
		//}	


		inline bool IsInMinMax(double value, double min, double max)
		{
			return value > min && value < max;
		}

		inline bool IsInMinMax(TAngle value, TAngle min, TAngle max)
		{
			return value.AsRad() > min.AsRad() && value.AsRad() < max.AsRad();
		}

		inline double min(double v0, double v1)
		{
			if (v0 < v1)
				return v0;
			else
				return v1;
		}
		inline double Blend(double v0, double v1, double factor)
		{
			return v0 * (1 - factor) + v1 * factor;
		}
	};

	struct TToolOrientation
	{
		Eigen::Vector3d pos, dir;
	};

	struct TKinematicsNode
	{
		TKinematicsPair kinematics_pair;

		bool move_over_pole[2];		//индекс в массив - текущий проверяемый вариант, но не текущая линия (т.к. маршруты перекрываются)
		bool change_line[2];

		double A_pole_change_delta[2];	//
		double C_pole_change_delta[2];	//
		double A_inc[2];					//
		double C_inc[2];					//
		int poles_count[2];			//

		int best_line;
		double tol;
	};

	enum PrimitiveMask
	{
		LINE = 1,
		CIRCLE = 2,
	};

	struct TMachineState
	{
		PrimitiveMask mask;
		std::string auxfun;
		Eigen::Vector3d center, normal;
		double radius;
		int spiral_times;
		double feed;
		double spndl_rpm;
		bool clw;//TODO разобраться с парсингом а то глючит
		bool rapid;
		int color;
		bool base_element;//если true - элемент нельзя убрать при компрессии т.к. он исходный(существовавший до разбиения)
		int cutcom;
		std::string path_name;
	};

	struct TPipelineElement
	{
		TMachineState state;

		TToolOrientation tool_orient;
		TKinematicsNode machine_orient;
	};

	struct TToolMovementElement
	{
		TMachineState state;
		TKinematics kinematics;

		TToolOrientation tool_orient;
		double move_time;
		double move_distance;
		double contour_correct_feed;
	};

	class TATPProcessor
	{
		CLSFProcessor::Conf::TCommon conf;
		CLSFProcessor::Conf::TFiveAxis five_axis_conf;
		CLSFProcessor::Conf::TProcessor processor_conf;

		double min_tol;
		double max_tol;

		int linear_node[3]; //индексы линейных звеньев
		int rot_node[2]; //индексы поворотных звеньев

		TUniversal5axis machine;

	public:		

		TATPProcessor(Conf::TCommon common_conf, Conf::TFiveAxis five_axis_conf, Conf::TProcessor processor_conf)
			:machine(five_axis_conf)
		{
			this->conf = conf;
			this->five_axis_conf = five_axis_conf;
			this->processor_conf = processor_conf;

			int curr_linear = 0, curr_rot = 0;

			for (int i = 0; i < five_axis_conf.nodes.size(); i++)
			{
				auto& node = five_axis_conf.nodes[i];
				if (node.is_linear)
				{
					linear_node[curr_linear] = i;
					curr_linear++;
					if (curr_linear > 2)
						throw std::exception("неверная конфигурация");
				}
				else if (!node.is_linear)
				{
					rot_node[curr_rot] = i;
					curr_rot++;
					if (curr_rot > 1)
						throw std::exception("неверная конфигурация");
				}
			}
		}

		bool AIsInPole(TAngle A)
		{
			return !Funcs::IsInMinMax(A, conf.A_pole_min, conf.A_pole_max);
		}
		bool CIsInPole(TAngle C)
		{
			return !Funcs::IsInMinMax(C, conf.C_pole_min, conf.C_pole_max);
		}
		bool AIsMoveOverPole(TAngle start, TAngle end)
			//перемещение вокруг оси A идет через полюс
			//start,end - перемещение в виде линейных координат (с учетом скручиваний и раскручиваний)
		{
			return start.AsRad() > conf.A_pole_max.AsRad() || start.AsRad() < conf.A_pole_min.AsRad() ||
				end.AsRad() > conf.A_pole_max.AsRad() || end.AsRad() < conf.A_pole_min.AsRad();
		}
		bool CIsMoveOverPole(TAngle start, TAngle end)
			//перемещение вокруг оси C идет через полюс
			//start,end - перемещение в виде линейных координат (с учетом скручиваний и раскручиваний)
		{
			bool isvalid = start.AsRad() > conf.C_pole_max.AsRad() || start.AsRad() < conf.C_pole_min.AsRad() ||
				end.AsRad() > conf.C_pole_max.AsRad() || end.AsRad() < conf.C_pole_min.AsRad();
			return isvalid;
		}

		//result - направление наикратчайшего перемещения из a0 в a1
			//dist - величина и знак перемещения (+ это CCW)
		bool IsCCWMove(TAngle a0, TAngle a1, TAngle& dist);

		//result - стоимость перемещения по наикратчайшему пути из v0 в v1
		double MovementCost(TKinematics v0, TKinematics v1);

		void FindAnyC(std::vector<TPipelineElement> &pipe, int start_from, int &region_end);

		//result - кратчайшие перемещения находятся в вариантах с одинаковыми индексами
		bool NeedLinearizeMovementSwap(std::vector<TPipelineElement> &pipe, int i);
		double GetInterpolationTolerance(TKinematics p0, TKinematics p1, TToolOrientation &middle_tool);

		//delta_A,delta_C - приращения необходимые для перемещения из кинематики 0 в кинематику 1
		void GetMovement(TKinematics p0, TKinematics p1, TAngle& delta_A, TAngle& delta_C);
		void Clear(std::vector<TPipelineElement> &pipe);

		void CalcKinematics(std::vector<TPipelineElement> &pipe);
		bool IsOrthogonalVector(Eigen::Vector3d v, int& axis, bool& pos_dir);
		void CircleLinearApprox(Eigen::Vector2d v0, Eigen::Vector2d v1, double rad, double arc_error, std::vector<Eigen::Vector2d>& result, bool nearest_arc, bool ccw, bool full_circle = false);

		//TODO пока что здесь все дуги разбиваются на мелкие линейные отрезки
		void CalcCircleParameters(std::vector<TPipelineElement> &pipe);

		void ValidateKinematics(std::vector<TPipelineElement> &pipe);

		void SwapVariants(std::vector<TPipelineElement> &pipe);

		void GetCWithMaxX(Eigen::Vector3d tool_pos, double A, double initial_C, double& result_C, Eigen::Vector3d& result_pos);
		void DetermineAnyCKinematics(std::vector<TPipelineElement> &pipe);
		void TraceLine(std::vector<TPipelineElement> &pipe, int line);
		void SelectBestLine(std::vector<TPipelineElement> &pipe, std::vector<TToolMovementElement> &result_pipe);

		//в отличие от предыдущей версии считает перемещение по всем координатам в линейном виде
		double MovementDistance(TKinematics k0, TKinematics k1);

		void CalculateMoveTime(std::vector<TToolMovementElement> &pipe, double &fast_movement_time, double &work_movement_time);

		void CalculateContourSpeed(std::vector<TToolMovementElement> &pipe);

		//return - произошло удаление элемента
		bool MergeFrames(std::vector<TToolMovementElement> &pipe, int window_start, int window_end);

		void Compress(std::vector<TToolMovementElement> &pipe);
		void TryAlignInvalidVariants(std::vector<TPipelineElement> &pipe);
		void Subdivide(std::vector<TToolMovementElement> &pipe);
		void ValidateKinematicsInverseAlgorithm(std::vector<TPipelineElement> &pipe);
		void MakePipe(std::vector<TPipelineElement> &pipe, std::vector<TCLSFToken> &atp_tokens);
		void PassThrough(std::vector<TCLSFToken> &atp_tokens, std::vector<TToolMovementElement> &result_pipe, double &fast_movement_time, double &work_movement_time);

		//только для standalone постов, далее убрать
		void GetCode(std::vector<TToolMovementElement> &pipe, std::string &result_code, const char* ext_header, const char* prog_id);

	};
}