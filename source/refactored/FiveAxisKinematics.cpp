#include "FiveAxisKinematics.h"

using namespace CLSFProcessor;

//result - направление наикратчайшего перемещения из a0 в a1
//dist - величина и знак перемещения (+ это CCW)
bool TATPProcessor::IsCCWMove(TAngle _a0, TAngle _a1, TAngle& dist)
{
	double a0 = _a0.To0_360Space();
	double a1 = _a1.To0_360Space();
	double dist_0 = abs(a0 - a1);
	double dist_1 = abs(2 * M_PI - abs(dist_0));

	bool result = (dist_0 < dist_1) ? (a1 > a0) : (a1 < a0);
	dist = TAngle((result ? 1.0 : -1.0)*Funcs::min(dist_0, dist_1));
	return result;
}

//result - стоимость перемещения по наикратчайшему пути из v0 в v1
double TATPProcessor::MovementCost(TKinematics v0, TKinematics v1)
{
	TAngle da, dc;
	IsCCWMove(TAngle(v0.v[ids.rot_node[0]]), TAngle(v1.v[ids.rot_node[0]]), da);
	IsCCWMove(TAngle(v0.v[ids.rot_node[1]]), TAngle(v1.v[ids.rot_node[1]]), dc);
	return sqrt(Funcs::sqr(da.AsRad()) + Funcs::sqr(dc.AsRad()));
}

void TATPProcessor::FindAnyC(std::vector<TPipelineElement> &pipe, int start_from, int &region_end)
{
	//TODO
	//region_end = pipe.size() - 1;
	//for (int i = start_from; i <= pipe.size() - 1; i++)
	//{
	//	if (pipe[i].machine_orient.any_C)
	//	{
	//		region_end = i;
	//		break;
	//	}
	//}
}

//result - кратчайшие перемещения находятся в вариантах с одинаковыми индексами
bool TATPProcessor::NeedLinearizeMovementSwap(std::vector<TPipelineElement> &pipe, int i)
{
	TKinematicsNode &t0 = pipe[i].machine_orient;
	TKinematicsNode &t1 = pipe[i + 1].machine_orient;
	bool result = (
		MovementCost(t0.kinematics_pair.variant[0], t1.kinematics_pair.variant[0]) >
		MovementCost(t0.kinematics_pair.variant[0], t1.kinematics_pair.variant[1]));
	assert(result == (
		MovementCost(t0.kinematics_pair.variant[1], t1.kinematics_pair.variant[0]) <
		MovementCost(t0.kinematics_pair.variant[1], t1.kinematics_pair.variant[1])));
	return result;
}


double TATPProcessor::GetInterpolationTolerance(TKinematics p0, TKinematics p1, TToolOrientation &middle_tool)
{
	Eigen::Vector3d kp0, kp1, kp_middle, tool_dir0, tool_dir1;
	TKinematics k;

	machine.FromMachineToolKinematics(p0, kp0, tool_dir0);
	machine.FromMachineToolKinematics(p1, kp1, tool_dir1);

	TKinematics middle;

	for (int i = 0; i < 5; i++)
		middle.v[i] = (p0.v[i] + p1.v[i])*0.5;

	machine.FromMachineToolKinematics(middle, middle_tool.pos, middle_tool.dir);

	double tol = (((kp0 + kp1)*0.5) - (middle_tool.pos)).norm();
	if (tol > max_tol)max_tol = tol;
	if (tol < min_tol)min_tol = tol;

	middle_tool.pos = (kp0 + kp1)*0.5;
	middle_tool.dir = ((tool_dir0 + tool_dir1)*0.5).normalized();

	return tol;
}

//delta_A,delta_C - приращения необходимые для перемещения из кинематики 0 в кинематику 1
void TATPProcessor::GetMovement(TKinematics p0, TKinematics p1, TAngle& delta_A, TAngle& delta_C)
{
	IsCCWMove(TAngle(p0.v[ids.rot_node[0]]), TAngle(p1.v[ids.rot_node[0]]), delta_A);
	IsCCWMove(TAngle(p0.v[ids.rot_node[1]]), TAngle(p1.v[ids.rot_node[1]]), delta_C);
}

void TATPProcessor::Clear(std::vector<TPipelineElement> &pipe)
{
	for (int i = 0; i < pipe.size(); i++)
	{
		memset(&pipe[i].machine_orient, 0, sizeof(pipe[i].machine_orient));
		for (int k = 0; k < 2; k++)
			pipe[i].machine_orient.poles_count[k] = -1;
		pipe[i].machine_orient.best_line = -1;
	}
}

void TATPProcessor::CalcKinematics(std::vector<TPipelineElement> &pipe)
{
	for (int i = 0; i < pipe.size(); i++)
	{
		if (pipe[i].state.mask&PrimitiveMask::CIRCLE
			|| pipe[i].state.mask&PrimitiveMask::LINE)
		{
			machine.ToMachineToolKinematics(pipe[i].tool_orient.pos, pipe[i].tool_orient.dir, pipe[i].machine_orient.kinematics_pair);

			for (int k = 0; k < 2; k++)
			{
				double tol0, tol1;
				CheckError(pipe[i].machine_orient.kinematics_pair.variant[k],
					pipe[i].tool_orient.pos, pipe[i].tool_orient.dir, tol0, tol1);
			}
		}
	}
}

bool TATPProcessor::IsOrthogonalVector(Eigen::Vector3d v, int& axis, bool& pos_dir)
{
	int c = 0;
	for (int i = 0; i < 3; i++)
		if (abs(v[i]) > conf.ortho_vec_epsilon)
		{
			axis = i;
			c++;
		}
	if (c == 1)
	{
		pos_dir = (v[axis] > 0);
		return true;
	}
	else
		return false;
}

void TATPProcessor::CircleLinearApprox(Eigen::Vector2d v0, Eigen::Vector2d v1, double rad, double arc_error, std::vector<Eigen::Vector2d>& result, bool nearest_arc, bool ccw, bool full_circle)
{
	TAngle l0, l1, dist;
	if (nearest_arc)
	{
		l0 = AngleFromDir(v0);
		l1 = AngleFromDir(v1);
		bool ccw = IsCCWMove(l0, l1, dist);
	}
	else //с явным указанием направления поворота
	{
		l0 = AngleFromDir(v0);
		l1 = AngleFromDir(v1);
		bool _ccw = IsCCWMove(l0, l1, dist);
		if (ccw != _ccw)
		{
			dist = TAngle::FromRad(dist.AsRad() - 2 * M_PI);
		}
		if (full_circle)
			dist = TAngle::FromRad(ccw ? 2 * M_PI : -2 * M_PI);
	}
	int points_high = abs(dist.AsRad()) / acos(1 - arc_error / rad);
	result.resize(points_high - 1);
	result[0] = v0;
	result.back() = v1;
	for (int i = 1; i < points_high; i++)
	{
		//double ang=l0*(points_high-i)/(double)points_high+l1*i/(double)points_high;
		double ang = l0.AsRad() + dist.AsRad() * i / (double)points_high;
		result[i] = Eigen::Vector2d(cos(ang), sin(ang));
	}
}


void TATPProcessor::CalcCircleParameters(std::vector<TPipelineElement> &pipe)//TODO пока что здесь все дуги разбиваются на мелкие линейные отрезки
{

	std::vector<Eigen::Vector2d> points;
	std::vector<TPipelineElement> new_pipe;
	new_pipe.reserve(pipe.size());

	for (int i = 0; i < pipe.size(); i++)
	{
		if (pipe[i].state.mask == PrimitiveMask::CIRCLE)
		{
			pipe[i].tool_orient.pos = pipe[i + 1].tool_orient.pos;
			if ((pipe[i].tool_orient.pos - (pipe[i - 1].tool_orient.pos)).norm() < 0.01)
				pipe[i].state.mask = PrimitiveMask::LINE;
			new_pipe.push_back(pipe[i]);
			i++;
		}
		else
			new_pipe.push_back(pipe[i]);
	}
	int t = 0, off = 0;

	pipe = new_pipe;

	//for(int calc_count_to_insert=0;calc_count_to_insert<2;calc_count_to_insert++)
	for (int i = 0; i < pipe.size(); i++)
	{
		if (pipe[i].state.mask == PrimitiveMask::CIRCLE)
		{
			int axis;
			bool pos_dir;
			if ((conf.use_circles && !IsOrthogonalVector(pipe[i].state.normal, axis, pos_dir)) || (!conf.use_circles))
			{
				Eigen::Vector3d lx, ly, lz;
				Eigen::Vector3d center = pipe[i].state.center;
				double rad = pipe[i].state.radius;

				lx = (pipe[i - 1].tool_orient.pos - center).normalized();
				lz = pipe[i].state.normal;
				ly = lz.cross(lx).normalized();

				if (pipe[i].state.spiral_times != -1)
				{
					points.clear();
					std::vector<Eigen::Vector2d> _points;
					ly = lz.cross((pipe[i - 1].tool_orient.pos - center).normalized()).normalized();
					lx = ly.cross(lz);

					for (int t = 0; t < pipe[i].state.spiral_times - 1; t++)
					{
						CircleLinearApprox(Eigen::Vector2d(1, 0), Eigen::Vector2d(1, 0),
							rad, 0.01, _points, false, false, true);
						for (int k = 0; k < _points.size(); k++)points.push_back(_points[k]);
					}
					CircleLinearApprox(Eigen::Vector2d(1, 0), Eigen::Vector2d(
						(pipe[i].tool_orient.pos - center).dot(lx),
						(pipe[i].tool_orient.pos - center).dot(ly)).normalized(),
						rad, 0.01, _points, false, false);
					for (int k = 0; k < _points.size(); k++)points.push_back(_points[k]);
				}
				else
				{
					CircleLinearApprox(Eigen::Vector2d(1, 0),
						Eigen::Vector2d(
						(pipe[i].tool_orient.pos - center).dot(lx),
							(pipe[i].tool_orient.pos - center).dot(ly)).normalized(),
						rad, 0.01, points, false, false);
				}

				//if(calc_count_to_insert==0)
				//{

				//}else
				{
					double z_step = lz.dot(pipe[i].tool_orient.pos - pipe[i - 1].tool_orient.pos);
					double z_first = pipe[i - 1].tool_orient.pos[2];
					double z_last = pipe[i].tool_orient.pos[2];

					pipe[i].state.mask = PrimitiveMask::LINE;
					int points_count = points.size();
					if (points.size() > 2)
					{
						//TODO очень медленно при большом количествве

						//TODO я убрал pipe.InsertCount(points_count-2,i);
						throw std::exception("not realized");

						int last_point = i + points_count - 2;
						pipe[last_point].tool_orient = pipe[i].tool_orient;


						*(TMachineState*)&pipe[last_point] = *(TMachineState*)&pipe[i];

						pipe[last_point].state.mask = PrimitiveMask::LINE;

						for (int t = 1; t < points_count - 1; t++)
						{
							*(TMachineState*)&pipe[i + t - 1] = *(TMachineState*)&pipe[last_point];
							pipe[i + t - 1].state.mask = PrimitiveMask::LINE;

							if (pipe[i].state.spiral_times != -1)
							{
								pipe[i + t - 1].tool_orient.pos = (lx*points[t][0] + ly * points[t][1])*rad + center;
								pipe[i + t - 1].tool_orient.pos[2] = Funcs::Blend(z_first, z_last, (t) / double(points_count));
							}
							else
								pipe[i + t - 1].tool_orient.pos = (lx*points[t][0] + ly * points[t][1])*rad + center;

							pipe[i + t - 1].tool_orient.dir = pipe[last_point].tool_orient.dir;
						}
					}
				}
			}
			else
			{
			}
		}
	}
}

void TATPProcessor::ValidateKinematics(std::vector<TPipelineElement> &pipe)
{
	//for (int i = 0; i <= pipe.size() - 1; i++)
	//	if (pipe[i].state.mask&PrimitiveMask::CIRCLE
	//		|| pipe[i].state.mask&PrimitiveMask::LINE)
	//	{
	//		for (int k = 0; k < 2; k++)
	//			if (pipe[i].machine_orient.any_C)
	//				pipe[i].machine_orient.valid[k] = true;
	//			else
	//				pipe[i].machine_orient.valid[k] = !AIsInPole(pipe[i].machine_orient.kinematics_pair.variant[k].A);
	//		if (!(pipe[i].machine_orient.valid[0] || pipe[i].machine_orient.valid[1]))
	//			throw exception("Invalid tool orientation");
	//	}
}

void TATPProcessor::SwapVariants(std::vector<TPipelineElement> &pipe)
{
	//for (int i = 0; i < pipe.size() - 1; i++)
	//	if (NeedLinearizeMovementSwap(pipe, i))
	//		Swap(pipe[i + 1].machine_orient.variant[0], pipe[i + 1].machine_orient.variant[1]);
}

//void TATPProcessor::GetCWithMaxX(Eigen::Vector3d tool_pos, double A, double initial_C, double& result_C, Eigen::Vector3d& result_pos)
//{
//	double step = DegToRad(2.0);
//	double dist = 10e20;
//	Eigen::Vector3d init_pos, p0, p1, t;
//	//while(dist>1)
//	{
//		init_pos = ToMachineToolKinematics(tool_pos, A, initial_C);
//		result_pos = init_pos;
//		p0 = ToMachineToolKinematics(tool_pos, A, initial_C - step);
//		p1 = ToMachineToolKinematics(tool_pos, A, initial_C + step);
//		if (p0[0] > init_pos[0] && p0[0] > p1[0])
//		{
//			initial_C -= step;
//			//while((t=ToMachineToolKinematics(tool_pos,A,initial_C-=step))[0]>p0[0])
//			{
//				//p0=t;
//				result_pos = p0;
//			}
//		}
//		else if (p1[0] > init_pos[0] && p1[0] > p0[0])
//		{
//			initial_C += step;
//			//while((t=ToMachineToolKinematics(tool_pos,A,initial_C+=step))[0]>p1[0])
//			{
//				//p1=t;
//				result_pos = p1;
//			}
//		}
//		else
//		{
//		}
//	}
//	result_C = initial_C;
//}
//
//void TATPProcessor::DetermineAnyCKinematics(std::vector<TPipelineElement> &pipe)
//{
//
//	//здесь мы можем использовать различные критерии работы координат при any_C
//	switch (any_C_criteria)
//	{
//	case 0:
//	{
//		//if(pipe[0].machine_orient.any_C)
//		//{
//		//	ToMachineToolKinematics(pipe[0].tool_orient.pos,pipe[0].tool_orient.dir,pipe[0].machine_orient);
//		//}
//		//for(int i=1;i<pipe.size();i++)
//		//	if(pipe[i].mask&PrimitiveMask::CIRCLE
//		//		||pipe[i].mask&PrimitiveMask::LINE)
//		//		if(pipe[i].machine_orient.any_C)
//		//		{
//		//			ToMachineToolKinematics(pipe[i].tool_orient.pos,pipe[i].tool_orient.dir,pipe[i].machine_orient,true,pipe[i-1].machine_orient.variant[0].C);
//		//		}
//	}break;
//	case 1://критерий максимальной координаты по X (для ТФЦ-600)
//	{
//		//double step=DegToRad(2.0);
//		//if(pipe[0].machine_orient.any_C)
//		//{
//		//	ToMachineToolKinematics(pipe[0].tool_orient.pos,pipe[0].tool_orient.dir,pipe[0].machine_orient);
//		//}
//		//for(int i=1;i<pipe.size();i++)
//		//	if(pipe[i].mask&PrimitiveMask::CIRCLE
//		//		||pipe[i].mask&PrimitiveMask::LINE)
//		//		if(pipe[i].machine_orient.any_C)
//		//		{
//		//			double initial_C=pipe[i-1].machine_orient.variant[0].C;
//		//			double result_C;
//		//			TVec<double,3> result_pos;
//
//		//			if(true)
//		//			{
//		//				GetCWithMaxX(pipe[i].tool_orient.pos,pipe[i].machine_orient.variant[0].A,initial_C,result_C,result_pos);
//		//				pipe[i].machine_orient.variant[0].pos=result_pos;
//		//				pipe[i].machine_orient.variant[1].pos=result_pos;
//		//				pipe[i].machine_orient.variant[0].C=result_C;
//		//				pipe[i].machine_orient.variant[1].C=result_C;
//		//			}else
//		//			{
//		//				TVec<double,3> pos0=ToMachineToolKinematics(pipe[i].tool_orient.pos,pipe[i].machine_orient.variant[0].A,initial_C);
//		//				TVec<double,3> pos1=ToMachineToolKinematics(pipe[i].tool_orient.pos,pipe[i].machine_orient.variant[0].A,initial_C+step);
//		//				TVec<double,3> pos2=ToMachineToolKinematics(pipe[i].tool_orient.pos,pipe[i].machine_orient.variant[0].A,initial_C-step);
//
//		//				if(pos0[0]>pos1[0]&&pos0[0]>pos2[0])
//		//				{
//		//					pipe[i].machine_orient.variant[0].pos=pos0;
//		//					pipe[i].machine_orient.variant[1].pos=pos0;
//		//					pipe[i].machine_orient.variant[0].C=initial_C;
//		//					pipe[i].machine_orient.variant[1].C=initial_C;
//		//				}
//		//				if(pos1[0]>pos0[0]&&pos1[0]>pos2[0])
//		//				{
//		//					pipe[i].machine_orient.variant[0].pos=pos1;
//		//					pipe[i].machine_orient.variant[1].pos=pos1;
//		//					pipe[i].machine_orient.variant[0].C=initial_C+step;
//		//					pipe[i].machine_orient.variant[1].C=initial_C+step;
//		//				}
//		//				if(pos2[0]>pos0[0]&&pos2[0]>pos0[0])
//		//				{
//		//					pipe[i].machine_orient.variant[0].pos=pos2;
//		//					pipe[i].machine_orient.variant[1].pos=pos2;
//		//					pipe[i].machine_orient.variant[0].C=initial_C-step;
//		//					pipe[i].machine_orient.variant[1].C=initial_C-step;
//		//				}
//
//		//			}
//		//			//ToMachineToolKinematics(pipe[i].tool_orient.pos,pipe[i].tool_orient.dir,pipe[i].machine_orient,true,pipe[i-1].machine_orient.variant[0].C);
//		//		}
//	}break;
//	case 2://критерий максимальной координаты по X (для ТФЦ-600)
//	{
//
//		//for(int i=1;i<pipe.size();i++)
//		//	if(pipe[i].mask&PrimitiveMask::CIRCLE
//		//		||pipe[i].mask&PrimitiveMask::LINE)
//		//		if(pipe[i].machine_orient.any_C)
//		//		{
//		//			TKinematicsPair<double> pair;
//		//			ToMachineToolKinematics(pipe[i].tool_orient.pos,pipe[i].tool_orient.pos.GetNormalized(),pair);//TODO а если позиция будет равна нулю?
//		//			double result_C=pair.variant[0].C;
//		//			TVec<double,3> pos=ToMachineToolKinematics(pipe[i].tool_orient.pos,pipe[i].machine_orient.variant[0].A,result_C);
//		//			pipe[i].machine_orient.variant[0].pos=pos;
//		//			pipe[i].machine_orient.variant[1].pos=pos;
//		//			pipe[i].machine_orient.variant[0].C=result_C;
//		//			pipe[i].machine_orient.variant[1].C=result_C;
//		//		}
//	}break;
//	default:assert(false);
//	}
//}
//
void TATPProcessor::TraceLine(std::vector<TPipelineElement> &pipe, int line)
{
	//int curr_line = line;
	//if (!pipe[0].machine_orient.valid[curr_line])curr_line = !line;
	//double curr_A = AToMachineRange(pipe[0].machine_orient.variant[curr_line].A);
	//double curr_C = CToMachineRange(pipe[0].machine_orient.variant[curr_line].C);

	//for (int i = 0; i < pipe.size(); i++)
	//{

	//	TKinematicsNode *curr = &pipe[i].machine_orient;

	//	switch (pipe[i].mask)
	//	{
	//	case PrimitiveMask::LINE:
	//	{
	//		if (i == pipe.size() - 1)break;

	//		TKinematicsNode *next = &pipe[i + 1].machine_orient;

	//		double delta_A, delta_C;

	//		//GetMovement(curr->variant[curr_line],next->variant[curr_line],delta_A,delta_C);
	//		IsCCWMove(curr_A, next->variant[curr_line].A, delta_A);
	//		IsCCWMove(curr_C, next->variant[curr_line].C, delta_C);

	//		curr->move_over_pole[line] =
	//			AIsMoveOverPole(curr_A, curr_A + delta_A)
	//			|| CIsMoveOverPole(curr_C, curr_C + delta_C);

	//		curr->change_line[curr_line] = false;

	//		if (curr->move_over_pole[line] || (!next->valid[curr_line]))
	//		{
	//			if (curr->valid[!curr_line])
	//			{
	//				curr->change_line[curr_line] = true;
	//				double new_A = AToMachineRange(curr->variant[!curr_line].A);
	//				double new_C = CToMachineRange(curr->variant[!curr_line].C);

	//				curr->A_pole_change_delta[line] = new_A - curr_A;
	//				curr->C_pole_change_delta[line] = new_C - curr_C;

	//				curr_A += curr->A_pole_change_delta[line];
	//				curr_C += curr->C_pole_change_delta[line];

	//				curr_line = !curr_line;
	//				//GetMovement(curr->variant[curr_line],next->variant[curr_line],delta_A,delta_C);
	//				IsCCWMove(new_A, next->variant[curr_line].A, delta_A);
	//				IsCCWMove(new_C, next->variant[curr_line].C, delta_C);
	//			}
	//			else
	//			{
	//				curr->change_line[curr_line] = false;
	//				//TODO обработка случая когда сменить полюс нельза а надо раскручиваться по C
	//				double new_A = AToMachineRange(curr->variant[curr_line].A);
	//				double new_C = CToMachineRange(curr->variant[curr_line].C);

	//				curr->A_pole_change_delta[line] = new_A - curr_A;
	//				curr->C_pole_change_delta[line] = new_C - curr_C;

	//				curr_A += curr->A_pole_change_delta[line];
	//				curr_C += curr->C_pole_change_delta[line];

	//				IsCCWMove(new_A, curr->variant[curr_line].A, delta_A);
	//				IsCCWMove(new_C, curr->variant[curr_line].C, delta_C);
	//			}

	//			//delta_A+=new_A-curr_A;
	//			//delta_C+=new_C-curr_C;
	//			//
	//			if (!next->valid[curr_line])
	//				throw string("Kinematics hasn't valid variant");
	//		}

	//		curr->A_inc[line] = delta_A;
	//		curr->C_inc[line] = delta_C;

	//		assert(!(AIsMoveOverPole(curr_A, curr_A + delta_A) || CIsMoveOverPole(curr_C, curr_C + delta_C)));
	//		curr_A += delta_A;
	//		curr_C += delta_C;
	//		assert(!(AIsInPole(curr_A) || CIsInPole(curr_C)));
	//	}break;
	//	case PrimitiveMask::CIRCLE:
	//	{
	//	}break;
	//	default:assert(false);
	//	}
	//}
}
//
//void TATPProcessor::SelectBestLine(std::vector<TPipelineElement> &pipe, std::vector<TToolMovementElement> &result_pipe)
//{
//	int line = 0;
//	int curr_line = line;
//	if (!pipe[0].machine_orient.valid[curr_line])curr_line = !line;
//	double curr_A = AToMachineRange(pipe[0].machine_orient.variant[curr_line].A);
//	double curr_C = CToMachineRange(pipe[0].machine_orient.variant[curr_line].C);
//	for (int i = 0; i < pipe.size(); i++)
//	{
//		TKinematicsNode *curr = &pipe[i].machine_orient;
//		switch (pipe[i].mask)
//		{
//		case PrimitiveMask::LINE:
//		{
//			TToolMovementElement t;
//			t.tool_orient = pipe[i].tool_orient;
//			*(TMachineState*)&t = pipe[i];
//			*(TKinematics*)&t = pipe[i].machine_orient.variant[curr_line];
//			t.A = curr_A;
//			t.C = curr_C;
//			t.any_C = pipe[i].machine_orient.any_C;
//
//			result_pipe.push_back(t);
//
//			if (i == pipe.size() - 1)break;
//			TKinematicsNode *next = &pipe[i + 1].machine_orient;
//			if (curr->move_over_pole[line] || (!next->valid[curr_line]))
//			{
//				StandartRetractEngageChangePole(i, curr_line, line, curr_A, curr_C, pipe, result_pipe);
//
//				curr_A += curr->A_pole_change_delta[line];
//				curr_C += curr->C_pole_change_delta[line];
//				if (curr->change_line[curr_line])
//					curr_line = !curr_line;
//			}
//			curr_A += curr->A_inc[line];
//			curr_C += curr->C_inc[line];
//		}break;
//		case PrimitiveMask::CIRCLE:
//		{
//			TToolMovementElement t;
//			t.tool_orient = pipe[i].tool_orient;
//			*(TMachineState*)&t = pipe[i];
//			*(TKinematics*)&t = pipe[i].machine_orient.variant[curr_line];
//			t.A = curr_A;
//			t.C = curr_C;
//			result_pipe.push_back(t);
//		}break;
//		default:assert(false);
//		}
//
//	}
//}

double TATPProcessor::MovementDistance(TKinematics k0, TKinematics k1)//в отличие от предыдущей версии считает перемещение по всем координатам в линейном виде
{
	double result = 0.0;
	for (int i = 0; i < 5; i++)
		result += Funcs::sqr(k0.v[i] + k1.v[i]);
	return sqrt(result);
}


void TATPProcessor::CalculateMoveTime(std::vector<TToolMovementElement> &pipe, double &fast_movement_time, double &work_movement_time)
{
	fast_movement_time = 0;
	work_movement_time = 0;

	pipe[0].move_distance = 0;
	pipe[0].move_time = 0;
	for (int i = 1; i <= pipe.size() - 1; i++)
	{

		switch (pipe[i].state.mask)
		{
		case PrimitiveMask::LINE:
		{
			pipe[i].move_distance = MovementDistance(pipe[i - 1].kinematics, pipe[i].kinematics);
		}break;
		case PrimitiveMask::CIRCLE:
		{
			Eigen::Vector3d lx, ly, lz;
			Eigen::Vector3d center = pipe[i].state.center;
			double rad = pipe[i].state.radius;

			lx = (pipe[i - 1].tool_orient.pos - center).normalized();
			lz = pipe[i].state.normal;
			ly = lz.cross(lx);

			pipe[i].move_distance = rad * abs(AngleFromDir(
				Eigen::Vector2d(
				(pipe[i].tool_orient.pos - center).dot(lx),
					(pipe[i].tool_orient.pos - center).dot(ly)).normalized()).AsRad())
				+ (pipe[i].state.spiral_times != -1 ? 2 * M_PI*pipe[i].state.spiral_times : 0);
		}break;
		default:assert(false);
		}
		pipe[i].move_time = pipe[i].move_distance / (pipe[i].state.rapid ? processor_conf.rapid_feed : pipe[i].state.feed)*60.0;
		if (pipe[i].state.rapid)
			fast_movement_time += pipe[i].move_time;
		else
			work_movement_time += pipe[i].move_time;
	}
}

void TATPProcessor::CalculateContourSpeed(std::vector<TToolMovementElement> &pipe)
{
	pipe[0].contour_correct_feed = pipe[0].state.feed;
	for (int i = 1; i <= pipe.size() - 1; i++)
	{
		double dist = (pipe[i].tool_orient.pos - (pipe[i - 1].tool_orient.pos)).norm();

		/*pipe[i].contour_correct_feed=ClampMax(contour_max_feed,
			pipe[i].feed*
			(dist<ortho_vec_epsilon?(1.0):(pipe[i].move_distance/dist)));*/
		pipe[i].contour_correct_feed = pipe[i].state.feed;
	}
}
//
//bool TATPProcessor::MergeFrames(std::vector<TToolMovementElement> &pipe, int window_start, int window_end)
////return - произошло удаление элемента
//{
//	int min_k = -1;
//	double min_time = 0;
//	for (int k = window_start + 1; k <= window_end - 1; k++)
//	{
//		double temp = pipe[k].move_time + pipe[k + 1].move_time;
//		if ((min_k == -1 || temp < min_time) && !pipe[k].base_element)
//		{
//			min_time = temp;
//			min_k = k;
//		}
//	}
//	if (min_k != -1)
//	{
//		pipe[min_k + 1].move_time += pipe[min_k].move_time;
//		pipe.Delete(min_k);
//		return true;
//	}
//	else return false;
//}
//
//void TATPProcessor::Compress(std::vector<TToolMovementElement> &pipe)
//{
//	int i = 0;
//	while (true)
//	{
//		while (pipe[i].rapid&&i < pipe.size() - 1)i++;
//		if (i == pipe.size() - 1)break;
//		double time = 0;
//		int window_end = i;
//		while (window_end - i < frames_on_1sec_max&&window_end < pipe.size() - 1)
//		{
//			time += pipe[window_end].move_time;
//			window_end++;
//		}
//		while (window_end <= pipe.size() - 1)
//		{
//			if (time < 1.0&&MergeFrames(pipe, i, window_end))
//			{
//				if (window_end > pipe.size() - 1)return;
//				time += pipe[window_end].move_time;//если было удаление кадр то на этом месте теперь новый кадр
//			}
//			else
//			{
//				time -= pipe[i].move_time;
//				i++;
//				window_end++;
//				if (window_end > pipe.size() - 1)break;
//				time += pipe[window_end].move_time;
//			}
//		}
//	}
//}
//
//void TATPProcessor::TryAlignInvalidVariants(std::vector<TPipelineElement> &pipe)
//{
//	for (int i = 0; i <= pipe.size() - 1; i++)
//		if (!pipe[i].machine_orient.any_C)
//		{
//			for (int k = 0; k < 2; k++)
//				if ((!pipe[i].machine_orient.valid[k]) && (!AIsInPole(pipe[i].machine_orient.variant[k].A)))
//				{
//					try {
//						pipe[i].machine_orient.variant[k].C = CToMachineRange(pipe[i].machine_orient.variant[k].C);
//						pipe[i].machine_orient.valid[k] = !AIsInPole(pipe[i].machine_orient.variant[k].A);
//					}
//					catch (exception e)
//					{
//					}
//					break;
//				}
//		}
//}

void TATPProcessor::Subdivide(std::vector<TToolMovementElement> &pipe)
{
	std::vector<Eigen::Vector2i> count_after_pair;
	count_after_pair.reserve(pipe.size() / 3);
	for (int i = 0; i < pipe.size() - 1; i++)
	{
		if (pipe[i].state.mask == PrimitiveMask::LINE)
		{
			double tol = 1000000;
			//while(tol>tolerance)
			{
				TToolMovementElement new_el;
				tol = GetInterpolationTolerance(pipe[i].kinematics, pipe[i + 1].kinematics, new_el.tool_orient);
				bool need_subdivide = false;
				if (pipe[i].state.rapid)
					need_subdivide = tol > processor_conf.rapid_tolerance;
				else need_subdivide = tol > processor_conf.tolerance;
				//TODO где брать any_C он же в TKinematicsNode
				//if (need_subdivide && ((conf.subdivide_only_any_C == pipe[i].any_C) || !conf.subdivide_only_any_C))
				//{
				//	int new_el_high = (pipe[i].state.rapid ? (tol / processor_conf.rapid_tolerance) : (tol / processor_conf.tolerance)) + 1;
				//	if (new_el_high > 1)
				//		count_after_pair.push_back(Eigen::Vector2i((new_el_high + 1) - 2, i));
				//	//(new_el_high+1)-2 уменьшается на 2 т.к. начальная и конечная точка уже имеется и надо добавить только промежуточные
				//}
			}
		}
	}
	for (int i = 0; i < count_after_pair.size(); i++)
		pipe.push_back(TToolMovementElement());
	int off = 0, curr_pair = 0;
	for (int i = 0; i < pipe.size() - 1; i++)
	{
		if (curr_pair > count_after_pair.size() - 1)break;
		switch (pipe[i].state.mask)
		{
		case PrimitiveMask::LINE:
			if (count_after_pair[curr_pair][1] + off == i)
			{
				int new_el_high = count_after_pair[curr_pair][0] + 1;

				int count_inserted = count_after_pair[curr_pair][0];
				off += count_inserted;


				Eigen::Vector3d p0, p1, d0, d1;
				double a0, a1, c0, c1;
				p0 = pipe[i].tool_orient.pos;
				p1 = pipe[i + 1 + count_inserted].tool_orient.pos;
				d0 = pipe[i].tool_orient.dir.normalized();
				d1 = pipe[i + 1 + count_inserted].tool_orient.dir.normalized();
				c0 = pipe[i].kinematics.v[ids.rot_node[1]];
				c1 = pipe[i + 1 + count_inserted].kinematics.v[ids.rot_node[1]];
				a0 = pipe[i].kinematics.v[ids.rot_node[0]];
				a1 = pipe[i + 1 + count_inserted].kinematics.v[ids.rot_node[0]];

				//pipe.InsertCount((new_el_high+1)-2,i);
				throw std::exception("not realized");

				for (int t = 1; t < new_el_high; t++)
				{
					TToolMovementElement new_el;
					new_el.tool_orient.pos = p0 * (new_el_high - t) / new_el_high + p1 * t / new_el_high;
					new_el.state.feed = pipe[i].state.feed;
					new_el.state.mask = pipe[i].state.mask;
					new_el.state.rapid = pipe[i].state.rapid;
					new_el.state.color = pipe[i].state.color;
					new_el.state.cutcom = pipe[i].state.cutcom;
					new_el.state.base_element = false;
					new_el.kinematics.v[ids.rot_node[0]] = a0 * (new_el_high - t) / new_el_high + a1 * t / new_el_high;
					new_el.kinematics.v[ids.rot_node[1]] = c0 * (new_el_high - t) / new_el_high + c1 * t / new_el_high;
					new_el.tool_orient.dir = machine.GetToolDirFromMachineToolKinematics(new_el.kinematics);
					new_el.tool_orient.pos = machine.ToMachineToolKinematics(new_el.tool_orient.pos, new_el.kinematics);
					pipe[i + t] = new_el;
				}
				i += new_el_high - 1;

				curr_pair++;
			}break;
		case PrimitiveMask::CIRCLE:
		{
		}break;
		default:assert(false);
		}
	}
}

void TATPProcessor::CheckError(TKinematics kinematics, Eigen::Vector3d original_pos, Eigen::Vector3d original_dir, double& tol0, double& tol1)
{
	Eigen::Vector3d pos, dir;
		machine.FromMachineToolKinematics(kinematics, pos, dir);
		tol0 = (original_dir - dir).norm();
		tol1 = (original_pos - pos).norm();

		if (tol0 > conf.inverse_kinemtatics_tol)
			throw std::exception("Превышение inverse_kinemtatics_tol для направления инструмента");
		if (tol1 > conf.inverse_kinemtatics_tol)
			throw std::exception("Превышение inverse_kinemtatics_tol для позиции");
}

void TATPProcessor::ValidateKinematicsInverseAlgorithm(std::vector<TPipelineElement> &pipe)
{
	double max_pos_tol = 0;
	double max_dir_tol = 0;

	for (int i = 0; i < pipe.size() - 1; i++)
		if (pipe[i].state.mask == PrimitiveMask::CIRCLE
			|| pipe[i].state.mask == PrimitiveMask::LINE)
		{
			for (int k = 0; k < 2; k++)
			{
				double tol0, tol1;
				CheckError(pipe[i].machine_orient.kinematics_pair.variant[k], 
					pipe[i].tool_orient.pos, pipe[i].tool_orient.dir, tol0, tol1);
				
				if (tol0 > max_dir_tol)
					max_dir_tol = tol0;
				if (tol1 > max_pos_tol)
					max_pos_tol = tol1;
			}
		}
	printf("pos_tol = %.9lf \tdir_tol = %.9lf\n", max_pos_tol, max_dir_tol);
}

void TATPProcessor::MakePipe(std::vector<TPipelineElement> &pipe, std::vector<TCLSFToken> &atp_tokens)
{
	pipe.reserve(atp_tokens.size());
	Eigen::Vector3d curr_dir(0, 0, 1);
	double curr_feed = 100;
	double curr_spndl_rpm = 0;
	int curr_cutcom = 0;//0-off 1-left 2-right
	bool clw = true;
	std::string curr_aux;
	std::string curr_path_cs_name;
	std::string curr_path_name;
	bool rapid = false;
	int curr_color = 0;
	if (!conf.use_tool_length_correction)
		conf.tool_length = 0;
	for (int i = 0; i < atp_tokens.size(); i++)
	{
		try {
			if (
				atp_tokens[i].Name() == "GOTO" ||
				atp_tokens[i].Name() == "FROM" ||
				atp_tokens[i].Name() == "GOHOME")
			{
				Eigen::Vector3d pos, dir;
				if (atp_tokens[i].ParamsCount() != 3 && atp_tokens[i].ParamsCount() != 6)
					throw std::string("Not enough parameters in GOTO");
				for (int k = 0; k < 3; k++)pos[k] = atof(atp_tokens[i][k].c_str());
				if (atp_tokens[i].ParamsCount() <= 3)
					dir = curr_dir;
				else
				{
					for (int k = 0; k < 3; k++)dir[k] = atof(atp_tokens[i][k + 3].c_str());
					curr_dir = dir;
				}
				TPipelineElement t;
				t.state.auxfun = curr_aux;
				t.tool_orient.pos = pos;
				t.tool_orient.dir = dir;
				t.state.mask = PrimitiveMask::LINE;
				t.state.feed = curr_feed;
				t.state.spndl_rpm = curr_spndl_rpm;
				t.state.clw = clw;
				if (atp_tokens[i].Name() == "GOHOME")rapid = true;
				if (atp_tokens[i].Name() == "FROM")rapid = true;
				t.state.rapid = rapid;
				t.state.base_element = true;
				t.state.color = curr_color;
				t.state.cutcom = curr_cutcom;
				t.state.path_name = curr_path_name;
				pipe.push_back(t);
				if (rapid)rapid = false;
				curr_aux = "";
			}
			else if (atp_tokens[i].Name() == "CIRCLE")
			{
				Eigen::Vector3d center, normal;
				double radius;
				TPipelineElement t;
				t.state.spiral_times = -1;
				if (atp_tokens[i].ParamsCount() < 11)
					throw std::string("Not enough parameters in CIRCLE");
				for (int k = 0; k < 3; k++)center[k] = atof(atp_tokens[i][k].c_str());
				for (int k = 0; k < 3; k++)normal[k] = atof(atp_tokens[i][k + 3].c_str());
				if (atp_tokens[i].ParamsCount() == 13 && atp_tokens[i][11] == "TIMES")
				{
					t.state.spiral_times = atof(atp_tokens[i][12].c_str());
				}
				radius = atof(atp_tokens[i][6].c_str());

				t.state.auxfun = curr_aux;
				t.tool_orient.dir = curr_dir;
				t.state.center = center;
				t.state.normal = normal;
				t.state.radius = radius;
				t.state.mask = PrimitiveMask::CIRCLE;
				t.state.feed = curr_feed;
				t.state.spndl_rpm = curr_spndl_rpm;
				t.state.clw = clw;
				t.state.rapid = rapid;
				t.state.base_element = true;
				t.state.color = curr_color;
				t.state.cutcom = curr_cutcom;
				t.state.path_name = curr_path_name;
				pipe.push_back(t);
				curr_aux = "";
			}
			else if (atp_tokens[i].Name() == "FEDRAT")
			{
				if (atp_tokens[i].ParamsCount() == 1)
					curr_feed = atof(atp_tokens[i][0].c_str());
				else
					curr_feed = atof(atp_tokens[i][1].c_str());
				rapid = false;
			}
			else if (atp_tokens[i].Name() == "SPINDL")
			{
				if (atp_tokens[i].ParamsCount() == 3)//TODO при смене инструмента у нас сбрасываются обороты шпинделя
				{
					curr_spndl_rpm = atof(atp_tokens[i][1].c_str());
					clw = (atp_tokens[i][2] == "CLW ");//TODO убирать пробельные символы
				}
			}
			else if (atp_tokens[i].Name() == "PAINT")
			{
				if (atp_tokens[i].ParamsCount() == 2 && (atp_tokens[i][0] == "COLOR"))
					curr_color = atof(atp_tokens[i][1].c_str());
			}
			else if (atp_tokens[i].Name() == "RAPID")
			{
				rapid = true;
			}
			else if (atp_tokens[i].Name() == "TLDATA")
			{
				if (atp_tokens[i][0] == "MILL")
					conf.tool_length = atof(atp_tokens[i][3].c_str());
				else if (atp_tokens[i][0] == "DRILL")
					conf.tool_length = atof(atp_tokens[i][4].c_str());
				else if (atp_tokens[i][0] == "TCUTTER")
					conf.tool_length = atof(atp_tokens[i][4].c_str());
				else throw std::string("unknown tool type");
				if (!conf.use_tool_length_correction)
					conf.tool_length = 0;
			}
			else if (atp_tokens[i].Name() == "AUXFUN")
			{
				if (atp_tokens[i][0] == "0")//ф-я вывода текста
				{
					curr_aux += "\n";
					for (int k = 1; k < atp_tokens[i].ParamsCount(); k++)
					{
						curr_aux += atp_tokens[i][k];
						if (k != atp_tokens[i].ParamsCount() - 1)
							curr_aux += ',';
					}
				}
				//if (atp_tokens[i][0] == "1")//включение спец ф-и oriented_from_goto
				//{
				//	//oriented_from_goto=true;
				//	//oriented_from_goto_orientation=lexical_cast<double>(atp_tokens[i][1].c_str());
				//	TOrientedFromGoto t;
				//	t.path_name = curr_path_name;
				//	t.orientation = atof(atp_tokens[i][1].c_str());
				//	oriented_from_goto.push_back(t);
				//}
			}
			else if (atp_tokens[i].Name() == "CUTCOM")
			{
				if (atp_tokens[i][0] == "LEFT")
				{
					curr_cutcom = 1;
				}
				else if (atp_tokens[i][0] == "RIGHT")
				{
					curr_cutcom = 2;
				}
				else if (atp_tokens[i][0] == "OFF")
				{
					curr_cutcom = 0;
				}
			}
			else if (atp_tokens[i].Name() == "NX_PROCESSOR_SET_CS_G")
			{
				conf.local_CS_G_index = atoi(atp_tokens[i][0].c_str());
			}
			else if (atp_tokens[i].Name() == "NX_PROCESSOR_PATH_CS_NAME")
			{
				curr_path_cs_name = atp_tokens[i][0].c_str();
			}
			else if (atp_tokens[i].Name() == "MSYS")
			{
				int local_g_id = 54;
				if (curr_path_cs_name == "")
				{
				}
				else
				{
					char* set_cs_template_start(".G");
					char* set_cs_template_end(".");
					size_t found = curr_path_cs_name.find(set_cs_template_start);
					if (found != std::string::npos)
					{
						size_t found_last = curr_path_cs_name.find(set_cs_template_end, found + 1);
						if (found_last > found)
						{
							std::string str_id = curr_path_cs_name.substr(found + strlen(set_cs_template_start), found_last - (found + strlen(set_cs_template_start)));
							local_g_id = atoi(str_id.c_str());
						}
					}
				}
				char buf[256];
				if (local_g_id > 600)
					sprintf(buf, "\nzeros(%i)\n", local_g_id);
				else
					sprintf(buf, "\nG%i\n", local_g_id);
				curr_aux += buf;
			}
			else if (atp_tokens[i].Name() == "TOOL PATH")
			{
				curr_path_name = atp_tokens[i][0].c_str();
				conf.tool_name = atp_tokens[i][2].c_str();
			}
			else if (atp_tokens[i].Name() == "END-OF-PATH")
			{
				curr_path_name = "";
			}
			//}catch(bad_cast& e)
			//{
			//	cout << (boost::format("ATP parser Error in line %i: %s\n")%i%e.what()).str();
		}
		catch (std::string& e)
		{
			//cout << (boost::format("ATP parser Error in line %i: %s\n")%i%e).str();
		}
	}
}

void TATPProcessor::PassThrough(std::vector<TCLSFToken> &atp_tokens, std::vector<TToolMovementElement> &result_pipe, double &fast_movement_time, double &work_movement_time)
{
	std::vector<TPipelineElement> pipe;
	MakePipe(pipe, atp_tokens);

	if (pipe.size() == 0)return;

	//std::vector<TToolMovementElement> result_pipe;
	bool need_rebuild = false;
	int max_rebuilds = 1;
	int rebuild = 0;

	Clear(pipe);

	CalcCircleParameters(pipe);
	CalcKinematics(pipe);
	ValidateKinematicsInverseAlgorithm(pipe);
	////FindAnyCBeetweenKinematics(pipe);
	//DetermineAnyCKinematics(pipe);
	ValidateKinematics(pipe);
	//TryAlignInvalidVariants(pipe);

	//TraceLine(pipe, 0);
	////TraceLine(pipe,1);
	//SelectBestLine(pipe, result_pipe);
	//if (conf.use_subdivision)
	//	Subdivide(result_pipe);//TODO для ТФЦ600 при разбивке поворота по C нужное перемещение по дуге не совпадает с прямой между позициями поэтому скачет X

	//CalculateMoveTime(result_pipe, fast_movement_time, work_movement_time);
	//CalculateContourSpeed(result_pipe);
	////Compress(result_pipe); //пока что очень медленно
}