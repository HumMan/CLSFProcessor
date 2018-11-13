#include "kinematicsConf.h"

namespace CLSFProcessor
{

	enum PrimitiveMask
	{
		LINE = 1,
		CIRCLE = 2,
	};

	struct TToolOrientation
	{
		Eigen::Vector3d pos, dir;
	};

	typedef double TKinematics[5];

	struct TKinematicsPair
	{
		TKinematics variant[2];//варианты кинематики станка
		bool has_undet_coord;//ось А вертикальна -> значение C может быть любое
		int undet_coord_id;
	};

	double AngleBetween(Eigen::Vector3d v0, Eigen::Vector3d v1);

	double AngleFromDir(const Eigen::Vector2d& v);

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

	class TUniversal5axis
	{
	private:
		CLSFProcessor::Conf::TFiveAxis conf;
		double tool_length;
	public:
		TUniversal5axis(CLSFProcessor::Conf::TFiveAxis use_conf, double use_tool_length = 0);
		void SetToolLength(double use_tool_length);
		Eigen::Vector3d ForwardLinearNode(Eigen::Vector3d p, Eigen::Vector3d axis, double value);
		Eigen::Vector3d InverseLinearNode(Eigen::Vector3d machine_p, Eigen::Vector3d axis, double value);
		Eigen::Vector3d ForwardRotateNode(Eigen::Vector3d p, Eigen::Vector3d off, Eigen::Vector3d axis, double angle);
		Eigen::Vector3d InverseRotateNode(Eigen::Vector3d machine_p, Eigen::Vector3d off, Eigen::Vector3d axis, double angle);
		Eigen::Vector3d ForwardRotateNode(Eigen::Vector3d dir, Eigen::Vector3d axis, double angle);
		Eigen::Vector3d InverseRotateNode(Eigen::Vector3d machine_dir, Eigen::Vector3d axis, double angle);
		//a0,a1 - углы на которые надо повернуть вектор p вокруг оси curr чтобы он полностью оказался в плоскости перпенидкулярной next
		//result - false если такого поворота не существует
		//any_rotation - вектор curr лежит в плоскости перпенд. next и совпадает с вектором curr
		bool GetRotationToNextPlane(Eigen::Vector3d curr, Eigen::Vector3d next, Eigen::Vector3d p, double rotations[], bool &any_rotation);

		//на какой угол надо повернуть v0 вокруг axis чтобы совместить его с v1
		double AngleBetweenVectors(Eigen::Vector3d axis, Eigen::Vector3d v0, Eigen::Vector3d v1);

		//вектор направления инструмента в системе координат детали для заданных машинных угловых координат
		//направлен от фланца к концу инструмента
		Eigen::Vector3d GetToolDirFromMachineToolKinematics(double* coords);
		Eigen::Vector3d ToMachineToolKinematics(Eigen::Vector3d tool_pos, double* coord);

		//use_fixed_C - используется для определения any_C, на выходе конкретная кинематика для данного fixed_C (должен быть от 0 до pi)
		void ToMachineToolKinematics(Eigen::Vector3d tool_pos, Eigen::Vector3d tool_dir,
			TKinematicsPair& result);
			

		void FromMachineToolKinematics(TKinematics source,
			Eigen::Vector3d &tool_pos, Eigen::Vector3d &tool_dir);
	};
}