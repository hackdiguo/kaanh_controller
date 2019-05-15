#ifndef KAANH_H_
#define KAANH_H_

#include <memory>
#include <aris.hpp>
#include"iir.h"

//statemachine//
# define M_RUN 0	//手动单步执行
# define READ_RT_DATA 1		//监控实时数据
# define READ_XML 2		//监控实时数据
# define A_RUN 3	//自动执行
# define A_QUIT 4	//退出自动执行，返回到手动模式
# define buffer_length 800
//statemachine//

// \brief 机器人命名空间
// \ingroup aris
//


namespace kaanh
{
	//电缸力检测参数声明
	//const std::string xmlpath = "C:\\Users\\kevin\\Desktop\\aris_rokae\\ServoPressorCmdList.xml";
	constexpr double ea_a = 3765.8, ea_b = 1334.8, ea_c = 45.624, ea_gra = 24, ea_index = -6, ea_gra_index = 36;  //电缸电流换算压力的系数，ea_k表示比例系数，ea_b表示截距，ea_offset表示重力影响量，ea_index表示电流扭矩系数=额定扭矩*6.28*减速比/导程/1000//

	//其他参数和函数声明 
	using Size = std::size_t;
	constexpr double PI = 3.141592653589793;
	auto createModelRokaeXB4(const double *robot_pm = nullptr)->std::unique_ptr<aris::dynamic::Model>;
	auto createControllerRokaeXB4()->std::unique_ptr<aris::control::Controller>;
	auto createPlanRootRokaeXB4()->std::unique_ptr<aris::plan::PlanRoot>;

	class moveC_3P : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		explicit moveC_3P(const std::string &name = "moveC_3P_plan");
		ARIS_REGISTER_TYPE(moveC_3P);
	};





	class moveC_CE : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		explicit moveC_CE(const std::string &name = "moveC_CE_plan");
		ARIS_REGISTER_TYPE(moveC_CE);
	};

	class moveC_RE : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		explicit moveC_RE(const std::string &name = "moveC_RE_plan");
		ARIS_REGISTER_TYPE(moveC_RE);
	};





	class moveL_Cos : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		explicit moveL_Cos(const std::string &name = "moveL_Cos_plan");
		ARIS_REGISTER_TYPE(moveL_Cos);
	};

	class moveJ_Cos : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		explicit moveJ_Cos(const std::string &name = "moveJ_Cos_plan");
		ARIS_REGISTER_TYPE(moveJ_Cos);
	};

	class moveJ_CosN : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		explicit moveJ_CosN(const std::string &name = "moveJ_CosN_plan");
		ARIS_REGISTER_TYPE(moveJ_CosN);
	};

	class moveJ_T : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		explicit moveJ_T(const std::string &name = "moveJ_T_plan");
		ARIS_REGISTER_TYPE(moveJ_T);
	};

	class MoveTTT : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		explicit MoveTTT(const std::string &name = "MoveTTT_plan");
		ARIS_REGISTER_TYPE(MoveTTT);
	};

	class moveJM_T : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		explicit moveJM_T(const std::string &name = "moveJM_T_plan");
		ARIS_REGISTER_TYPE(moveJM_T);
	};

	class moveJM_TQ : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		explicit moveJM_TQ(const std::string &name = "moveJM_TQ_plan");
		ARIS_REGISTER_TYPE(moveJM_TQ);
	};

}

#endif
