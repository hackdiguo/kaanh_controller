#ifndef IIR_H_
#define IIR_H_

#include <vector>

# define FORE_VEL_LENGTH 20	//速度平均值滤波buffer长度
# define MEDIAN_LENGTH 41	//中值滤波buffer长度

namespace IIR_FILTER
{ 
	class IIR
	{
		private:
			std::vector<double> m_pNum;
			std::vector<double> m_pDen;
		public:
			std::vector<double> m_px;
			std::vector<double> m_py;
			int m_num_order;
			int m_den_order;
			IIR();
			void setPara(std::vector<double> &num, std::vector<double> &den);
			double filter(double data);
	};
	//IIR滤波器设计参数//
	const double num[] = { 0.05,0.05,0.05,0.05,0.05,0.05,0.05,0.05,0.05,0.05,0.05,0.05,0.05,0.05,0.05,0.05,0.05,0.05,0.05,0.05};

	const double den[] = { 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0 };
}

#endif