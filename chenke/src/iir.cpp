#include"iir.h"
#include<stdio.h>


using namespace IIR_FILTER;

IIR::IIR()
{
	m_pNum.clear();
	m_pDen.clear();
	m_px.clear();
	m_py.clear();
	m_num_order = -1;
	m_den_order = -1;
};
/** \brief
 *
 * \param num 分子多项式的系数，升序排列,num[0] 为常数项
 * \param m 分子多项式的阶数
 * \param den 分母多项式的系数，升序排列,den[0] 为常数项
 * \param m 分母多项式的阶数
 * \return
 */
void IIR::setPara(std::vector<double> &num, std::vector<double> &den)
{
	m_num_order = num.size();
	m_den_order = den.size();

	m_pNum.resize(m_num_order);
	m_pDen.resize(m_den_order);

	m_px.resize(m_num_order);
	m_py.resize(m_den_order);

	for (int i = 0; i < m_num_order; i++)
	{
		m_pNum[i] = num[i];
		m_px[i] = 0.0;
	}
	for (int i = 0; i < m_den_order; i++)
	{
		m_pDen[i] = den[i];
		m_py[i] = 0.0;
	}
}

/** \brief 滤波函数，采用直接I型结构
 *
 * \param data 传入输入数据
 * \return 滤波后的结果
 */
double IIR::filter(double data)
{
	m_py[0] = 0.0; // 存放滤波后的结果
	m_px[0] = data;
	for (int i = 0; i < m_num_order; i++)
	{
		m_py[0] = m_py[0] + m_pNum[i] * m_px[i];
	}
	for (int i = 1; i < m_den_order; i++)
	{
		m_py[0] = m_py[0] - m_pDen[i] * m_py[i];
	}
	for (int i = m_num_order-1; i >= 1; i--)
	{
		m_px[i] = m_px[i - 1];
	}
	for (int i = m_den_order-1; i >= 1; i--)
	{
		m_py[i] = m_py[i - 1];
	}
	return m_py[0];
}
