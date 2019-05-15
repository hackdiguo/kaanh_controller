﻿#include <iostream>
#include <aris.hpp>
#include "kaanh.h"
#include<atomic>
#include<string>
#include<filesystem>


std::atomic_bool is_automatic = false;
using namespace aris::dynamic;

double fce_data[buffer_length], fce_send[buffer_length];
int data_num = 0, data_num_send = 0;
std::vector<std::vector<std::string>> plantrack(6, std::vector<std::string>());
std::atomic_int which_di = 0;

auto xmlpath = std::filesystem::absolute(".");
const std::string xmlfile = "rokae.xml";

int main(int argc, char *argv[])
{
    std::cout <<"new"<<std::endl;

    xmlpath = xmlpath / xmlfile;
    std::cout<< xmlpath <<std::endl;
	auto&cs = aris::server::ControlServer::instance();
	auto port = argc < 2 ? 5866 : std::stoi(argv[1]);

	//生成rokae.xml文档
	/*
	cs.resetController(kaanh::createControllerRokaeXB4().release());
	cs.resetModel(aris::dynamic::createModelRokaeXB4().release());
	cs.resetPlanRoot(kaanh::createPlanRootRokaeXB4().release());
	cs.resetSensorRoot(new aris::sensor::SensorRoot);

	//std::cout << cs.controller().xmlString() << std::endl;

	//末端位置加0.1的工件长度//
	double pe[6];
	cs.model().generalMotionPool()[0].makI().getPe(
    cs.model().generalMotionPool()[0].makI().fatherPart(),
    pe);

    pe[0] += 0.0;
	cs.model().generalMotionPool()[0].makI().setPrtPe(pe);
	cs.model().generalMotionPool()[0].makJ();
	cs.model().solverPool()[0].allocateMemory();

	cs.saveXmlFile(xmlpath.string().c_str());
    */

    cs.loadXmlFile(xmlpath.string().c_str());

	cs.start();

	// interaction //
	std::list<std::tuple<aris::core::Msg, std::shared_ptr<aris::plan::PlanTarget>>> result_list;
	std::mutex result_mutex;

	aris::core::Socket socket("server", "", "5866", aris::core::Socket::WEB);
	socket.setOnReceivedMsg([&](aris::core::Socket *socket, aris::core::Msg &msg)->int
	{
		std::string msg_data = msg.toString();

		static int cout_count = 0;
		if (++cout_count % 10 == 0)
			//std::cout << "recv:" << msg_data << std::endl;

		LOG_INFO << "the request is cmd:"
			<< msg.header().msg_size_ << "&"
			<< msg.header().msg_id_ << "&"
			<< msg.header().msg_type_ << "&"
			<< msg.header().reserved1_ << "&"
			<< msg.header().reserved2_ << "&"
			<< msg.header().reserved3_ << ":"
			<< msg_data << std::endl;

		try
		{
			std::stringstream ss(msg_data);
			for (std::string cmd; std::getline(ss, cmd);)
			{
				auto result = cs.executeCmd(aris::core::Msg(cmd));

				std::unique_lock<std::mutex> l(result_mutex);
				result_list.push_back(std::make_tuple(msg, result));
			}
		}
		catch (std::exception &e)
		{
			std::cout << e.what() << std::endl;
			LOG_ERROR << e.what() << std::endl;

			try
			{
				aris::core::Msg m;
				m.setMsgID(msg.header().msg_id_);
				m.setType(msg.header().msg_type_);
				m.header().reserved1_ = msg.header().reserved1_;
				m.header().reserved2_ = msg.header().reserved2_;
				m.header().reserved3_ = msg.header().reserved3_;
				socket->sendMsg(m);
			}
			catch (std::exception &e)
			{
				std::cout << e.what() << std::endl;
				LOG_ERROR << e.what() << std::endl;
			}
		}

		return 0;
	});
	socket.setOnReceivedConnection([](aris::core::Socket *sock, const char *ip, int port)->int
	{
		std::cout << "socket receive connection" << std::endl;
		LOG_INFO << "socket receive connection:\n"
			<< std::setw(aris::core::LOG_SPACE_WIDTH) << "|" << "  ip:" << ip << "\n"
			<< std::setw(aris::core::LOG_SPACE_WIDTH) << "|" << "port:" << port << std::endl;
		return 0;
	});
	socket.setOnLoseConnection([](aris::core::Socket *socket)
	{
		std::cout << "socket lose connection" << std::endl;
		LOG_INFO << "socket lose connection" << std::endl;
		for (;;)
		{
			try
			{
				socket->startServer("5866");
				break;
			}
			catch (std::runtime_error &e)
			{
				std::cout << e.what() << std::endl << "will try to restart server socket in 1s" << std::endl;
				LOG_ERROR << e.what() << std::endl << "will try to restart server socket in 1s" << std::endl;
				std::this_thread::sleep_for(std::chrono::seconds(1));
			}
		}
		std::cout << "socket restart successful" << std::endl;
		LOG_INFO << "socket restart successful" << std::endl;

		return 0;
	});
	socket.startServer(std::to_string(port));

	std::thread result_thread([&]()
	{
		while (true)
		{
			std::unique_lock<std::mutex> lck(result_mutex);
			for (auto result = result_list.begin(); result != result_list.end();)
			{
				auto cmd_ret = std::get<1>(*result);
				if (cmd_ret->finished.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready)
				{
					auto ret = cmd_ret->ret;
					auto &msg = std::get<0>(*result);

					if (auto str = std::any_cast<std::string>(&ret))
					{
						try
						{
							aris::core::Msg ret_msg(*str);

							ret_msg.setMsgID(msg.header().msg_id_);
							ret_msg.setType(msg.header().msg_type_);
							ret_msg.header().reserved1_ = msg.header().reserved1_;
							ret_msg.header().reserved2_ = msg.header().reserved2_;
							ret_msg.header().reserved3_ = msg.header().reserved3_;
							socket.sendMsg(ret_msg);
						}
						catch (std::exception &e)
						{
							std::cout << e.what() << std::endl;
							LOG_ERROR << e.what() << std::endl;
						}
					}
					else
					{
						try
						{
							aris::core::Msg ret_msg;
							ret_msg.setMsgID(msg.header().msg_id_);
							ret_msg.setType(msg.header().msg_type_);
							ret_msg.header().reserved1_ = msg.header().reserved1_;
							ret_msg.header().reserved2_ = msg.header().reserved2_;
							ret_msg.header().reserved3_ = msg.header().reserved3_;
							socket.sendMsg(ret_msg);
						}
						catch (std::exception &e)
						{
							std::cout << e.what() << std::endl;
							LOG_ERROR << e.what() << std::endl;
						}
					}
				}

				result_list.erase(result++);
			}
			lck.unlock();

			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}
	});

	/*
	aris::core::Socket udp_socket("server", "", "5867", aris::core::Socket::UDP_RAW);
	udp_socket.setOnReceivedRawData([&](aris::core::Socket *socket, const char *data, int size)->int
	{
		try
		{
			std::string msg_data(data, size);

			aris::core::Calculator c;
			auto mat = c.calculateExpression(msg_data);

			double value[6]{ 2147483647, 2147483647, 2147483647, 2147483647, 2147483647, 2147483647 };
			s_vs(6, value, mat.data());
			s_nv(6, 1.0 / 2147483647.0, mat.data());

			// xy 客户和simtool不一样 //
			std::swap(mat.data()[0], mat.data()[1]);
			mat.data()[0] = -mat.data()[0];

			std::swap(mat.data()[3], mat.data()[4]);
			mat.data()[3] = -mat.data()[3];

			mat.data()[0] *= 0.035;
			mat.data()[1] *= 0.035;
			mat.data()[2] *= 0.037;
			mat.data()[3] *= 0.08;
			mat.data()[4] *= 0.08;
			mat.data()[5] *= 0.04;

			// 向上的轴加1.0，为默认位置 //
			mat.data()[2] += 0.513;
			mat.data()[1] -= 0.0103;

			auto cmd = "am --pe=" + mat.toString();

			static int i = 0;
			if (++i % 100 == 0)
			{
				std::cout << cmd << std::endl;
			}


			cs.executeCmd(aris::core::Msg(cmd));
		}
		catch (std::runtime_error &e)
		{
			std::cout << e.what() << std::endl;
		}

		return 0;
	});
	udp_socket.setOnReceivedConnection([](aris::core::Socket *sock, const char *ip, int port)->int
	{
		std::cout << "socket receive connection" << std::endl;
		LOG_INFO << "socket receive connection:\n"
			<< std::setw(aris::core::LOG_SPACE_WIDTH) << "|" << "  ip:" << ip << "\n"
			<< std::setw(aris::core::LOG_SPACE_WIDTH) << "|" << "port:" << port << std::endl;
		return 0;
	});
	udp_socket.setOnLoseConnection([](aris::core::Socket *socket)
	{
		std::cout << "socket lose connection" << std::endl;
		LOG_INFO << "socket lose connection" << std::endl;
		for (;;)
		{
			try
			{
				socket->startServer("5866");
				break;
			}
			catch (std::runtime_error &e)
			{
				std::cout << e.what() << std::endl << "will try to restart server socket in 1s" << std::endl;
				LOG_ERROR << e.what() << std::endl << "will try to restart server socket in 1s" << std::endl;
				std::this_thread::sleep_for(std::chrono::seconds(1));
			}
		}
		std::cout << "socket restart successful" << std::endl;
		LOG_INFO << "socket restart successful" << std::endl;

		return 0;
	});
	udp_socket.startServer();
	*/

	// 接收命令 //
	for (std::string command_in; std::getline(std::cin, command_in);)
	{
		try
		{
			auto target = cs.executeCmd(aris::core::Msg(command_in));
		}
		catch (std::exception &e)
		{
			std::cout << e.what() << std::endl;
			LOG_ERROR << e.what() << std::endl;
		}
	}
	return 0;
}
