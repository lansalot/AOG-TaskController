/**
 * @author Daan Steenbergen
 * @brief The main application class
 * @version 0.1
 * @date 2025-1-20
 *
 * @copyright 2025 Daan Steenbergen
 */
#pragma once

#include <boost/asio.hpp>
#include <memory>

#include "isobus/isobus/isobus_speed_distance_messages.hpp"

#include "settings.hpp"
#include "task_controller.hpp"
#include "udp_connections.hpp"

class Application
{
public:
	bool initialize();
	bool update();
	void stop();

private:
	std::shared_ptr<Settings> settings = std::make_shared<Settings>();
	boost::asio::io_context ioContext = boost::asio::io_context();
	std::shared_ptr<UdpConnections> udpConnections = std::make_shared<UdpConnections>(settings, ioContext);

	std::shared_ptr<MyTCServer> tcServer;
	std::shared_ptr<isobus::SpeedMessagesInterface> speedMessagesInterface;
};
