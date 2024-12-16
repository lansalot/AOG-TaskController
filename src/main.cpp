/**
 * @author Daan Steenbergen
 * @brief An ISOBUS Task Controller for AgOpenGPS
 * @version 0.1
 * @date 2024-12-16
 *
 * @copyright 2024 Daan Steenbergen
 *
 */

#include <boost/asio.hpp>
#include <atomic>
#include <csignal>
#include <iostream>
#include <memory>
#include <future>

#include "isobus/hardware_integration/can_hardware_interface.hpp"
#include "isobus/hardware_integration/twai_plugin.hpp"
#include "isobus/isobus/can_general_parameter_group_numbers.hpp"
#include "isobus/isobus/can_network_manager.hpp"
#include "isobus/isobus/can_partnered_control_function.hpp"
#include "isobus/isobus/can_stack_logger.hpp"
#include "isobus/isobus/isobus_standard_data_description_indices.hpp"
#include "isobus/isobus/isobus_task_controller_server.hpp"
#include "isobus/hardware_integration/available_can_drivers.hpp"
#include "isobus/isobus/isobus_preferred_addresses.hpp"

#include "console_logger.cpp"
#include <bitset>

using boost::asio::ip::udp;

static constexpr std::uint8_t NUMBER_OF_SECTIONS = 6;
static std::shared_ptr<isobus::ControlFunction> clientTC = nullptr;

static std::atomic_bool running = {true};
void signal_handler(int)
{
    running = false;
}

// Create the task controller server object, this will handle all the ISOBUS communication for us
class MyTCServer : public isobus::TaskControllerServer
{
public:
    MyTCServer(std::shared_ptr<isobus::InternalControlFunction> internalControlFunction,
               std::uint8_t numberBoomsSupported,
               std::uint8_t numberSectionsSupported,
               std::uint8_t numberChannelsSupportedForPositionBasedControl,
               const isobus::TaskControllerOptions &options) : TaskControllerServer(internalControlFunction,
                                                                                    numberBoomsSupported,
                                                                                    numberSectionsSupported,
                                                                                    numberChannelsSupportedForPositionBasedControl,
                                                                                    options)
    {
    }

    bool activate_object_pool(std::shared_ptr<isobus::ControlFunction> client, ObjectPoolActivationError &, ObjectPoolErrorCodes &, std::uint16_t &, std::uint16_t &) override
    {
        clientTC = client;
        return true;
    }

    bool change_designator(std::shared_ptr<isobus::ControlFunction>, std::uint16_t, const std::vector<std::uint8_t> &) override
    {
        return true;
    }

    bool deactivate_object_pool(std::shared_ptr<isobus::ControlFunction>) override
    {
        return true;
    }

    bool delete_device_descriptor_object_pool(std::shared_ptr<isobus::ControlFunction>, ObjectPoolDeletionErrors &) override
    {
        return true;
    }

    bool get_is_stored_device_descriptor_object_pool_by_structure_label(std::shared_ptr<isobus::ControlFunction>, const std::vector<std::uint8_t> &, const std::vector<std::uint8_t> &) override
    {
        return false;
    }

    bool get_is_stored_device_descriptor_object_pool_by_localization_label(std::shared_ptr<isobus::ControlFunction>, const std::array<std::uint8_t, 7> &) override
    {
        return false;
    }

    bool get_is_enough_memory_available(std::uint32_t) override
    {
        return true;
    }

    void identify_task_controller(std::uint8_t) override
    {
        // When this is called, the TC is supposed to display its TC number for 3 seconds if possible (which is passed into this function).
        // Your TC's number is your function code + 1, in the range of 1-32.
    }

    void on_client_timeout(std::shared_ptr<isobus::ControlFunction>) override
    {
        // You can use this function to handle when a client times out (6 Seconds)
    }

    void on_process_data_acknowledge(std::shared_ptr<isobus::ControlFunction>, std::uint16_t, std::uint16_t, std::uint8_t, ProcessDataCommands) override
    {
        // This callback lets you know when a client sends a process data acknowledge (PDACK) message to you
    }

    bool on_value_command(std::shared_ptr<isobus::ControlFunction>, std::uint16_t, std::uint16_t, std::int32_t, std::uint8_t &) override
    {
        return true;
    }

    bool store_device_descriptor_object_pool(std::shared_ptr<isobus::ControlFunction>, const std::vector<std::uint8_t> &, bool) override
    {
        return true;
    }
};

int main()
{
    std::signal(SIGINT, signal_handler);

    auto canDriver = std::make_shared<isobus::NTCANPlugin>(42);
    if (nullptr == canDriver)
    {
        std::cout << "Unable to find a CAN driver. Please make sure you have one of the above drivers installed with the library." << std::endl;
        std::cout << "If you want to use a different driver, please add it to the list above." << std::endl;
        return -1;
    }
    isobus::CANStackLogger::set_can_stack_logger_sink(&logger);
    isobus::CANStackLogger::set_log_level(isobus::CANStackLogger::LoggingLevel::Debug); // Change this to Debug to see more information
    isobus::CANHardwareInterface::set_number_of_can_channels(1);
    // isobus::CANHardwareInterface::set_periodic_update_interval(10);
    isobus::CANHardwareInterface::assign_can_channel_frame_handler(0, canDriver);

    if ((!isobus::CANHardwareInterface::start()) || (!canDriver->get_is_valid()))
    {
        std::cout << "Failed to start CAN hardware interface." << std::endl;
        return -1;
    }

    isobus::NAME ourNAME(0);

    //! Make sure you change these for your device!!!!
    ourNAME.set_arbitrary_address_capable(true);
    ourNAME.set_industry_group(2);
    ourNAME.set_device_class(0);
    ourNAME.set_function_code(static_cast<std::uint8_t>(isobus::NAME::Function::TaskController));
    ourNAME.set_identity_number(20);
    ourNAME.set_ecu_instance(0);
    ourNAME.set_function_instance(0); // TC #1. If you want to change the TC number, change this.
    ourNAME.set_device_class_instance(0);
    ourNAME.set_manufacturer_code(1407);

    auto serverCF = isobus::CANNetworkManager::CANNetwork.create_internal_control_function(ourNAME, 0, isobus::preferred_addresses::IndustryGroup2::TaskController_MappingComputer); // The preferred address for a TC is defined in ISO 11783
    auto addressClaimedFuture = std::async(std::launch::async, [&serverCF]()
                                           {
			while (!serverCF->get_address_valid())
				std::this_thread::sleep_for(std::chrono::milliseconds(100)); });

    // If this fails, probably the update thread is not started
    addressClaimedFuture.wait_for(std::chrono::seconds(5));
    if (!serverCF->get_address_valid())
    {
        std::cout << "Failed to claim address for TC server. The control function might be invalid." << std::endl;
        return -1;
    }

    MyTCServer server(serverCF,
                      4,   // Booms
                      255, // Sections
                      16,  // Channels
                      isobus::TaskControllerOptions()
                          .with_implement_section_control());
    auto &languageInterface = server.get_language_command_interface();
    languageInterface.set_language_code("en"); // This is the default, but you can change it if you want
    languageInterface.set_country_code("NL");  // This is the default, but you can change it if you want
    server.initialize();

    std::cout << "Task controller server started." << std::endl;

    std::uint32_t lastTime = 0;
    std::array<bool, NUMBER_OF_SECTIONS> sectionWorkStates{false, false, false, false, false, false};
    bool requiresUpdate = false;

    // Set up the UDP server
    boost::asio::io_service io_service;
    udp::socket udpConnection(io_service);
    udpConnection.open(udp::v4());
    udpConnection.set_option(boost::asio::socket_base::broadcast(true));
    udpConnection.non_blocking(true);
    udp::endpoint local_endpoint(boost::asio::ip::address_v4::any(), 8888);
    udpConnection.bind(local_endpoint);

    std::cout << "Local endpoint address: " << local_endpoint.address().to_string() << std::endl;
    std::cout << "Broadcast address: " << boost::asio::ip::address_v4::broadcast().to_string() << std::endl;

    uint8_t rxBuffer[512];
    size_t rxIndex = 0;

    std::uint8_t sectionIndex = 0;
    while (running)
    {
        std::uint32_t currentTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

        if (currentTime - lastTime >= 200)
        {
            lastTime = currentTime;

            uint8_t AOG[] = {0x80, 0x81, 0x7B, 0xED, 8, 1, 2, 3, 4, 0, 0, 0, 0, 0xCC};

            // add the checksum
            int16_t CK_A = 0;
            for (uint8_t i = 2; i < sizeof(AOG) - 1; i++)
            {
                CK_A = (CK_A + AOG[i]);
            }
            AOG[sizeof(AOG) - 1] = CK_A;

            udp::endpoint broadcast_endpoint(boost::asio::ip::address_v4::broadcast(), 8889);
            udpConnection.send_to(boost::asio::buffer(AOG, sizeof(AOG)), broadcast_endpoint);
        }

        // Peek to see if we have any data
        boost::system::error_code ec;
        udp::endpoint sender_endpoint;
        size_t bytesReceived = udpConnection.receive_from(boost::asio::buffer(rxBuffer + rxIndex, sizeof(rxBuffer) - rxIndex), sender_endpoint, 0, ec);

        if (ec == boost::asio::error::would_block)
        {
            // No data available
        }
        else if (!ec)
        {
            rxIndex += bytesReceived;
            std::uint8_t index = 0;

            while (rxIndex >= 8)
            {
                if (rxBuffer[index++] == 0x80 && rxBuffer[index++] == 0x81)
                {
                    std::uint8_t src = rxBuffer[index++];
                    std::uint8_t pgn = rxBuffer[index++];
                    std::uint8_t len = rxBuffer[index++];

                    if (pgn == 0xEF) // 239 - EF Machine Data
                    {
                        // We only care about the sections
                        for (std::uint8_t i = 0; i < NUMBER_OF_SECTIONS; i++)
                        {
                            bool newState = rxBuffer[index + 6] & (1 << i);
                            if (newState != sectionWorkStates[i])
                            {
                                printf("Section %d is set to  '%s' by AOG\n", i + 1, newState ? "ON" : "OFF");
                                sectionWorkStates[i] = newState;
                                requiresUpdate = true;
                            }
                        }
                        index += len + 1;
                    }
                    else
                    {
                        // Unknown PGN, reset buffer
                        // std::cout << "Unknown PGN: " << std::hex << static_cast<int>(pgn) << std::endl;
                        rxIndex = 0;
                    }
                }
                else
                {
                    // Unknown start of message, reset buffer
                    std::cout << "Unknown start of message: " << std::hex << static_cast<int>(rxBuffer[index - 2]) << " " << static_cast<int>(rxBuffer[index - 1]) << std::endl;
                    rxIndex = 0;
                }

                // Move any remaining data to the front of the buffer
                if (index < rxIndex)
                {
                    std::memmove(rxBuffer, rxBuffer + index, rxIndex - index);
                    rxIndex -= index;
                }
                else
                {
                    rxIndex = 0;
                }
            }
        }
        else
        {
            // Handle other errors
            isobus::CANStackLogger::error("UDP receive error: " + ec.message());
        }

        if (requiresUpdate)
        {
            // Send the new section work states to the client
            std::uint32_t value = 0;
            for (std::uint8_t i = 0; i < 6; i++)
            {
                value |= (sectionWorkStates[i] ? static_cast<std::uint32_t>(0x01) : static_cast<std::uint32_t>(0x00)) << (2 * i);
            }
            server.send_set_value_and_acknowledge(clientTC, static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SetpointCondensedWorkState1_16), 2, value);
            requiresUpdate = false;
        }

        server.update();

        // Update again in a little bit
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    server.terminate();
    isobus::CANHardwareInterface::stop();
}