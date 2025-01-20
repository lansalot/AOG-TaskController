#include "isobus/isobus/can_stack_logger.hpp"

#include <iostream>

// A log sink for the CAN stack
class CustomLogger : public isobus::CANStackLogger
{
public:
	/// @brief Destructor for the custom logger.
	virtual ~CustomLogger() = default;

	void sink_CAN_stack_log(CANStackLogger::LoggingLevel level, const std::string &text) override
	{
		switch (level)
		{
			case LoggingLevel::Debug:
			{
				std::cout << "[Debug]";
			}
			break;

			case LoggingLevel::Info:
			{
				std::cout << "[Info]";
			}
			break;

			case LoggingLevel::Warning:
			{
				std::cout << "[Warn]";
			}
			break;

			case LoggingLevel::Error:
			{
				std::cout << "[Error]";
			}
			break;

			case LoggingLevel::Critical:
			{
				std::cout << "[Critical]";
			}
			break;
		}
		std::cout << text << std::endl; // Write the text to stdout
	}
};

static CustomLogger logger;
