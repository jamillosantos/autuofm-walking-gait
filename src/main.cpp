#include <iostream>

#include <boost/program_options.hpp>
#include <csignal>

#include "motors/simumotorupdater.h"
#include "engine/robot.h"

int main(int argc, const char **argv)
{
	namespace po = boost::program_options;
	using namespace mote::walking;

	boost::filesystem::path configurationDirectory(CONFIGURATION_FOLDER);
	std::string configurationFile((configurationDirectory / "arash.json").string());

	// Declare the supported options.
	po::options_description desc("Allowed options");
	desc.add_options()
		("help,h", "Display this help")
		("config,c", po::value<std::string>(&configurationFile), "set the configuration file");

	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);
	po::notify(vm);

	if (vm.count("help"))
	{
		std::cout << desc << "\n";
		return 0;
	}

	std::cerr << "Configuration file: " << configurationFile << std::endl;
	Configuration configuration;
	configuration.loadFromFile(configurationFile);
	std::cerr << "-- Robot: " << configuration.robot << std::endl;
	std::cerr << std::endl;

	sensors::IMU imu;
	HumanoidPart robotPart;

	Robot robot(configuration, imu, robotPart);
	std::unique_ptr<networking::UDPClient> udpClient;
	std::unique_ptr<motors::SimuMotorUpdater> updater;
	if (configuration.simu)
	{
		udpClient.reset(new networking::UDPClient(configuration.simu->address, configuration.simu->port));
		updater.reset(new motors::SimuMotorUpdater(*udpClient, robotPart));
		updater->init();
		updater->start();
	}
	std::signal(SIGTERM, [](int signum) {
		std::cout << "Stopping ..." << std::endl;
	});
	robot.run();

	return 0;
}
