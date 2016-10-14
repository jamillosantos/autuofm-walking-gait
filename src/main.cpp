#include <iostream>

#include <boost/program_options.hpp>
#include "engine/robot.h"

int main(int argc, const char **argv)
{
	using namespace mote;
	namespace po = boost::program_options;

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
	mote::walking::Configuration configuration;
	configuration.loadFromFile(configurationFile);
	std::cerr << "-- Robot: " << configuration.robot << std::endl;
	std::cerr << std::endl;

	mote::walking::sensors::IMU imu;

	mote::walking::Robot robot(configuration, imu);
	robot.run();

	return 0;
}
