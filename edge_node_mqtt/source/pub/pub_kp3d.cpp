/**
 * @file pub_kp3d.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-12-12
 * 
 * @copyright Copyright (c) 2021
 * 
 * USAGE:
 * ./pub_kp3d -fconf <conf-fp> 
 *            [-fcamera <camera-stream-fp] 
 *            [-tframe <timestamp-ns-to-reach>] 
 *            [-tstart <time-string-until-start>]
 *            [-h] [--help]
 */


#include "maeve/core.hpp"
#include "mqtt_maeve.hpp"

#include "maeve/parser/arguments.hpp"

void print_help()
{
    std::cout << "USAGE: "                                     << std::endl
        << " ./pub_kp3d -fconf <conf-fp> "                     << std::endl
        << "            [-fcamera <camera-stream-fp] "         << std::endl
        << "            [-tframe <timestamp-ns-to-reach>] "    << std::endl
        << "            [-tstart <time-string-until-start>]"   << std::endl
        << "            [-h] [--help]"                         << std::endl;
}


int main(int argc, char* argv[])
{
    maeve::parser::Arguments args(argc, argv);
    if (!args["-h"].empty() || !args["--help"].empty())
    {
        print_help();
        return 0;
    }
    if (args["-fconf"].empty())
    {
        std::cout << "Error: missing file path of configuration file." 
            << std::endl;
        print_help();
        return 1;
    }
    std::cout << "Application arguments: " << std::endl;
    args.print({"-fconf", "-fcamera", "-tframe", "-tstart"});

    std::string conf_fp(args["-fconf"].as<std::string>());
    

    maeve::CameraSetting device_setting;
    maeve::Camera device;
    if (!args["-fcamera"].empty())
    {
        device = maeve::Camera(
            args["-fcamera"].as<std::string>(),
            device_setting);
    }
    else 
    {
        device = maeve::Camera({
            maeve::RgbSensor(1242, 2208, 15), 
            maeve::DepthSensor(1242, 2208, 15)},
            device_setting);
    }
    maeve::DNN model(maeve::TRTPoseModelKind::DENSENET);

    maeve::Maeve platform(device, model);
    maeve::MqttMaeve interface(
        platform, conf_fp, "config-out.ini", 
        args["-tframe"].empty() ? -1 : args["-tframe"].as<std::int64_t>(), 
        args["-tstart"].as<std::string>());
    interface.run_kp3d();
}