/**
 * @file pub_kp3d_rtsp.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-12-12
 * 
 * @copyright Copyright (c) 2021
 * 
 * USAGE:
 * ./pub_kp3d_rtsp -fconf <conf-fp> 
 *                 [-h] [--help]
 * 
 * EXAMPLE:
 * ./pub_kp3d_rtsp -fconf /mnt/nas/conf.ini 
 */


#include "maeve/core.hpp"
#include "mqtt_maeve.hpp"

#include "maeve/parser/arguments.hpp"

void print_help()
{
    std::cout << "USAGE: "                                     << std::endl
        << " ./pub_kp3d_rtsp -fconf <conf-fp> "                << std::endl
        << "                 [-h] [--help]"                    << std::endl;
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
    args.print({"-fconf"});

    std::string conf_fp(args["-fconf"].as<std::string>());
    
    maeve::Camera device;
    maeve::DNN model(maeve::TRTPoseModelKind::DENSENET);

    maeve::Maeve platform(device, model);
    maeve::MqttMaeve interface(
        platform, conf_fp, "config-out.ini");
    interface.run_kp3d_rtsp();
}