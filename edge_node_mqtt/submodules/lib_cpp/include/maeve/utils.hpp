/**
 * @file utils.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-12-14
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef UTILS_HPP
#define UTILS_HPP

#include <unistd.h>
#include <limits.h>
#include <string>
#include <fstream>
#include <sstream>
#include <ctime>
#include <locale>
#include <iomanip>


namespace maeve {

const std::string MACHINE_ID_FP("/etc/machine-id");
const std::string TIME_FORMAT("%d/%m/%Y %H:%M:%S");

class Utils {
public:
    static std::string get_host_name()
    {
        static char hostname[HOST_NAME_MAX + 1];
        gethostname(hostname, HOST_NAME_MAX + 1);
        return std::string(hostname);
    }

    static std::string get_machine_id()
    {
        std::ifstream file;
        file.open(MACHINE_ID_FP);
        if(!file.is_open() || !file.good()) 
        {
            return std::string();
        }
        std::string ret;
        file >> ret;
        return ret;
    }

    static std::time_t timestamp_conv(std::string time_str)
    {
        struct std::tm tm;
        tm.tm_isdst = -1;
        std::stringstream ss(time_str);
        ss >> std::get_time(&tm, TIME_FORMAT.c_str());
        std::time_t time = std::mktime(&tm);
        return time;
    }

    static std::string timestamp_conv(std::time_t time = std::time(0))
    {
        std::tm tm = *std::localtime(&time);
        std::stringstream ss;
        ss << std::put_time(&tm, TIME_FORMAT.c_str());
        return std::string(ss.str());
    }
};

} // namespace maeve

#endif // UTILS_HPP