/**
 * @file config.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-12-14
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "parser.hpp"

#include <string>
#include <fstream>
#include <sstream>
#include <stdexcept>


#ifndef PARSER_CONFIG_HPP
#define PARSER_CONFIG_HPP

namespace maeve {

namespace parser {

class ConfigField : public Parser {
public:
    ConfigField(std::string field = std::string())
        : Parser()
        , _field{field}
    {

    }

    bool empty()
    {
        return _field.empty();
    }

    template<typename T>
    T as()
    {
        T ret; 
        _tc.template operator()<T>(_field, &ret);
        return ret;
    }

private:
    std::string _field;
};


class ConfigSection : public Parser {
public:
    ConfigSection(std::string section = std::string())
        : Parser()
        , _section{section}
    {

    }

    bool empty()
    {
        return _section.empty();
    }

    ConfigField operator[](std::string field_id)
    {
        auto field_idx = _section.find(field_id);
        if (field_idx == std::string::npos) return ConfigField();
        auto value_idx = _section.find('=', field_idx) + 1;
        auto value = _section.substr(value_idx, 
            _section.find('\n', value_idx) - value_idx);
        return ConfigField(value);
    }

private:
    std::string _section;
};


class Config : public Parser {
public:
    Config(std::string fp)
        : Parser()
        , _fp{fp}
    {

    }

    ConfigSection operator[](std::string section_id)
    {
        auto file = std::ifstream{_fp};
        if(!file.is_open() || !file.good()) 
        {
            return ConfigSection();
        }

        std::string section("");
        bool start_to_append = false;

        std::string line;
        while(std::getline(file, line))
        {
            if (!start_to_append 
                && line.find("[" + section_id + "]") != std::string::npos)
            {
                start_to_append = true;
                continue;
            }
            
            if (start_to_append)
            {
                if (line.find('[') != std::string::npos) break;   
                section += line + '\n';
            }
        }

        if (!start_to_append)
        {
            return ConfigSection();
        }

        return ConfigSection(section);
    }

private:
    std::string _fp;
};

} // namespace parser

} // namespace maeve

#endif // PARSER_CONFIG_HPP