/**
 * @file arguments.hpp
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
#include <vector>
#include <iostream>


#ifndef PARSER_ARGUMENTS_HPP
#define PARSER_ARGUMENTS_HPP

namespace maeve {

namespace parser {

class ArgumentsField : public Parser {
public:
    ArgumentsField(std::string field = std::string())
        : _field{field}
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

    operator std::string() { return _field; }
    const std::string& field() { return _field; }

private:
    std::string _field;
};


class Arguments : public Parser {
public:
    Arguments(int argc, char const* const* argv)
        : Parser()
    {
        _args.resize(argc);
        for (std::size_t i = 0; i < argc; ++i)
        {
            _args[i] = std::string(argv[i]);
        }
    }

    ArgumentsField at(std::size_t field_id, std::size_t field_id_end = 0)
    {
        if (field_id < _args.size())
        {
            if (field_id_end > field_id && field_id_end < _args.size())
            {
                std::string field("");
                std::size_t i = field_id;
                for (; i < (field_id_end - 1); ++i)
                {
                    field += _args[i] + " ";
                }
                return ArgumentsField(field + _args[i]);
            }
            return ArgumentsField(_args[field_id]);
        }
        else 
        {
            return ArgumentsField();
        }
    }

    ArgumentsField operator[](std::size_t field_id)
    {
        return at(field_id);
    }

    ArgumentsField operator[](std::string field_id)
    {
        auto field_iter = std::find(_args.begin(), _args.end(), field_id);
        if (field_iter != _args.end())
        {
            auto iter = field_iter + 1;
            if (iter == _args.end()) return ArgumentsField(); 
            std::string field("");
            while (iter != _args.end() && (*iter)[0] != '-')
            {
                field += *iter++ + " ";
            }
            field.pop_back();
            return ArgumentsField(field);
        }
        else 
        {
            return ArgumentsField();
        }
    }

    void print(const std::vector<std::string> fields)
    {
        for (const auto &f: fields)
        {
            std::cout << f << ": " 
                << (operator[](f).empty() ? 
                    "!empty!" : operator[](f).as<std::string>())
                << std::endl; 
        }
    }

    operator std::vector<std::string>() { return _args; }
    const std::vector<std::string>& args() { return _args; }

private:
    std::vector<std::string> _args;
};

} // namespace parser

} // namespace maeve

#endif // PARSER_ARGUMENTS_HPP