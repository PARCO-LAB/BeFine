/**
 * @file type_checker.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-12-22
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef PARSER_TYPE_CHECKER_HPP
#define PARSER_TYPE_CHECKER_HPP

#include <string>
#include <sstream>
#include <vector>
#include <regex>

namespace maeve {

namespace parser {

/**
 * @brief Enumeration class with the list of parserizable types.
 */
enum class Type : int
{
    NONE    = -1, 
    AUTO    =  0,
    FLOAT   =  1,
    INT     =  2,
    BOOL    =  3,
    STRING  =  4,
    OBJECT  =  5,
};

/**
 * @brief Regex to detect float values in string.
 */
const std::regex _float_regex   { "^[-+]?[0-9]*\\.?[0-9]+([eE][-+]?[0-9]+)?$" };

/**
 * @brief Regex to detect int values in string.
 */
const std::regex _integer_regex { "^[-+]?(0|[1-9][0-9]*)$" };

/**
 * @brief Regex to detect bool values in string.
 */
const std::regex _boolean_regex { "^(true|false)$" };

/**
 * @brief Regex to detect strings
 */
const std::regex _string_regex  { "^\".*\"$" };

/**
 * @brief Convert a string in the templated type T and put the result in the 
 * pointer.
 * @tparam T  The type to convert to.
 * @param s   The string to convert. 
 * @param ptr The pointer to the memory in which put the result.
 * @return true  If convertion succeedes.
 * @return false If convertion fails.
 */
template<typename T>
bool convert(const std::string &s, T *ptr)
{
    std::stringstream ss{s};
    ss >> *ptr;
    return !ss.fail() && ss.eof();
}

/**
 * @brief Template specialization for string of convert<T>
 * @tparam std::string
 * @param s   The string to convert. 
 * @param ptr The pointer to the memory in which put the result.
 * @return true  If convertion succeedes.
 * @return false If convertion fails.
 */
template<> inline
bool convert<std::string>(const std::string &s, std::string *ptr)
{
    *ptr = s;
    return true;
}

/**
 * @brief Template specialization for boolean of convert<T>
 * @tparam bool
 * @param s   The string to convert. 
 * @param ptr The pointer to the memory in which put the result.
 * @return true  If convertion succeedes.
 * @return false If convertion fails.
 */
template<> inline
bool convert<bool>(const std::string &s, bool *ptr)
{
    if (s == "true" || s == "1")
    {
        *ptr = true;
    }
    else 
    {
        *ptr = false;
    }
    return true;
}

/**
 * @brief TypeChecker class that manages the types of parsed strings in input. 
 */
class TypeChecker
{
public:
    /**
     * @brief Construct a new TypeChecker object.
     */
    TypeChecker() {};

    /**
     * @brief Destroy the TypeChecker object.
     */
    virtual ~TypeChecker() {};
    
    /**
     * @brief Check if the input string is of type float. 
     * @param in     The input string.
     * @return true  It is of type float.
     * @return false It is not of type float.
     */
    static bool is_float(const std::string& in)  
        { return parse(in) == Type::FLOAT;  };

    /**
     * @brief Check if the input string is of type bool. 
     * @param in     The input string.
     * @return true  It is of type bool.
     * @return false It is not of type bool.
     */
    static bool is_bool(const std::string& in)   
        { return parse(in) == Type::BOOL;   };

    /**
     * @brief Check if the input string is of type int. 
     * @param in     The input string.
     * @return true  It is of type int.
     * @return false It is not of type int.
     */
    static bool is_int(const std::string& in)    
        { return parse(in) == Type::INT;    };
    
    /**
     * @brief Check if the input string is of type string. 
     * @param in     The input string.
     * @return true  It is of type string.
     * @return false It is not of type string.
     */
    static bool is_string(const std::string& in) 
        { return parse(in) == Type::STRING; };
    
    /**
     * @brief Operator overloading for string convertion (see convert). 
     * @tparam T 
     * @param s   The string to convert. 
     * @param ptr The pointer to the memory in which put the result.
     * @return true  If convertion succeedes.
     * @return false If convertion fails.
     */
    template<typename T> 
    bool operator()(const std::string &s, T *ptr) const 
    {
        return parse<T>(s, ptr);
    }

    /**
     * @brief Operator overloading that parse a string (see parse). 
     * @param field The string field.
     * @return Type The type of the parsed field.
     */
    Type operator()(const std::string &field) const 
    {
        return parse(field);
    }

    /**
     * @brief Operator () overloading that parse a vector of string. 
     * @param field The vector of string field.
     * @return Type The vector types of the parsed fields.
     */
    std::vector<Type> operator()(
        const std::vector<std::string> &fields) const 
    {
        return parse(fields);
    }

    /**
     * @brief Convert a string in the specified template type (see convert). 
     * @tparam T  The specified type. 
     * @param s   The string to convert. 
     * @param ptr The pointer to the memory in which put the result.
     * @return true  If convertion succeedes.
     * @return false If convertion fails.
     */
    template<typename T>
    static bool parse(const std::string &s, T *ptr)
    {
        return convert<T>(s, ptr);
    }
    
    /**
     * @brief Parse the type of the given string field.
     * @param field The string to parse.
     * @return Type The type of the converted string.
     */
    static Type parse(const std::string &field)
    {
        if (field.empty())
        {
            return Type::NONE;
        }

        if (std::regex_match(field, _string_regex))
        {
            return Type::STRING;
        }
        else if (std::regex_match(field, _boolean_regex))
        {
            return Type::BOOL;
        }
        else if (std::regex_match(field, _integer_regex))
        {
            return Type::INT;
        }
        else if (std::regex_match(field, _float_regex))
        {
            return Type::FLOAT;
        }
        else 
        {
            return Type::STRING;
        }
    }

    /**
     * @brief Parse a vector of string fields.
     * @param fields The string fields to parse.
     * @return std::vector<Type> The resulting types of string fields.
     */
    static std::vector<Type> parse(
        const std::vector<std::string> &fields)
    {
        std::vector<Type> ret;
        ret.resize(fields.size());
        for (std::size_t i = 0; i < fields.size(); ++i)
        {
            ret[i] = parse(fields[i]);
        }
        return ret;
    }
};

/**
 * @brief Operator overloading to cout the type.
 * @param os  Input stream.
 * @param obj Type to print.
 * @return std::ostream& Output stream.
 */
inline std::ostream& operator<<(std::ostream& os, const Type& obj)
{
   switch(obj)
   {
        case Type::NONE:   os << "NONE"; break;
        case Type::AUTO:   os << "AUTO"; break;
        case Type::FLOAT:  os << "FLOAT"; break; 
        case Type::INT:    os << "INT"; break;
        case Type::BOOL:   os << "BOOL"; break;
        case Type::STRING: os << "STRING"; break;
        case Type::OBJECT: os << "OBJECT"; break;
        default: break;
   }
   os << "(" << static_cast<std::underlying_type<Type>::type>(obj) << ")";
   return os;
}

/**
 * @brief Operator overloading to cout a vector of types.
 * @param os  Input stream.
 * @param obj Types to print.
 * @return std::ostream& Output stream.
 */
inline std::ostream& operator<<(std::ostream& os, const std::vector<Type>& obj)
{
    os << "{";
    for (auto &pt: obj)
    {
        os << pt << ",";
    }
    os << "}";
    return os;
}

/**
 * @brief Check if two vectors of types are equals.
 * @param lhs Vector of types first operand.
 * @param rhs Vector of types second operand.
 * @return true  Equals.
 * @return false Not equals.
 */
inline bool operator==(
    const std::vector<Type> &lhs, const std::vector<Type> &rhs)
{
    if (lhs.size() != rhs.size()) return false;
    for (std::size_t i = 0; i < lhs.size(); ++i)
    {
        if (lhs.at(i) != rhs.at(i))
        {
            return false;
        }
    }
    return true;
}

/**
 * @brief Check if two vectors of types are different.
 * @param lhs Vector of types first operand.
 * @param rhs Vector of types second operand.
 * @return true  Different.
 * @return false Equals.
 */
inline bool operator!=(
    const std::vector<Type> &lhs, const std::vector<Type> &rhs)
{
    return !operator==(lhs, rhs);
}

} // namespace parser

} // namespace maeve

#endif // PARSER_TYPE_CHECKER_HPP
