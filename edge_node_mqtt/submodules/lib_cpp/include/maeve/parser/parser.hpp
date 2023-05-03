/**
 * @file parser.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-12-22
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef PARSER_PARSER_HPP
#define PARSER_PARSER_HPP

#include "type_checker.hpp"

namespace maeve {

namespace parser {

/**
 * @brief Generic class for parser the shares the TypeChecker entity.
 */
class Parser 
{
public:
    /**
     * @brief Construct a new Parser object.
     */
    Parser() : _tc() {}

    /**
     * @brief Destroy the Parser object.
     */
    virtual ~Parser() {};

protected:
    TypeChecker _tc; ///< TypeChecker entity to perform parsing and convertion.
};

} // namespace parser

} // namespace maeve

#endif // PARSER_PARSER_HPP
