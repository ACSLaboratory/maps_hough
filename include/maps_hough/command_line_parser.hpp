////////////////////////////////////////////////////////////////////////////////
// My naming conventions for the project
// Put header files separated different category and in alphabetical order
// unless there some compatibility issue between the header files
// macros and const : ALL_IN_CAPS
// namespace : NS_prefix_small_letter_with_underscore
// class and structs : First_letter_caps_with_underscore
// functions and variables: all_small_letters
/////////////////////////////////////////////////////////////////////////////////

#ifndef MAPS_HOUGH_COMMAND_LINE_PARSER_HPP
#define MAPS_HOUGH_COMMAND_LINE_PARSER_HPP

#include <iostream>
#include <string>


class CommandLineParser
{
private:
  int argc;
  char **argv;

public:
  // Constructor
  CommandLineParser(int _argc, char **_argv);

  // Modules
  bool operator[] (std::string param);
  std::string operator() (std::string param, std::string def_value="-1");
};

#endif  // MAPS_HOUGH_COMMAND_LINE_PARSER_HPP
