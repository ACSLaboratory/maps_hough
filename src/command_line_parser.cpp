//
// Created by Ragesh on 5/5/18.
//

////////////////////////////////////////////////////////////////////////////////
// My naming conventions for the project
// Put header files separated different category and in alphabetical order
// unless there some compatibility issue between the header files
// macros and const : ALL_IN_CAPS
// namespace : NS_prefix_small_letter_with_underscore
// class and structs : First_letter_caps_with_underscore
// functions and variables: all_small_letters
/////////////////////////////////////////////////////////////////////////////////

#include "maps_hough/command_line_parser.hpp"


CommandLineParser::CommandLineParser(int _argc, char **_argv): argc(_argc), argv(_argv){}

bool CommandLineParser::operator[] (std::string param)
{
  int idx = -1;
  for (int i = 0; i < argc && idx == -1; i++)
  {
    if (std::string(argv[i]) == param)
    {
      idx = i;
    }
  }

  return (idx != -1);
}

std::string CommandLineParser::operator() (std::string param, std::string def_value)
{
  int idx = -1;
  for (int i = 0; i < argc && idx == -1; i++)
  {
    if (std::string(argv[i]) == param)
    {
      idx = i;
    }
  }

  if (idx == -1)
  {
    return def_value;
  }
  else
  {
    return argv[idx + 1];
  }
}