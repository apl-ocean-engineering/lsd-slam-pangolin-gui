
#pragma once
#include <string>
#include <vector>
#include <fstream>

#define BOOST_FILESYSTEM_NO_DEPRECATED
#include <boost/filesystem/path.hpp>
namespace fs = boost::filesystem;



std::string &ltrim(std::string &s);
std::string &rtrim(std::string &s);
std::string &trim(std::string &s);

int getdir (fs::path dir, std::vector<fs::path> &files);

int getFile (std::string source, std::vector<std::string> &files);
