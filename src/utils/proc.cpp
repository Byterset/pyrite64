/**
* @copyright 2025 - Max Bebök
* @license MIT
*/
#include "proc.h"

#include <fstream>
#include <memory>

#include "logger.h"

namespace
{
  constexpr uint32_t BUFF_SIZE = 128;
}

std::string Utils::Proc::runSync(const std::string &cmd)
{
  std::shared_ptr<FILE> pipe(popen(cmd.c_str(), "r"), pclose);
  if(!pipe)return "";

  char buffer[BUFF_SIZE];
  std::string result{};

  while(!feof(pipe.get()))
  {
    if(fgets(buffer, BUFF_SIZE, pipe.get()) != nullptr) {
      result += buffer;
    }
  }
  return result;
}

bool Utils::Proc::runSyncLogged(const std::string&cmd) {
  FILE* pipe = popen(cmd.c_str(), "r");
  if(!pipe)return "";

  char buffer[BUFF_SIZE];
  while(!feof(pipe))
  {
    if(fgets(buffer, BUFF_SIZE, pipe) != nullptr) {
      Logger::logRaw(buffer);
    }
  }
  return pclose(pipe) == 0;
}
