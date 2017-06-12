/**
 * @file Platform/Linux/File.cpp
 */

#include "Platform/File.h"
#include "Platform/BHAssert.h"

#include <cstring>
#include <unistd.h>
#include <sys/stat.h>


#ifndef TARGET_TOOL
#include "Tools/Global.h"
//#include "Tools/Settings.h"
#endif

#include <cstdarg>

File::File(const std::string& name, const char* mode, bool tryAlternatives)
{
  fullName = name;
  std::list<std::string> names = getFullNames(name);
  if(tryAlternatives)
  {
    for(auto& path : names)
    {
      stream = fopen(path.c_str(), mode);
      if(stream)
      {
        fullName = path;
        break;
      }
    }
  }
  else
  {
    stream = fopen(names.back().c_str(), mode);
    if(stream)
      fullName = names.back();
  }
}

File::~File()
{
  if(stream != 0)
    fclose((FILE*)stream);
}

void File::read(void* p, size_t size)
{
  VERIFY(!eof());
  VERIFY(fread(p, 1, size, (FILE*)stream) > 0);
}

char* File::readLine(char* p, size_t size)
{
  VERIFY(!eof());
  return fgets(p, static_cast<int>(size), (FILE*)stream);
}

void File::write(const void* p, size_t size)
{
  //if opening failed, stream will be 0 and fwrite would crash
  ASSERT(stream != 0);
  VERIFY(fwrite(p, 1, size, (FILE*)stream) == size);
}

void File::printf(const char* format, ...)
{
  va_list args;
  va_start(args, format);
  vfprintf((FILE*)stream, format, args);
  va_end(args);
}

bool File::eof()
{
  //never use feof(stream), because it informs you only after reading
  //too far and is never reset, e.g. if the stream grows afterwards,
  //our implementation can handle both correctly:
  if(!stream)
    return false;
  else
  {
    int c = fgetc((FILE*)stream);
    if(c == EOF)
      return true;
    else
    {
      VERIFY(ungetc(c, (FILE*)stream) != EOF);
      return false;
    }
  }
}

size_t File::getSize()
{
  if(!stream)
    return 0;
  else
  {
    const long currentPos = ftell((FILE*)stream);
    ASSERT(currentPos >= 0);
    VERIFY(fseek((FILE*)stream, 0, SEEK_END) == 0);
    const long size = ftell((FILE*)stream);
    VERIFY(fseek((FILE*)stream, currentPos, SEEK_SET) == 0);
    return static_cast<size_t>(size);
  }
}

bool File::isAbsolute(const char* path)
{
  return (path[0] && path[1] == ':') || path[0] == '/' || path[0] == '\\';
}

std::list<std::string> File::getConfigDirs()
{
  std::list<std::string> dirs;
  const std::string configDir = std::string(getBHDir()) + "/Config/";
  /*
#ifndef TARGET_TOOL
  if(Global::settingsExist())
  {
    dirs.push_back(configDir + "Robots/" + Global::getSettings().headName + "/Head/");
    dirs.push_back(configDir + "Robots/" + Global::getSettings().bodyName + "/Body/");
    dirs.push_back(configDir + "Robots/" + Global::getSettings().headName + "/" + Global::getSettings().bodyName + "/");
    dirs.push_back(configDir + "Robots/" + Global::getSettings().headName + "/");
    dirs.push_back(configDir + "Robots/Default/");
    dirs.push_back(configDir + "Locations/" + Global::getSettings().location + "/");
    if(Global::getSettings().location != "Default")
      dirs.push_back(configDir + "Locations/Default/");
  }
#endif
*/
  dirs.push_back(configDir);
  return dirs;
}

std::list<std::string> File::getFullNames(const std::string& rawName)
{
  std::string name = rawName;
  for(size_t i = name.length(); i-- > 0;)
    if(name[i] == '\\')
      name[i] = '/';

  std::list<std::string> names;
  /*
  if((name[0] != '.' || (name.size() >= 2 && name[1] == '.')) && !isAbsolute(name.c_str())) // given path is relative to getBHDir()
  {
    std::list<std::string> dirs = getConfigDirs();
    for(std::string& dir : dirs)
      names.push_back(dir + name);
  }
  else
    names.push_back(name);
    */
  return names;
}

const char* File::getBHDir()
{
  static char dir[FILENAME_MAX] = {0};
  if(!dir[0])
  {
    VERIFY(getcwd(dir, sizeof(dir) - 7) != 0);
    char* end = dir + strlen(dir) - 1;
    struct stat buff;
    while(true)
    {
      if(*end == '/' || end == dir - 1)
      {
        strcpy(end + 1, "Config");
        if(stat(dir, &buff) == 0)
          if(S_ISDIR(buff.st_mode))
          {
            end[end > dir ? 0 : 1] = '\0';
            return dir;
          }
      }
      if(end < dir)
        break;
      end--;
    }
    strcpy(dir, ".");
  }
  return dir;
}
