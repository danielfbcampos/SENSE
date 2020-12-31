#ifndef CONSOLELOG_H
#define CONSOLELOG_H

#include <iostream>
#include <cstdio>
#include <cstdarg>


namespace CLog {
  const std::string BLACK   = "\033[1;30m";
  const std::string RED     = "\033[1;31m";
  const std::string GREEN   = "\033[1;32m";
  const std::string YELLOW  = "\033[1;33m";
  const std::string BLUE    = "\033[1;34m";
  const std::string MAGENTA = "\033[1;35m";
  const std::string CYAN    = "\033[1;36m";
  const std::string WHITE   = "\033[1;37m";
  const std::string CLEAR   = "\033[0m";

  class CLog
  {
    public:
      enum { Timing=0, All, Debug, Info, Warning, Error, Fatal, None};
      static void Write(int nLevel, const char *szFormat, ...);
      static void SetLevel(int nLevel);

    protected:
      static void CheckInit();
      static void Init();

    private:
      CLog();
      static bool m_bInitialised;
      static int  m_nLevel;
  };

  bool CLog::m_bInitialised;
  int  CLog::m_nLevel;
}
#endif


