#include "console_log.h"

namespace CLog {
  void CLog::Write(int nLevel, const char *szFormat, ...)
  {
    CheckInit();

    if (m_nLevel == Timing)
    {
      if(nLevel == m_nLevel)
      {
        va_list args;
        va_start(args, szFormat);
        vprintf(szFormat, args);
        va_end(args);
      }
      return;
    }

    if (nLevel >= m_nLevel)
    {
      va_list args;
      va_start(args, szFormat);
      vprintf(szFormat, args);
      va_end(args);
    }
  }
  void CLog::SetLevel(int nLevel)
  {
    m_nLevel = nLevel;
    m_bInitialised = true;
  }
  void CLog::CheckInit()
  {
    if (!m_bInitialised)
    {
      Init();
    }
  }
  void CLog::Init()
  {
    int nDfltLevel(CLog::All);
    // Retrieve your level from an environment variable,
    // registry entry or wherecer
    SetLevel(nDfltLevel);
  }
}
