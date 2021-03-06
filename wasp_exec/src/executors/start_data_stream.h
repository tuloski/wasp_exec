// This may look like C code, but it is really -*- C++ -*-

#ifndef _STOP_DATA_STREAM_H
#define _STOP_DATA_STREAM_H

#include "executor.h"

#include <string>

namespace Exec {

  class StartDataStream : public virtual Executor {
  private:

  public:
    StartDataStream (std::string ns, int id);
    virtual ~StartDataStream () {};

    virtual bool check ();

    virtual bool prepare ();
    virtual void start ();
    virtual bool abort ();


  };
};
#endif