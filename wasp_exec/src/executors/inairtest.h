// This may look like C code, but it is really -*- C++ -*-

#ifndef _EXECUTORINAIRTEST_H
#define _EXECUTORINAIRTEST_H

#include "executor.h"
#include "lrs_msgs_tst/TSTNodeInfo.h"
#include <tf/transform_listener.h>
#include "mms_msgs/MMS_status.h"// output


namespace Exec {

  class InAirTest : public virtual Executor {
  private:
    std::map<std::string, std::string> params;
    tf::TransformListener listener;
    mms_msgs::MMS_status _mms_status;
    bool received_mms_state;
    int counter_wait_mms_state;

  public:
    InAirTest (std::string ns, int id);
    virtual ~InAirTest () {};

    bool prepare ();
    void start ();
    void callbackState(const mms_msgs::MMS_status::ConstPtr& msg);
  };

};

#endif
