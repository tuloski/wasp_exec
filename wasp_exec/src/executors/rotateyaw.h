// This may look like C code, but it is really -*- C++ -*-

#ifndef _EXECUTORROTATEYAW_H
#define _EXECUTORROTATEYAW_H

#include "executor.h"
#include "mms_msgs/Ack_mission.h"
#include "guidance_node_amsl/Reference.h"

#include "ros/ros.h"

namespace Exec {

  class RotateYaw : public virtual Executor {
  private:
	double yaw;
	double speed;
	bool mission_succesfull;	//true if the command is accomplished
	int16_t qqseq_;                   //internal seq to save the sequence of the saved command
	int max_time;                //max time for execution (ms)

  public:
    RotateYaw (std::string ns, int id);
    virtual ~RotateYaw () {};

    bool prepare ();
    void start ();
    bool abort ();
    void callbackAckMission(const mms_msgs::Ack_mission::ConstPtr& msg);
	
  };

};

#endif
