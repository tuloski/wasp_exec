// This may look like C code, but it is really -*- C++ -*-

#ifndef _EXECUTORTAKEOFF_H
#define _EXECUTORTAKEOFF_H

#include "executor.h"
#include "mms_msgs/Ack_mission.h"
#include "lrs_msgs_tst/ConfirmReq.h"
#include "mms_msgs/Cmd.h"

#include "ros/ros.h"

namespace Exec {

  class TakeOff : public virtual Executor {
  private:
	double height;
	int16_t frame;
	bool mission_succesfull;			//true if the command is accomplished
	int16_t qqseq_;                   //internal seq to save the sequence of the saved command
	int max_time;                //max time for execution (ms)

  public:
	TakeOff (std::string ns, int id);
	virtual ~TakeOff () {};

	virtual bool prepare ();
	virtual void start ();
	void callbackAckMission(const mms_msgs::Ack_mission::ConstPtr& msg);
	//virtual bool abort ();
	
  };

};

#endif
