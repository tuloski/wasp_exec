// This may look like C code, but it is really -*- C++ -*-

#ifndef _EXECUTORLAND_H
#define _EXECUTORLAND_H

#include "executor.h"
#include "mms_msgs/Ack_mission.h"
#include "lrs_msgs_tst/ConfirmReq.h"
#include "mms_msgs/Cmd.h"

#include "ros/ros.h"

namespace Exec {

  class Land : public virtual Executor {
  private:
	int16_t frame;
	bool mission_succesfull;			//true if the command is accomplished
	int16_t qqseq_;                   //internal seq to save the sequence of the saved command
	int max_time;                //max time for execution (ms)

  public:
	Land (std::string ns, int id);
	virtual ~Land () {};

	bool prepare ();
	void start ();
	void callbackAckMission(const mms_msgs::Ack_mission::ConstPtr& msg);
	virtual bool abort ();

  };

};

#endif
