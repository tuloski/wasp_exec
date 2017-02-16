// This may look like C code, but it is really -*- C++ -*-

#ifndef _EXECUTOROPERATORCONTROL_H
#define _EXECUTOROPERATORCONTROL_H

#include "executor.h"
#include "mms_msgs/Ack_mission.h"

#include "ros/ros.h"
/*class reference_class{              
	public:
	double latitude;         //deg
	double longitude;		//deg
	double altitude;			//meters
	double yaw;				//rad
	int frame;
};*/

namespace Exec {

  class OperatorControl : public virtual Executor {
	private:
	bool enough_requested;      //enough flag to stop operator control
	int16_t frame;
	bool mission_succesfull;	//true if the command is accomplished
	int16_t qqseq_;                   //internal seq to save the sequence of the saved command

  public:
    OperatorControl (std::string ns, int id);
    virtual ~OperatorControl () {};

    bool prepare ();
    void start ();
    bool abort ();
	virtual bool enough_execution ();
  };

};

#endif
