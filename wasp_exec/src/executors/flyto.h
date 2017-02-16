// This may look like C code, but it is really -*- C++ -*-

#ifndef _EXECUTORFLYTO_H
#define _EXECUTORFLYTO_H

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

  class FlyTo : public virtual Executor {
	private:
	double x;
	double y;
	double z;
	double speed;
	int16_t frame;
	bool flag_ground;
	bool mission_succesfull;	//true if the command is accomplished
	int16_t qqseq_;                   //internal seq to save the sequence of the saved command
	int max_time;                //max time for execution (ms)

  public:
    FlyTo (std::string ns, int id);
    virtual ~FlyTo () {};

    bool prepare ();
    void start ();
    bool abort ();
    virtual void callbackAckMission(const mms_msgs::Ack_mission::ConstPtr& msg);
  };

};

#endif
