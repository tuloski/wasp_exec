// This may look like C code, but it is really -*- C++ -*-

#ifndef _EXECUTORTAKEPICTURES_H
#define _EXECUTORTAKEPICTURES_H

#include "executor.h"
#include "mms_msgs/Ack_mission.h"
#include "qos_sensors_autopilot/Qos_sensors.h"

#include "ros/ros.h"

namespace Exec {

  class TakePictures : public virtual Executor {
  private:
	int32_t n;
	float delay;                 //delay in seconds between pictures
	bool mission_succesfull;	//true if the command is accomplished
	int16_t qqseq_;                   //internal seq to save the sequence of the saved command
	int max_time;                //max time for execution (ms)
	qos_sensors_autopilot::Qos_sensors qos_sens_;
	bool received_qos_sensors;

  public:
    TakePictures (std::string ns, int id);
    virtual ~TakePictures () {};

	virtual bool check ();
    bool prepare ();
    void start ();
    bool abort ();
    void callbackAckMission(const mms_msgs::Ack_mission::ConstPtr& msg);
	void callbackSensors(const qos_sensors_autopilot::Qos_sensors::ConstPtr& msg);
  };

};

#endif
