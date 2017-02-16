// This may look like C code, but it is really -*- C++ -*-

#ifndef _EXECUTORSCANGROUNDSINGLE_H
#define _EXECUTORSCANGROUNDSINGLE_H

#include "executor.h"
#include "mms_msgs/Ack_mission.h"
#include "reference/Grid_info.h"
#include "lrs_msgs_tst/ConfirmReq.h"
#include "mms_msgs/Cmd.h"
#include "guidance_node_amsl/Reference.h"
#include "qos_sensors_autopilot/Qos_sensors.h"

#include "ros/ros.h"

namespace Exec {
  class ScanGroundSingle : public virtual Executor {
  private:
	std::vector<geographic_msgs::GeoPoint> vertex;
	double speed;
	double altitude;
	int frame;
	bool flag_ground;
	bool flag_repeat;
	bool enough_requested;      //enough flag to stop patrolling
    bool pause_requested;
    bool continue_requested;
	bool mission_succesfull;	//true if the command is accomplished
	int qqseq_;                   //internal seq to save the sequence of the saved command
	int max_time;                //max time for execution (ms)
	bool received_grid_info;
	qos_sensors_autopilot::Qos_sensors qos_sens_;
	bool received_qos_sensors;

  public:
    ScanGroundSingle (std::string ns, int id);
    virtual ~ScanGroundSingle () {};

	virtual bool check ();
	bool prepare ();
	void start ();
	bool abort ();
	void callbackAckMission(const mms_msgs::Ack_mission::ConstPtr& msg);
	void callbackGridInfo(const reference::Grid_info::ConstPtr& msg);
	void callbackSensors(const qos_sensors_autopilot::Qos_sensors::ConstPtr& msg);
	virtual bool enough_execution ();
    virtual bool request_pause ();
    virtual bool continue_execution ();
  };

};

#endif
