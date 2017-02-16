#include "inairtest.h"

#include "executil.h"
#include "tstutil.h"

#include <iostream>
#include <boost/lexical_cast.hpp>

extern ros::NodeHandle * global_nh;

using namespace std;

Exec::InAirTest::InAirTest (std::string ns, int id) : Executor (ns, id) {

  lrs_msgs_tst::TSTExecInfo einfo;
  einfo.can_be_aborted = false;
  einfo.can_be_enoughed = false;
  einfo.can_be_paused = false;
  set_exec_info(ns, id, einfo);

  update_from_exec_info (einfo);  
}

void Exec::InAirTest::callbackState(const mms_msgs::MMS_status::ConstPtr& msg){
	_mms_status = *msg;
	received_mms_state = true;
}

bool Exec::InAirTest::prepare () {
  bool res = true;
  received_mms_state = false;
  counter_wait_mms_state = 0;
  ROS_INFO ("Exec::InAirTest::prepare");

  if (res) {
    set_active_flag (node_ns, node_id, true);
  }

  return res;
}


void Exec::InAirTest::start () {
  ROS_INFO ("Exec::InAirTest::start: %s - %d", node_ns.c_str(), node_id);

  if (!do_before_work()) {
    ROS_ERROR("Exec InAirTest: do_before_work failed");
    return;
  }

  ROS_INFO ("TREE NS: %s", tni.ns.c_str());
  ROS_INFO ("THIS ID: %d", tni.id);
  ROS_INFO ("PARENT ID: %d", tni.parent_id);

  //  sleep (5);

  string answer = "unknown";

  ros::Subscriber state_sub;   //subscriber to qos_sensors topic
  state_sub = global_nh->subscribe("mms_status", 10, &Exec::InAirTest::callbackState,this);

  while (!received_mms_state){
	ros::spinOnce();    //when called from delegation there is no ROS so we need to run the spin to read the topics
	usleep(200);
	counter_wait_mms_state++;
	if (counter_wait_mms_state > 15){	//Waited mms_state for 3 seconds...something failed
		fail ("Exec InAirTest: Failed to receive mms state");
		return;
	}
  }
  //TODO use enum instead of values
  if (_mms_status.mms_state == 10 || _mms_status.mms_state == 20 || _mms_status.mms_state == 30
		  || _mms_status.mms_state == 40 || _mms_status.mms_state == 45 || _mms_status.mms_state == 71
		  || _mms_status.mms_state == 711 || _mms_status.mms_state == 712){
	  answer = "false";
  } else {
	  answer = "true";
  }
  ROS_ERROR("Exec InAirTest: answer: %s", answer.c_str());


  if (set_parameter_string (tni.ns, tni.parent_id, "answer", answer)) {
    ROS_INFO ("answer set in parent");
  } else {
    fail ("Exec InAirTest: Failed to set answer parameter in parent");
    return;
  }
  
  ROS_INFO ("Exec::InAirTest: finished");

  wait_for_postwork_conditions ();
}


