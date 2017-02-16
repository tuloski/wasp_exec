#include "start_data_stream.h"

#include <iostream>
#include <string>

#include "tstutil.h"
#include "executil.h"

using namespace std;

Exec::StartDataStream::StartDataStream (std::string ns, int id) : Executor (ns, id) {
	set_delegation_expandable(false);
	lrs_msgs_tst::TSTExecInfo einfo;
	einfo.can_be_aborted = false;
	set_exec_info(ns, id, einfo);
}


bool Exec::StartDataStream::check () {
  ROS_INFO ("StartDataStream CHECK");

  std::string ns = ros::names::clean (ros::this_node::getNamespace());

  fetch_node_info();

  if (!init_params ()) {
    ROS_ERROR("expand: init_params failed");
    return false;
  }

  return true;
}


bool Exec::StartDataStream::prepare () {
  bool res = true;
  ROS_INFO ("Exec::StartDataStream::prepare");
  if (res) {
    set_active_flag (node_ns, node_id, true);
  }
  return res;
}


void Exec::StartDataStream::start () {
  ROS_INFO ("Exec::StartDataStream::start: %s - %d", node_ns.c_str(), node_id);

  ros::NodeHandle n;

  if (!do_before_work()) {
    return;
  }

  string sensor_type = "";
  get_param("sensor-type", sensor_type);

  ROS_INFO ("STARTING DATA STREAM FOR: %s", sensor_type.c_str());

  sleep (3);

  wait_for_postwork_conditions ();

}

bool Exec::StartDataStream::abort () {
  bool res = false;
  ROS_INFO("Exec::StartDataStream::abort");

  return res;
}