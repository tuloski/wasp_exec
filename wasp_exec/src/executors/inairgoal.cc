#include "inairgoal.h"

#include <iostream>
#include <string>

#include "tstutil.h"
#include "executil.h"

using namespace std;

Exec::InAirGoal::InAirGoal (std::string ns, int id) : Executor (ns, id) {
  set_delegation_expandable(true);

  lrs_msgs_tst::TSTExecInfo einfo;
  einfo.can_be_aborted = false;
  einfo.can_be_enoughed = false;
  einfo.can_be_paused = false;
  set_exec_info(ns, id, einfo);

  update_from_exec_info (einfo);  
}


int Exec::InAirGoal::expand (int free_id, std::vector<std::string> possible_units,
			     int expansion_try, int & expansion_can_be_tried) {

  std::string ns = ros::names::clean (ros::this_node::getNamespace());

  expansion_can_be_tried = expansion_try;

  ROS_INFO("InAirGoal: expand: %s", ns.c_str());

  //
  // Get node parameters
  //

  fetch_node_info();

  if (!init_params ()) {
    ROS_ERROR("inairgoal: expand: init_params failed");
    return 0;
  }

  //
  // Expand the tree
  //

  vector<string> vars;
  vector<string> cons;

  // Top seq

  int seqid = create_child_node (node_ns, "seq", "seq", node_id);
  ROS_INFO("seqid:%d", seqid);
  set_execution_unit(node_ns, seqid, ns);
  //  set_constraints(node_ns, seqid, vars, cons);
  set_parameter_int32(node_ns, seqid, "unique_node_id", free_id++);

  //
  // Tell operator
  //


	int testif = create_child_node (node_ns, "test-if", "test-if", seqid);
	set_execution_unit(node_ns, testif, ns);
	set_parameter_int32(node_ns, testif, "unique_node_id", free_id++);

	int testid = create_child_node (node_ns, "in-air-test", "in-air-test", testif);
	set_execution_unit(node_ns, testid, ns);
	set_parameter_int32(node_ns, testid, "unique_node_id", free_id++);

	int noop1 = create_child_node (node_ns, "no-op", "no-op", testif);
	set_execution_unit(node_ns, noop1, ns);
	set_parameter_int32(node_ns, noop1, "unique_node_id", free_id++);

	int takeoffid = create_child_node (node_ns, "take-off", "take-off", testif);
	set_execution_unit(node_ns, takeoffid, ns);
	set_parameter_int32(node_ns, takeoffid, "unique_node_id", free_id++);

	int noop2 = create_child_node (node_ns, "no-op", "no-op", testif);
	set_execution_unit(node_ns, noop2, ns);
	set_parameter_int32(node_ns, noop2, "unique_node_id", free_id++);

    
    
  
  return free_id;
}

bool Exec::InAirGoal::check () {
  ROS_INFO ("InAirGoal CHECK");

  std::string ns = ros::names::clean (ros::this_node::getNamespace());

  fetch_node_info();

  if (!init_params ()) {
    ROS_ERROR("expand: init_params failed");
    return false;
  }

  return true;
}


bool Exec::InAirGoal::prepare () {
  bool res = true;
  ROS_INFO ("Exec::InAirGoal::prepare");
  if (res) {
    set_active_flag (node_ns, node_id, true);
  }
  return res;
}


void Exec::InAirGoal::start () {
  ROS_INFO ("Exec::InAirGoal::start: %s - %d", node_ns.c_str(), node_id);

  ros::NodeHandle n;

  if (!do_before_work()) {
    return;
  }

  // Code from sequence executor here, do function call


  if (!do_seq_work(tni, node_ns)) {
    fail("do_seq_work() failed");
    return;
  }

  wait_for_postwork_conditions ();

}




