#include "rotateyaw.h"

#include "ros/ros.h"

#include <boost/thread.hpp>
#include <iostream>

#include "lrs_msgs_tst/ConfirmReq.h"
#include "mms_msgs/Cmd.h"
#include "mms_msgs/Ack_mission.h"
#include "guidance_node_amsl/Reference.h"

#include "executil.h"

extern ros::NodeHandle * global_nh;
extern ros::Publisher * global_confirm_pub;
extern ros::Publisher * global_cmd_pub;

extern std::map<std::string, boost::thread *> threadmap;

extern int16_t global_seq;
extern boost::mutex seq_mutex;
extern boost::mutex ref_mutex;   

extern guidance_node_amsl::Reference latest_reference;

using namespace std;




Exec::RotateYaw::RotateYaw (std::string ns, int id) : Executor (ns, id) {
	lrs_msgs_tst::TSTExecInfo einfo;
	einfo.can_be_aborted = false;
	einfo.can_be_enoughed = false;
	einfo.can_be_paused = false;
	set_exec_info(ns, id, einfo);
	update_from_exec_info (einfo);	
}


void Exec::RotateYaw::callbackAckMission(const mms_msgs::Ack_mission::ConstPtr& msg){
	ROS_INFO ("Exec::RotateYaw::Received ack. Sequence: %d - Mission item: %d",msg->seq, msg->mission_item_reached);
	if (msg->mission_item_reached && (msg->seq == qqseq_) && (qqseq_ != 0)){
		mission_succesfull = true;
	}
	if (!msg->mav_mission_accepted && (msg->seq == qqseq_) && (qqseq_ != 0)){
		fail ("RotateYaw: action not possible in actual state");
		return;
	}
}

bool Exec::RotateYaw::prepare () {
  bool res = true;
  qqseq_ = 0;
  ROS_INFO ("Exec::RotateYaw::prepare");
  mission_succesfull = false;  

  if (res) {
    set_active_flag (node_ns, node_id, true);
  }

  return res;
}


void Exec::RotateYaw::start () {
	ros::Subscriber mission_ack_sub;   //subscriber to ack_mission topic
	ROS_INFO ("Exec::RotateYaw::start: %s - %d", node_ns.c_str(), node_id);
	mission_ack_sub = global_nh->subscribe("ack_mission", 10, &Exec::RotateYaw::callbackAckMission,this);
	try {
		if (!do_before_work ()) {
		  return;
		}

		if (!get_param("heading", yaw)) {
			fail ("RotateYaw: parameter heading is missing");
			return;
		}
		yaw = yaw / 180.0f * M_PI;

		ROS_INFO ("Exec::RotateYaw: Execution unit: %s", tni.execution_ns.c_str());

		//
		//
		//

		mms_msgs::Cmd cmd;

		cmd.target_system = 0;
		cmd.target_component = 0;
		{                                              //brackets are to ensure (I hope) that after the shared resource (seq) is accessed, the mutex destructor is called and the mutex unlocked
			boost::mutex::scoped_lock lock(seq_mutex);
			qqseq_ = global_seq;
			cmd.seq = global_seq++;
		}
		ROS_INFO ("Sequence for RotateYaw: %d",qqseq_);
		{
			boost::mutex::scoped_lock lock(ref_mutex);
			cmd.frame = latest_reference.frame;
		}
		cmd.command = 16;         //WAYPOINT
		cmd.current = 0;			//not used
		cmd.autocontinue = 0;		//not used
		cmd.param1 = 0;            //not used
		cmd.param2 = 1.5;          //acceptance angle, not used now, TODO to add. TODO add a parameter to RotateYaw maybe
		cmd.param3 = 0;            //speed, not used now, TODO to add
		cmd.param4 = yaw;
		{
			boost::mutex::scoped_lock lock(ref_mutex);
			cmd.param5 = (float)(latest_reference.Latitude)/10000000.0f;    //degrees WGS84
			cmd.param6 = (float)(latest_reference.Longitude)/10000000.0f;	   //degrees WGS84
			cmd.param7 = (float)(latest_reference.AltitudeRelative)/1000.0f;		//meters
		}

	
		global_cmd_pub->publish (cmd);
		ROS_INFO ("Exec::RotateYaw: Sent WP command");


		//
		// Replace the sleep with useful work.
		//
	
		max_time = 15000;		//TODO make estimation of the execution time (ms). Let's suppose that with 15s it should make any rotation
	
		boost::this_thread::interruption_point();
		for (int i=0; i<max_time; i++) {
			usleep(1000);
			boost::this_thread::interruption_point();
			if (mission_succesfull) break;
		}

		//
		// When we reach this point the node execution whould be finished.
		//

		if (!mission_succesfull){
			fail ("RotateYaw: Timeout");
			return;
		} else {
			set_succeeded_flag (node_ns, node_id, true);
			set_aborted_flag (node_ns, node_id, false);
			set_finished_flag (node_ns, node_id, true);
		}

		ROS_INFO ("Exec::RotateYaw: FINISHED");

		wait_for_postwork_conditions ();
	}
	catch (boost::thread_interrupted) {
		ROS_ERROR("BOOST INTERUPTED IN RotateYaw");
		set_succeeded_flag (node_ns, node_id, false);
		set_aborted_flag (node_ns, node_id, true);
		set_finished_flag (node_ns, node_id, true);
	}
}

bool Exec::RotateYaw::abort () {
  bool res = false;
  ROS_INFO("Exec::TakeOff::abort");

  return res;
}
