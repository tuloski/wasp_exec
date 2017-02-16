#include "land.h"

#include "ros/ros.h"

#include <boost/thread.hpp>
#include <iostream>

#include "executil.h"

extern ros::NodeHandle * global_nh;
extern ros::Publisher * global_confirm_pub;
extern ros::Publisher * global_cmd_pub;

extern std::map<std::string, boost::thread *> threadmap;

extern int16_t global_seq;
extern boost::mutex seq_mutex;


using namespace std;




Exec::Land::Land (std::string ns, int id) : Executor (ns, id) {
  lrs_msgs_tst::TSTExecInfo einfo;
	einfo.can_be_aborted = false;
	einfo.can_be_enoughed = false;
	einfo.can_be_paused = false;
	set_exec_info(ns, id, einfo);
	update_from_exec_info (einfo);
}


void Exec::Land::callbackAckMission(const mms_msgs::Ack_mission::ConstPtr& msg){
	ROS_INFO ("Exec::Land::Received ack. Sequence: %d - Mission item: %d",msg->seq, msg->mission_item_reached);
	if (msg->mission_item_reached && (msg->seq == qqseq_) && (qqseq_ != 0)){
		mission_succesfull = true;
	}
	if (!msg->mav_mission_accepted && (msg->seq == qqseq_) && (qqseq_ != 0)){
		fail ("Land: action not possible in actual state");
		return;
	}
}

bool Exec::Land::prepare () {
  bool res = true;
	qqseq_ = 0;
  ROS_INFO ("Exec::Land::prepare");
  mission_succesfull = false;  

  if (res) {
    set_active_flag (node_ns, node_id, true);
  }

  return res;
}


void Exec::Land::start () {
	ROS_INFO ("Exec::Land::start: %s - %d", node_ns.c_str(), node_id);
	ros::Subscriber mission_ack_sub;   //subscriber to ack_mission topic
	mission_ack_sub = global_nh->subscribe("ack_mission", 10, &Exec::Land::callbackAckMission,this);
	try {

		if (!do_before_work ()) {
		  return;
		}
			
		frame = 6;      //BAROMETER frame. Relative Global UNIBO
		ROS_INFO ("Exec::Land: Execution unit: %s", tni.execution_ns.c_str());


		mms_msgs::Cmd cmd1;

		cmd1.target_system = 0;
		cmd1.target_component = 0;
		{                                              //brackets are to ensure (I hope) that after the shared resource (seq) is accessed, the mutex destructor is called and the mutex unlocked
			boost::mutex::scoped_lock lock(seq_mutex);
			qqseq_ = global_seq;
			cmd1.seq = global_seq++;	
		}
		cmd1.frame = frame;	
		cmd1.command = 21;      //LAND    
		cmd1.current = 0;				//not used
		cmd1.autocontinue = 0;		//not used
		cmd1.param1 = 0;         //not used
		cmd1.param2 = 0;         //not used
		cmd1.param3 = 0;        	//not used
		cmd1.param4 = 0;         //not used
		cmd1.param5 = 0;    			//not used
		cmd1.param6 = 0;	   			//not used
		cmd1.param7 = 0;					//not used
		
		global_cmd_pub->publish (cmd1);

		//
		// Replace the sleep with useful work.
		//

		max_time = 50000;		//TODO make estimation of the execution time?? Hard to estimate for landing

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
			fail ("Land: Timeout");
			return;
		} else {
			set_succeeded_flag (node_ns, node_id, true);
			set_aborted_flag (node_ns, node_id, false);
			set_finished_flag (node_ns, node_id, true);
		}

		ROS_INFO ("Exec::Land: FINISHED");

		wait_for_postwork_conditions ();
	}
	catch (boost::thread_interrupted) {
		ROS_ERROR("BOOST INTERUPTED IN Land");
		set_succeeded_flag (node_ns, node_id, false);
		set_aborted_flag (node_ns, node_id, true);
		set_finished_flag (node_ns, node_id, true);
	}
}

bool Exec::Land::abort () {
  bool res = false;
  ROS_INFO("Exec::TakeOff::abort");

  return res;
}
