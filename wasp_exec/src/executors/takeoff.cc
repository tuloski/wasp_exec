#include "takeoff.h"

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




Exec::TakeOff::TakeOff (std::string ns, int id) : Executor (ns, id) {
  lrs_msgs_tst::TSTExecInfo einfo;
	einfo.can_be_aborted = false;
	einfo.can_be_enoughed = false;
	einfo.can_be_paused = false;
	set_exec_info(ns, id, einfo);
	update_from_exec_info (einfo);
}


void Exec::TakeOff::callbackAckMission(const mms_msgs::Ack_mission::ConstPtr& msg){
	ROS_INFO ("Exec::TakeOff::Received ack. Sequence: %d - Mission item: %d",msg->seq, msg->mission_item_reached);
	if (msg->mission_item_reached && (msg->seq == qqseq_) && (qqseq_ != 0)){
		mission_succesfull = true;
	}
	if (!msg->mav_mission_accepted && (msg->seq == qqseq_) && (qqseq_ != 0)){
		fail ("TakeOff: action not possible in actual state");
		return;
	}
}

bool Exec::TakeOff::prepare () {
  bool res = true;
	qqseq_ = 0;
  ROS_INFO ("Exec::TakeOff::prepare");
  mission_succesfull = false;

  if (res) {
    set_active_flag (node_ns, node_id, true);
  }

  return res;
}


void Exec::TakeOff::start () {
	ros::Subscriber mission_ack_sub;   //subscriber to ack_mission topic
	ROS_INFO ("Exec::TakeOff::start: %s - %d", node_ns.c_str(), node_id);
	mission_ack_sub = global_nh->subscribe("ack_mission", 10, &Exec::TakeOff::callbackAckMission,this);
	try {

		if (!do_before_work ()) {
		  return;
		}

		/*if (!get_param("z", height)){                          //DEPRECATED
			fail ("takeoff: parameter z is missing");
			return;
		}*/
		
		/*if (int32_params["follow_ground_flag"].have_value) {                  //TODO add this flag to tstfactory?? otherwise always baro
			if (int32_params["follow_ground_flag"].value == 1) frame = 11;       //Sonar
			else frame = 6;      //Altitude AMSL.
		} else*/
		frame = 6;      //AMSL UNIBO

		ROS_INFO ("Exec::TakeOff: Execution unit: %s", tni.execution_ns.c_str());

		mms_msgs::Cmd cmd1;
		mms_msgs::Cmd cmd2;

		cmd1.target_system = 0;
		cmd1.target_component = 0;
		cmd1.seq = 0;
		cmd1.frame = frame;	
		cmd1.command = 179;      //SET HOME       
		cmd1.current = 0;				//not used
		cmd1.autocontinue = 0;		//not used
		cmd1.param1 = 0;         //not used
		cmd1.param2 = 0;         //not used
		cmd1.param3 = 0;        	//not used
		cmd1.param4 = 0;         //not used
		cmd1.param5 = 0;    			//not used
		cmd1.param6 = 0;	   			//not used
		cmd1.param7 = 0;					//not used

		cmd2.target_system = 0;
		cmd2.target_component = 0;
		{                                              //brackets are to ensure (I hope) that after the shared resource (seq) is accessed, the mutex destructor is called and the mutex unlocked
			boost::mutex::scoped_lock lock(seq_mutex);
			qqseq_ = global_seq;
			cmd2.seq = global_seq++;	
		}
		ROS_INFO ("Sequence for TakeOff: %d",qqseq_);
		cmd2.frame = frame;			
		cmd2.command = 22;         //TAKEOFF         
		cmd2.current = 0;			//not used
		cmd2.autocontinue = 0;		//not used
		cmd2.param1 = 0;            //not used
		cmd2.param2 = 0;          //not used
		cmd2.param3 = 0;        //not used
		cmd2.param4 = 0;            //not used
		cmd2.param5 = 0;    		//not used
		cmd2.param6 = 0;	   		//not used
		cmd2.param7 = 7;		//height (m)    TODO how to decide appropriate take-off height??



		global_cmd_pub->publish (cmd1);
		usleep(500000);              //little sleep of 0.5s between set home and takeoff command
		global_cmd_pub->publish (cmd2);

		//
		// Replace the sleep with useful work.
		//

		max_time = (int)(25 / 1.0f * 1000.0f * 4.0f) + 20 * 1000; //TODO make better estimation(ms): actual--> height(m) / speed(m/s) * s2ms * safety_coeff + time2arm(s) * s2ms
		//ROS_INFO ("Exec::TakeOff: Max_time: %d - height: %f", max_time, height);

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
			fail ("TakeOff: Timeout");
			return;
		} else {
			set_succeeded_flag (node_ns, node_id, true);
			set_aborted_flag (node_ns, node_id, false);
			set_finished_flag (node_ns, node_id, true);
		}

		ROS_INFO ("Exec::TakeOff: FINISHED");

		wait_for_postwork_conditions ();
	}
	catch (boost::thread_interrupted) {
		ROS_ERROR("BOOST INTERUPTED IN TakeOff");
		set_succeeded_flag (node_ns, node_id, false);
		set_aborted_flag (node_ns, node_id, true);
		set_finished_flag (node_ns, node_id, true);
	}
}

bool Exec::TakeOff::abort () {
  bool res = false;
  ROS_INFO("Exec::TakeOff::abort");

  return res;
}
