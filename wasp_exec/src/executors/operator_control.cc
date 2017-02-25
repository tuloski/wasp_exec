#include "operator_control.h"

#include "ros/ros.h"

#include <boost/thread.hpp>
#include <iostream>

#include "lrs_msgs_tst/ConfirmReq.h"
#include "mms_msgs/Cmd.h"
//#include "mms_msgs/Ack_mission.h"

#include "executil.h"

extern ros::NodeHandle * global_nh;
extern ros::Publisher * global_confirm_pub;
extern ros::Publisher * global_cmd_pub;

extern std::map<std::string, boost::thread *> threadmap;

extern int16_t global_seq;
extern boost::mutex seq_mutex;
extern boost::mutex ref_mutex;        
//extern guidance_node_amsl::Reference latest_reference;

using namespace std;


Exec::OperatorControl::OperatorControl (std::string ns, int id) : Executor (ns, id), enough_requested(false) {
	lrs_msgs_tst::TSTExecInfo einfo;
	einfo.can_be_aborted = true;
	einfo.can_be_enoughed = true;
	einfo.can_be_paused = false;
	set_exec_info(ns, id, einfo);
	update_from_exec_info (einfo);
}


/*void Exec::OperatorControl::callbackAckMission(const mms_msgs::Ack_mission::ConstPtr& msg){
	ROS_INFO ("Exec::OperatorControl::Received ack. Sequence: %d - Mission item: %d",msg->seq, msg->mission_item_reached);
	if (msg->mission_item_reached && (msg->seq == qqseq_) && (qqseq_ != 0)){
		mission_succesfull = true;
	}
	if (!msg->mav_mission_accepted && (msg->seq == qqseq_) && (qqseq_ != 0)){
		fail ("OperatorControl: action not possible in actual state");
		return;
	}
}*/  //TODO check if we need this


bool Exec::OperatorControl::prepare () {
  bool res = true;
  qqseq_ = 0;
  ROS_INFO ("Exec::OperatorControl::prepare");
  mission_succesfull = false;  

  if (res) {
    set_active_flag (node_ns, node_id, true);
  }

  return res;
}


void Exec::OperatorControl::start () {
	ROS_INFO ("Exec::OperatorControl::start: %s - %d", node_ns.c_str(), node_id);
	//ros::Subscriber mission_ack_sub;   //subscriber to ack_mission topic
	//mission_ack_sub = global_nh->subscribe("/ack_mission", 10, &Exec::OperatorControl::callbackAckMission,this);
	
    try {

        if (!do_before_work ()) {
          return;
        }

        ROS_INFO ("Exec::OperatorControl: Execution unit: %s", tni.execution_ns.c_str());

        //------------------ Command ----------------------//
        mms_msgs::Cmd cmd;

	    cmd.target_system = 0;          
	    cmd.target_component = 0;
	    {                                              //brackets are to ensure (I hope) that after the shared resource (seq) is accessed, the mutex destructor is called and the mutex unlocked
		    boost::mutex::scoped_lock lock(seq_mutex);
		    qqseq_ = global_seq;
		    cmd.seq = global_seq++;
	    }
	    ROS_INFO ("Sequence for OperatorControl: %d",qqseq_);
	    cmd.frame = frame;
	    cmd.command = 252;          //MAV_CMD_OVERRIDE_GOTO (PAUSE/CONTINUE)
	    cmd.current = 0;			//not used
	    cmd.autocontinue = 0;		//not used
	    cmd.param1 = 0;             //0-->PAUSE - 1-->CONTINUE
	    cmd.param2 = 0;          	//not used
	    cmd.param3 = 0;        		//not used
	    cmd.param4 = 0;				//not used
	    cmd.param5 = 0;    			//not used
	    cmd.param6 = 0;	   			//not used
	    cmd.param7 = 0;				//not used

	    global_cmd_pub->publish (cmd);
	    ROS_INFO ("Exec::OperatorControl: Sent PAUSE command");
		//----------------------------------------------------//


        //
        // Replace the sleep with useful work.
        //

	    boost::this_thread::interruption_point();

	    while (!enough_requested()) {
		  usleep (100000);
		  //TODO maybe put timeout
		  boost::this_thread::interruption_point();
		}
	    	ROS_ERROR ("Exec::OperatorControl::enough_execution");
		mission_succesfull = true;

		cmd.target_system = 0;
		cmd.target_component = 0;
		{                                              //brackets are to ensure (I hope) that after the shared resource (seq) is accessed, the mutex destructor is called and the mutex unlocked
			boost::mutex::scoped_lock lock(seq_mutex);
			qqseq_ = global_seq;
			cmd.seq = global_seq++;
		}
		ROS_INFO ("Sequence for OperatorControl: %d",qqseq_);
		cmd.frame = frame;
		cmd.command = 252;         	//MAV_CMD_OVERRIDE_GOTO (PAUSE/CONTINUE)
		cmd.current = 0;			//not used
		cmd.autocontinue = 0;		//not used
		cmd.param1 = 1;             //0-->PAUSE - 1-->CONTINUE
		cmd.param2 = 0;          	//not used
		cmd.param3 = 0;        		//not used
		cmd.param4 = 0;				//not used
		cmd.param5 = 0;    			//not used
		cmd.param6 = 0;	   			//not used
		cmd.param7 = 0;				//not used

		global_cmd_pub->publish (cmd);
		ROS_INFO ("Exec::OperatorControl: Sent CONTINUE command");
	    //
	    // When we reach this point the node execution whould be finished.
	    //

		set_succeeded_flag (node_ns, node_id, true);
		set_aborted_flag (node_ns, node_id, false);
		set_finished_flag (node_ns, node_id, true);
        
	    ROS_INFO ("Exec::OperatorControl: FINISHED");

	    wait_for_postwork_conditions ();
    }
    catch (boost::thread_interrupted) {
	abort_fail("OperatorControl ABORT");
        //set_succeeded_flag (node_ns, node_id, false);
        //set_aborted_flag (node_ns, node_id, true);
        //set_finished_flag (node_ns, node_id, true);
    }
}

/*bool Exec::OperatorControl::enough_execution () {
  bool res = true;
  ROS_ERROR ("Exec::ScanGroundSingle::enough_execution");
  enough_requested = true;
  return res;
}*/

/*bool Exec::OperatorControl::abort () {
  bool res = false;
  ROS_ERROR("Exec::OperatorControl::abort");
  ostringstream os;
  os << node_ns << "-" << node_id;
  if (threadmap.find (os.str()) != threadmap.end()) {                               //PUT AGAIN
    ROS_ERROR("EXECUTOR EXISTS: Sending interrupt to running thread");
    threadmap[os.str()]->interrupt();
    // Platform specific things to to

    return true;
  } else {
    ROS_ERROR ("Executor does not exist: %s", os.str().c_str());
    return false;
  }
  return res;
}*/
