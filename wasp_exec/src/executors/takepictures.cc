#include "takepictures.h"

#include "ros/ros.h"

#include <boost/thread.hpp>
#include <iostream>

#include "lrs_msgs_tst/ConfirmReq.h"
#include "mms_msgs/Cmd.h"
#include "mms_msgs/Ack_mission.h"

#include "executil.h"

extern ros::NodeHandle * global_nh;
extern ros::Publisher * global_confirm_pub;
extern ros::Publisher * global_cmd_pub;

extern std::map<std::string, boost::thread *> threadmap;

extern int16_t global_seq;
extern boost::mutex seq_mutex;

using namespace std;




Exec::TakePictures::TakePictures (std::string ns, int id) : Executor (ns, id),
		received_qos_sensors (false){
	lrs_msgs_tst::TSTExecInfo einfo;
	einfo.can_be_aborted = true;
	einfo.can_be_enoughed = false;
	einfo.can_be_paused = false;
	set_exec_info(ns, id, einfo);
	update_from_exec_info (einfo);
}


void Exec::TakePictures::callbackAckMission(const mms_msgs::Ack_mission::ConstPtr& msg){
	ROS_INFO ("Exec::TakePictures::Received ack. Sequence: %d - Mission item: %d",msg->seq, msg->mission_item_reached);
	if (msg->mission_item_reached && (msg->seq == qqseq_) && (qqseq_ != 0)){
		mission_succesfull = true;
	}
	if (!msg->mav_mission_accepted && (msg->seq == qqseq_) && (qqseq_ != 0)){
		fail ("TakePictures: action not possible in actual state");
		return;
	}
}

void Exec::TakePictures::callbackSensors(const qos_sensors_autopilot::Qos_sensors::ConstPtr& msg){
	qos_sens_ = *msg;
	ROS_INFO ("QoS: received %d - %d",qos_sens_.camera_present, qos_sens_.camera_working);
	received_qos_sensors = true;
}

bool Exec::TakePictures::check () {
	received_qos_sensors = false;
	ROS_INFO ("TakePictures::Inside Check");
	bool res = true;
	ros::Subscriber sensors_ack_sub;   //subscriber to qos_sensors topic
	sensors_ack_sub = global_nh->subscribe("qos_sensors", 10, &Exec::TakePictures::callbackSensors,this);

	fetch_node_info();
	
	while (!received_qos_sensors){
		ROS_INFO ("TakePictures::No QoS for sensors....");
		ros::spinOnce();    //when called from delegation there is no ROS so we need to run the spin to read the topics
		usleep(20000);
	}
	received_qos_sensors = false;
	//
	// Check that we have camera sensor. Return false if we do not have this sensor.
	//
	if (!qos_sens_.camera_present){
		ROS_INFO ("TakePictures::Camera not present");
		res = false;
	}
	if (!qos_sens_.camera_working){
		ROS_INFO ("TakePictures::Camera not working");
		res = false;
	}
	return res;
}


bool Exec::TakePictures::prepare () {
  bool res = true;
  qqseq_ = 0;
  ROS_INFO ("Exec::TakePictures::prepare");
  mission_succesfull = false;  

  if (res) {
    set_active_flag (node_ns, node_id, true);
  }

  return res;
}


void Exec::TakePictures::start () {
	ros::Subscriber mission_ack_sub;   //subscriber to ack_mission topic
	ROS_INFO ("Exec::TakePictures::start: %s - %d", node_ns.c_str(), node_id);
	mission_ack_sub = global_nh->subscribe("ack_mission", 10, &Exec::TakePictures::callbackAckMission,this);
	try {

		if (!do_before_work ()) {
		  return;
		}

		if (int32_params["n"].have_value) {
		  n = int32_params["n"].value;
		} else {
		  fail ("takepicture: parameter n is missing");
		  return;
		}

		if (float64_params["delay_between_pictures_in_seconds"].have_value) {
			delay = float64_params["delay_between_pictures_in_seconds"].value;
		} else {
			delay = 0;
			if (n == 1){
				//If we take 1 photo we don't need delay.
			} else {
				fail("takepicture: parameter delay_between_pictures_in_seconds is missing");
				return;
			}
		}

		ROS_INFO ("Exec::TakePictures: Execution unit: %s", tni.execution_ns.c_str());

		//ROS_INFO ("Exec::TakePictures: %f %f %f %s - %f", p.point.x, p.point.y, p.point.z, p.header.frame_id.c_str(), speed);

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
		ROS_INFO ("Sequence for TakePictures: %d",qqseq_);
		cmd.frame = 0;			//not used
		cmd.command = 2000;         //MAV_CMD_IMAGE_START_CAPTURE         
		cmd.current = 0;			//not used
		cmd.autocontinue = 0;		//not used
		cmd.param1 = delay;            //delay between photos
		cmd.param2 = n;         //number of photos to take. 0 for unlimited
		cmd.param3 = 0;        	//resolution. Not used now. TODO to add
		cmd.param4 = 0;       	//not used
		cmd.param5 = 0;    		//not used
		cmd.param6 = 0;	   		//not used
		cmd.param7 = 0;			//not used

		global_cmd_pub->publish (cmd);
		ROS_INFO ("Exec::TakePictures: Sent take pictures command");
		

		//
		// Replace the sleep with useful work.
		//

		if (n==0){ max_time = 1800000;    //~30 minutes (max flight time)
		} else if (n==1) max_time = 5*1000.0f;	//5 seconds to take one picture
		else max_time = n * delay * 1000.0f * 1.3;		//estimated time given by: (n of photos) x (delay) x (Seconds2Milliseconds) x (Safety Coefficient)

		boost::this_thread::interruption_point();
		for (int i=0; i<max_time; i++) {
			usleep(1000);
			boost::this_thread::interruption_point();
			if (mission_succesfull) break;
		}

		//
		// When we reach this point the node execution whould be finished.
		//

		if (!mission_succesfull) {
			fail ("TakePictures: Timeout");
			return;
		} else {
			set_succeeded_flag (node_ns, node_id, true);
			set_aborted_flag (node_ns, node_id, false);
			set_finished_flag (node_ns, node_id, true);
		}

		ROS_INFO ("Exec::TakePictures: FINISHED");

		wait_for_postwork_conditions ();
	}
	catch (boost::thread_interrupted) {
		ROS_ERROR("BOOST INTERUPTED IN TakePictures");
		set_succeeded_flag (node_ns, node_id, false);
		set_aborted_flag (node_ns, node_id, true);
		set_finished_flag (node_ns, node_id, true);
	}
}

bool Exec::TakePictures::abort () {
  bool res = false;
  ROS_ERROR("Exec::TakePictures::abort");
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
}
