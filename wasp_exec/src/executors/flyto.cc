#include "flyto.h"

#include "ros/ros.h"

#include <boost/thread.hpp>
#include <iostream>

#include "lrs_msgs_tst/ConfirmReq.h"
#include "mms_msgs/Cmd.h"
#include "mms_msgs/Ack_mission.h"
#include <wgs84_ned_lib/wgs84_ned_lib.h>   
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

//TODO stop timer in case of MANUAL_FLIGHT and restart when rollback?

Exec::FlyTo::FlyTo (std::string ns, int id) : Executor (ns, id) {
	lrs_msgs_tst::TSTExecInfo einfo;
	einfo.can_be_aborted = true;
	einfo.can_be_enoughed = false;
	einfo.can_be_paused = false;
	set_exec_info(ns, id, einfo);
	update_from_exec_info (einfo);	
}


void Exec::FlyTo::callbackAckMission(const mms_msgs::Ack_mission::ConstPtr& msg){
	ROS_INFO ("Exec::FlyTo::Received ack. Sequence: %d - Mission item: %d",msg->seq, msg->mission_item_reached);
	if (msg->mission_item_reached && (msg->seq == qqseq_) && (qqseq_ != 0)){
		mission_succesfull = true;
	}
	if (!msg->mav_mission_accepted && (msg->seq == qqseq_) && (qqseq_ != 0)){
		fail ("FlyTo: action not possible in actual state");
		return;
	}
}


bool Exec::FlyTo::prepare () {
  bool res = true;
  qqseq_ = 0;
  ROS_INFO ("Exec::FlyTo::prepare");
  mission_succesfull = false;  

  if (res) {
    set_active_flag (node_ns, node_id, true);
  }

  return res;
}


void Exec::FlyTo::start () {
	ROS_INFO ("Exec::FlyTo::start: %s - %d", node_ns.c_str(), node_id);
	ros::Subscriber mission_ack_sub;   //subscriber to ack_mission topic
	mission_ack_sub = global_nh->subscribe("ack_mission", 10, &Exec::FlyTo::callbackAckMission,this);
	
    try {

        if (!do_before_work ()) {
          return;
        }

        //----------------- Params --------------------//
	    if (!get_param("commanded-speed", speed)) {
		    fail ("flyto: parameter commanded-speed is missing");
		    return;
	    }

	    geographic_msgs::GeoPoint p;
	    if (get_param("p", p)) {
		    ROS_ERROR ("FLYTO: %f %f - %f - %f", p.latitude, p.longitude, p.altitude, speed);
	    } else {
		    fail ("flyto: parameter p is missing");
		    return;
	    }

	    if (!get_param("follow_ground_flag", flag_ground)){
		    frame = 6;     //AMSL
	    } else {
		    if (flag_ground){
			    frame = 11;   //SONAR
			    if (!get_param("follow_ground_altitude", p.altitude)){               //replacing the altitude with the one in follow_ground_altitude
				    fail("ScanGroundSingle: Parameter 'follow_ground_altitude' do not exist but flag 'follow_ground' is active");
			    }
		    }
		    else frame = 6;				//AMSL
	    }
		
		/*std::string location;                                 //location: for indoor at terra
		if (global_nh->getParam("/location", location)){
			if (location == "terra"){
				double temp_latitude = p.latitude;
				p.latitude = p.longitude;
				p.longitude = -temp_latitude;
			}
		}*/
        //----------------------------------------------------//

        ROS_INFO ("Exec::Flyto: Execution unit: %s", tni.execution_ns.c_str());

        //------------ Execution Time Evaluation ---------------//
        {
            boost::mutex::scoped_lock lock(ref_mutex);
            double temp_x1, temp_x2, temp_y1, temp_y2;
	        get_pos_NED_from_WGS84 (&temp_x1, &temp_y1, (float)(latest_reference.Latitude)/10000000.0f, (float)(latest_reference.Longitude)/10000000.0f, 0.0f, 0.0f);
	        get_pos_NED_from_WGS84 (&temp_x2, &temp_y2, p.latitude, p.longitude, 0.0f, 0.0f);
	        double distance = sqrt(pow(temp_x1-temp_x2,2)+pow(temp_y1-temp_y2,2)+pow((float)(latest_reference.AltitudeRelative)/1000.0f-p.altitude,2));
	        max_time = (int)(distance / speed * 1000.0f * 5.0f + 10000.0f);    //TODO this is not true...or at least it depends on how speed is interpreted. This works if speed is along the trajectory
													        // and not only in x-y plane
            ROS_INFO ("Exec::FlyTo: Latest Ref: %d - %d", latest_reference.Latitude, latest_reference.Longitude);
            ROS_INFO ("Exec::FlyTo: Max_time: %d - Distance: %f", max_time, distance);
        }
        //----------------------------------------------------//


        //------------------ Command ----------------------//
        mms_msgs::Cmd cmd;

	    cmd.target_system = 0;          
	    cmd.target_component = 0;
	    {                                              //brackets are to ensure (I hope) that after the shared resource (seq) is accessed, the mutex destructor is called and the mutex unlocked
		    boost::mutex::scoped_lock lock(seq_mutex);
		    qqseq_ = global_seq;
		    cmd.seq = global_seq++;
	    }
	    ROS_INFO ("Sequence for FlyTo: %d",qqseq_);
	    cmd.frame = frame;
	    cmd.command = 16;         //WAYPOINT         
	    cmd.current = 0;			//not used
	    cmd.autocontinue = 0;		//not used
	    cmd.param1 = speed;            //speed
	    cmd.param2 = 1.5;          //acceptance radius X-Y, not used now, TODO to add. TODO add a parameter to FlyTo maybe
	    cmd.param3 = 0.4;        //acceptance radius Z, not used now, TODO to add. TODO add a parameter to FlyTo maybe
        {
            boost::mutex::scoped_lock lock(ref_mutex);                                     
	        cmd.param4 = latest_reference.Yawangle;
        }
	    cmd.param5 = p.latitude;    
	    cmd.param6 = p.longitude;	   
	    cmd.param7 = p.altitude;

	    global_cmd_pub->publish (cmd);
	    ROS_INFO ("Exec::Flyto: Sent WP command");
		//----------------------------------------------------//


        //
        // Replace the sleep with useful work.
        //

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
		    fail ("FlyTo: Timeout");
		    return;
    	}else {
		    set_succeeded_flag (node_ns, node_id, true);
		    set_aborted_flag (node_ns, node_id, false);
		    set_finished_flag (node_ns, node_id, true);
	    }

        
	    ROS_INFO ("Exec::FlyTo: FINISHED");

	    wait_for_postwork_conditions ();
    }
    catch (boost::thread_interrupted) {
        ROS_ERROR("BOOST INTERUPTED IN flyto");
        set_succeeded_flag (node_ns, node_id, false);
        set_aborted_flag (node_ns, node_id, true);
        set_finished_flag (node_ns, node_id, true);
    }
}

bool Exec::FlyTo::abort () {
  bool res = false;
  ROS_ERROR("Exec::FlyTo::abort");
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
