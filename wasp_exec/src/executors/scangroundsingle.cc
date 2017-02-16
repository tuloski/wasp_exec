#include "scangroundsingle.h"

#include "ros/ros.h"

#include <boost/thread.hpp>
#include <iostream>

#include <string>
#include <math.h>

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

//TODOs:

Exec::ScanGroundSingle::ScanGroundSingle (std::string ns, int id) : 
  Executor (ns, id), 
  enough_requested(false), pause_requested(false),
  continue_requested(false),received_qos_sensors(false) {
	lrs_msgs_tst::TSTExecInfo einfo;
	einfo.can_be_enoughed = true;
	einfo.can_be_paused = true;
  einfo.can_be_enoughed = true;
	set_exec_info(ns, id, einfo);
	update_from_exec_info (einfo);
}


void Exec::ScanGroundSingle::callbackAckMission(const mms_msgs::Ack_mission::ConstPtr& msg){
	ROS_INFO ("Exec::ScanGroundSingle::Received ack. Sequence: %d - Mission item: %d",msg->seq, msg->mission_item_reached);
	if (msg->mission_item_reached && (msg->seq == qqseq_) && (qqseq_ != 0)){
		mission_succesfull = true;
	}
	if (!msg->mav_mission_accepted && (msg->seq == qqseq_) && (qqseq_ != 0)){
		fail ("Exec::ScanGroundSingle: action not possible in actual state");
		return;
	}
}

void Exec::ScanGroundSingle::callbackSensors(const qos_sensors_autopilot::Qos_sensors::ConstPtr& msg){
	qos_sens_ = *msg;
	received_qos_sensors = true;
	//ROS_ERROR ("Exec::ScanGroundSingle::QOS SENSORS! %s - %s",(qos_sens_.camera_present ? "true" : "false"),(qos_sens_.camera_working ? "true" : "false"));
}

void Exec::ScanGroundSingle::callbackGridInfo(const reference::Grid_info::ConstPtr& msg){
	//ROS_INFO ("Exec::ScanGroundSingle::Received grid info.");
	if (!msg->success) fail("ScanGroundSingle: GRID algorithm failed");
	else if (!received_grid_info){
		ROS_INFO ("Exec::ScanGroundSingle::Received grid info. INSIDE");
		received_grid_info = true;
		max_time = (int)(msg->exec_time * 1000.0f * 7); //milliseconds and safety coefficient
	}
}

bool Exec::ScanGroundSingle::check () {
	ROS_INFO ("Exec::ScanGroundSingle::Inside check");
	bool res = true;
	ros::Subscriber sensors_ack_sub;   //subscriber to qos_sensors topic
	sensors_ack_sub = global_nh->subscribe("qos_sensors", 10, &Exec::ScanGroundSingle::callbackSensors,this);

	string sensortype;
	fetch_node_info();

	if (init_params()) {
		ROS_INFO ("Exec::ScanGroundSingle::Init param");
		if (get_param("sensor-type", sensortype)) {
			while (!received_qos_sensors){
				ros::spinOnce();    //when called from delegation there is no ROS so we need to run the spin to read the topics
				usleep(200);
			}
			ROS_INFO ("Exec::ScanGroundSingle::Got sensors list");
			received_qos_sensors = false;
			//
			// Check that we have this sensor. Return false if we do not have this sensor.
			//
			if (sensortype == "camera"){
				if (qos_sens_.camera_present && qos_sens_.camera_working){
					res = true;
				} else {
					ROS_INFO ("Exec::ScanGroundSingle::NO CAMERA SENSOR OR BROKEN!!! %d - %d",qos_sens_.camera_present,qos_sens_.camera_working);
					return false;
				}
			} else if (sensortype == "artva"){
				if (qos_sens_.artva_present && qos_sens_.artva_working && qos_sens_.sonar_present && qos_sens_.sonar_working){
					res = true;
				} else {
					return false;
				}
			} else if (sensortype == "laser"){
				if (qos_sens_.laser_present && qos_sens_.laser_working){
					res = true;
				} else {
					return false;
				}
			} else {
				ROS_INFO ("Exec::ScanGroundSingle::NO Sensor PARAM wrong: should be camera, artva or laser!!!");
				return false;
			}
		} else {
			ROS_INFO ("Exec::ScanGroundSingle::NO Sensor param!!!");
			return false;
		}
	}
  ROS_INFO ("Exec::ScanGroundSingle::Finished Check");
  return res;
}

bool Exec::ScanGroundSingle::prepare () {
  bool res = true;
  received_grid_info = false;
  max_time = 1;   //if not received from grid info it will stop the wait loop immediately
  qqseq_ = 0;
  ROS_INFO ("Exec::ScanGroundSingle::prepare");
  mission_succesfull = false;  

  if (res) {
    set_active_flag (node_ns, node_id, true);
  }

  return res;
}


void Exec::ScanGroundSingle::start () {
	ros::Subscriber mission_ack_sub;   //subscriber to ack_mission topic
	ros::Subscriber grid_info_sub;   //subscriber to ack_mission topic
	ROS_INFO ("Exec::ScanGroundSingle::start: %s - %d", node_ns.c_str(), node_id);
	mission_ack_sub = global_nh->subscribe("ack_mission", 10, &Exec::ScanGroundSingle::callbackAckMission,this);
	grid_info_sub = global_nh->subscribe("grid_info", 2, &Exec::ScanGroundSingle::callbackGridInfo,this);
	
	ros::Subscriber sensors_ack_sub;   //subscriber to qos_sensors topic
	sensors_ack_sub = global_nh->subscribe("qos_sensors", 10, &Exec::ScanGroundSingle::callbackSensors,this);

	try {

		if (!do_before_work ()) {
		  return;
		}
		string sensor_type;

		if (!get_param("area", vertex)) {                          //vertex of the area
			fail("ScanGroundSingle: Parameter 'area' do not exist or is not set");
		return;
		}
		if (!get_param("sensor-type", sensor_type)) {                          //vertex of the area
			fail("ScanGroundSingle: Parameter 'sensor-type' do not exist or is not set");
			return;
		}
		if (!get_param("follow_ground_flag", flag_ground)){
			frame = 6;     //AMSL
			altitude = 1590;	//TODO appropriate altitude should be taken from DCM!!!
		} else {
			if (flag_ground){
				frame = 11;   //SONAR
				if (!get_param("follow_ground_altitude", altitude)){
					fail("ScanGroundSingle: Parameter 'follow_ground_altitude' do not exist but flag 'follow_ground' is active");
					return;
				}
			}
			else frame = 6;				//AMSL
			altitude = 1590;			//TODO appropriate altitude should be taken from DCM!!!
		}
		if (sensor_type == "artva" && !flag_ground){
			fail("ScanGroundSingle: Missing 'follow_ground_altitude' flag with sensor: 'artva'");
			return;
		}
		if (!get_param("repeat_flag", flag_repeat)){
			flag_repeat = false;
		}

		//Check sensors status
		while (!received_qos_sensors){
			ros::spinOnce();    //when called from delegation there is no ROS so we need to run the spin to read the topics
			usleep(200);
		}
		ROS_INFO ("Exec::ScanGroundSingle::Got sensors list");
		received_qos_sensors = false;
		//
		// Check that we have this sensor. Return false if we do not have this sensor.
		//
		if (sensor_type == "camera"){
			if (qos_sens_.camera_present && qos_sens_.camera_working){
				//Good
			} else {
				fail("ScanGroundSingle: NO CAMERA SENSOR!!!");
				return;
			}
		} else if (sensor_type == "artva"){
			if (qos_sens_.artva_present && qos_sens_.artva_working && qos_sens_.sonar_present && qos_sens_.sonar_working){
				//Good
			} else {
				fail("ScanGroundSingle: NO ARTVA SENSOR!!!");
				return;
			}
		} else if (sensor_type == "laser"){
			if (qos_sens_.laser_present && qos_sens_.laser_working){
				//Good
			} else {
				fail("ScanGroundSingle: NO LASER SENSOR!!!");
				return;
			}
		} else {
			ROS_INFO ("Exec::ScanGroundSingle::NO Sensor PARAM wrong: should be camera, artva or laser!!!");
			return;
		}

		//----------- Execute

		ROS_INFO ("Exec::ScanGroundSingle: Execution unit: %s", tni.execution_ns.c_str());

		//ROS_INFO ("Exec::ScanGroundSingle: %f %f %f %s - %f", p.point.x, p.point.y, p.point.z, p.header.frame_id.c_str(), speed);

		mms_msgs::Cmd cmd;
		int n_messages = (int)ceil((double)vertex.size()/3.0f);

		ROS_INFO ("Exec::ScanGroundSingle vertex_size: %zu", vertex.size());
		ROS_INFO ("Exec::ScanGroundSingle n_messages: %d", n_messages);

		enum string_code {
			ARTVA,
			CAMERA
		};

		string_code sensor;
		
		if (sensor_type == "artva") sensor = ARTVA;
		else if (sensor_type == "camera") sensor = CAMERA;
		
		
		switch (sensor){
			case ARTVA:
			//ROS_INFO ("Sending GRID for camera");
			cmd.target_system = 0;                
			cmd.target_component = 0;
			{                                              //brackets are to ensure (I hope) that after the shared resource (seq) is accessed, the mutex destructor is called and the mutex unlocked
				boost::mutex::scoped_lock lock(seq_mutex);
				qqseq_ = global_seq;
				cmd.seq = global_seq++;
			}
			//ROS_INFO ("Sequence for ScanGroundSingle: %d",qqseq_);
			cmd.frame = frame;		
			cmd.command = 160;         //GRID         
			cmd.current = 0;			//not used
			cmd.autocontinue = 0;		//not used
			cmd.param1 = 1;            //speed     //TODO calculate an appropriate speed
			cmd.param2 = 2;          //distance //TODO calculate an appropriate distance
			cmd.param3 = altitude;        		//altitude wrt ground
			cmd.param4 = vertex.size();       		//N vertex
			cmd.param5 = 1;                     //yaw orienting at every WP
			cmd.param6 = 0;	   					//not used, maybe for acceptance radius?
			cmd.param7 = flag_repeat ? 1.0f : 0.0f;	//flag repeat for patrolling
			global_cmd_pub->publish (cmd);
			//ROS_INFO ("Exec::ScanGroundSingle: Sent GRID command");
			
			for (int i=0; i<n_messages; i++){
				cmd.seq = 0;
				cmd.frame = frame;
				cmd.command = 161;         //GRID VERTEX    
				cmd.current = 0;			//not used
				cmd.autocontinue = 0;		//not used
				if (i*3+1 <= vertex.size()){
					cmd.param1 = vertex[i*3].latitude;             //vertex i+1 x
					cmd.param2 = vertex[i*3].longitude;          	 //vertex i+1 y
					//ROS_INFO ("Exec::ScanGroundSingle: Vertex : %f - %f", vertex[i*3].latitude, vertex[i*3].longitude);
				} else {
					cmd.param1 = 0;             //vertex i+1 x
					cmd.param2 = 0;          	//vertex i+1 y
				}
				if (i*3+2 <= vertex.size()){
					cmd.param3 = vertex[i*3+1].latitude;             //vertex i+2 x
					cmd.param4 = vertex[i*3+1].longitude;          	 //vertex i+2 y
					//ROS_INFO ("Exec::ScanGroundSingle: Vertex : %f - %f", vertex[i*3+1].latitude, vertex[i*3+1].longitude);
				} else {
					cmd.param3 = 0;             //vertex i+2 x
					cmd.param4 = 0;          	//vertex i+2 y
				}
				if (i*3+3 <= vertex.size()){
					cmd.param5 = vertex[i*3+2].latitude;             //vertex i+3 x
					cmd.param6 = vertex[i*3+2].longitude;          	 //vertex i+3 y
					//ROS_INFO ("Exec::ScanGroundSingle: Vertex : %f - %f", vertex[i*3+2].latitude, vertex[i*3+2].longitude);
				} else {
					cmd.param5 = 0;             //vertex i+3 x
					cmd.param6 = 0;          	//vertex i+3 y
				}
				cmd.param7 = 0;		
				global_cmd_pub->publish (cmd);
				//ROS_INFO ("Exec::ScanGroundSingle: Sent vertex command");
			}
			break;
		
			case CAMERA:
			//ROS_INFO ("Sending GRID for camera");
			cmd.target_system = 0;                
			cmd.target_component = 0;
			{                                              //brackets are to ensure (I hope) that after the shared resource (seq) is accessed, the mutex destructor is called and the mutex unlocked
				boost::mutex::scoped_lock lock(seq_mutex);
				qqseq_ = global_seq;
				cmd.seq = global_seq++;
			}
			//ROS_INFO ("Sequence for ScanGroundSingle: %d",qqseq_);
			cmd.frame = frame;		
			cmd.command = 160;         //GRID         
			cmd.current = 0;			//not used
			cmd.autocontinue = 0;		//not used
			cmd.param1 = 1.8;            //speed     //TODO calculate an appropriate speed
			cmd.param2 = 5;          //distance //TODO calculate an appropriate distance
			cmd.param3 = altitude;        		//altitude, depends on frame
			cmd.param4 = vertex.size();       		//N vertex
			cmd.param5 = 0;                     //yaw orienting at every WP
			cmd.param6 = 0;	   					//not used, maybe for acceptance radius?
			cmd.param7 = (float)flag_repeat;	//flag repeat for patrolling
			global_cmd_pub->publish (cmd);
			//ROS_INFO ("Exec::ScanGroundSingle: Sent GRID command");

			for (int i=0; i<n_messages; i++){
				cmd.seq = 0;
				cmd.frame = frame;
				cmd.command = 161;         //GRID VERTEX    
				cmd.current = 0;			//not used
				cmd.autocontinue = 0;		//not used
				if (i*3+1 <= vertex.size()){
					cmd.param1 = vertex[i*3].latitude;             //vertex i+1 x
					cmd.param2 = vertex[i*3].longitude;          	 //vertex i+1 y
					//ROS_INFO ("Exec::ScanGroundSingle: Vertex : %f - %f", vertex[i*3].latitude, vertex[i*3].longitude);
				} else {
					cmd.param1 = 0;             //vertex i+1 x
					cmd.param2 = 0;          	//vertex i+1 y
				}
				if (i*3+2 <= vertex.size()){
					cmd.param3 = vertex[i*3+1].latitude;             //vertex i+2 x
					cmd.param4 = vertex[i*3+1].longitude;          	 //vertex i+2 y
					//ROS_INFO ("Exec::ScanGroundSingle: Vertex : %f - %f", vertex[i*3+1].latitude, vertex[i*3+1].longitude);
				} else {
					cmd.param3 = 0;             //vertex i+2 x
					cmd.param4 = 0;          	//vertex i+2 y
				}
				if (i*3+3 <= vertex.size()){
					cmd.param5 = vertex[i*3+2].latitude;             //vertex i+3 x
					cmd.param6 = vertex[i*3+2].longitude;          	 //vertex i+3 y
					//ROS_INFO ("Exec::ScanGroundSingle: Vertex : %f - %f", vertex[i*3+2].latitude, vertex[i*3+2].longitude);
				} else {
					cmd.param5 = 0;             //vertex i+3 x
					cmd.param6 = 0;          	//vertex i+3 y
				}
				cmd.param7 = 0;		
				global_cmd_pub->publish (cmd);
				//ROS_INFO ("Exec::ScanGroundSingle: Sent vertex command");
			}
			break;
		}


		//
		// Replace the sleep with useful work.
		//
		while (!received_grid_info){        //need to receive exec time from reference to fill max_time     //TODO check maybe dangerous
			usleep(1000);
		}
		
		if (flag_repeat){
			max_time = 0;  //unlimited time because patrolling
		}
		
		switch (sensor){         //Send sensor primitives
			case ARTVA: 
			//TODO ARTVA
			//DO we need a node for handling ARTVA or is it always running? Maybe the QoS node at least.
			break;
		
			case CAMERA:
			cmd.target_system = 0;          
			cmd.target_component = 0;
			cmd.seq = 0;
			cmd.frame = 0;			//not used
			cmd.command = 2000;         //MAV_CMD_IMAGE_START_CAPTURE         
			cmd.current = 0;			//not used
			cmd.autocontinue = 0;		//not used
			cmd.param1 = 2;            //delay between photos     //TODO set appropriate
			//cmd.param2 = round(max_time/1000.0f/1.0f);         //number of photos to take. Calculated as max_time (in seconds) divided by the delay (1s in this case).
			cmd.param2 = 0;         //number of photos to take. Unlimited
			cmd.param3 = 0;        	//resolution. Not used now. TODO to add
			cmd.param4 = 0;       	//not used
			cmd.param5 = 0;    		//not used
			cmd.param6 = 0;	   		//not used
			cmd.param7 = 0;			//not used

			global_cmd_pub->publish (cmd);
			ROS_INFO ("Exec::ScanGroundSingle: Sent take pictures command");
			break;
		}

		boost::this_thread::interruption_point();
		
		if (max_time != 0){                       //waiting for mission successful
			for (int i=0; i<max_time; i++) {
				usleep(1000);
				boost::this_thread::interruption_point();
				if (pause_requested){
					pause_requested = false;
					set_paused_flag (node_ns, node_id, true);
					//SEND PAUSE COMMAND
					cmd.seq = 0;
					cmd.frame = 0;			//not used
					cmd.command = 252;         //PAUSE/CONTINUE       
					cmd.param1 = 0;            //0-->Pause 1-->Continue
					global_cmd_pub->publish (cmd);
					while (!continue_requested){
						//Wait for continue
						usleep(1000);
					}
					continue_requested = false;
					set_paused_flag (node_ns, node_id, false);
					//SEND CONTINUE COMMAND
					cmd.seq = 0;
					cmd.frame = 0;			//not used
					cmd.command = 252;         //PAUSE/CONTINUE       
					cmd.param1 = 1;            //0-->Pause 1-->Continue
					global_cmd_pub->publish (cmd);
				}
				if (mission_succesfull) break;
			}
		} else{									//waiting for enough
			while (!enough_requested) {
			  usleep (1000);
			  boost::this_thread::interruption_point();
			  if (pause_requested){
					pause_requested = false;
					set_paused_flag (node_ns, node_id, true);
					//SEND PAUSE COMMAND
					cmd.seq = 0;
					cmd.frame = 0;			//not used
					cmd.command = 252;         //PAUSE/CONTINUE       
					cmd.param1 = 0;            //0-->Pause 1-->Continue
					global_cmd_pub->publish (cmd);
					while (!continue_requested){
						//Wait for continue
						usleep(1000);
					}
					continue_requested = false;
					set_paused_flag (node_ns, node_id, false);
					//SEND CONTINUE COMMAND
					cmd.seq = 0;
					cmd.frame = 0;			//not used
					cmd.command = 252;         //PAUSE/CONTINUE       
					cmd.param1 = 1;            //0-->Pause 1-->Continue
					global_cmd_pub->publish (cmd);
				}
			}
			mission_succesfull = true;
			//Mission finished-->send last waypoint as reference
			mms_msgs::Cmd cmd;
			cmd.target_system = 0;          
			cmd.target_component = 0;
			cmd.seq = 0;
			cmd.command = 16;         //WAYPOINT         
			cmd.current = 0;			//not used
			cmd.autocontinue = 0;		//not used
			cmd.param1 = 1;            //speed, not used now, TODO to add
			cmd.param2 = 1.5;          //acceptance radius X-Y, not used now, TODO to add. TODO add a parameter to FlyTo maybe
			cmd.param3 = 0.4;        //acceptance radius Z, not used now, TODO to add. TODO add a parameter to FlyTo maybe
			{
				boost::mutex::scoped_lock lock(ref_mutex);                                     
				cmd.param4 = latest_reference.Yawangle;
				cmd.frame = latest_reference.frame;
				cmd.param5 = (float)(latest_reference.Latitude)/10000000.0f;    //degrees WGS84
				cmd.param6 = (float)(latest_reference.Longitude)/10000000.0f;	   //degrees WGS84
				cmd.param7 = (float)(latest_reference.AltitudeRelative)/1000.0f;		//meters
			}

			global_cmd_pub->publish (cmd);
			ROS_INFO ("Exec::Flyto: Sent WP command");
		}

		//
		// When we reach this point the node execution whould be finished.
		//
		
		cmd.command = 2001;         //MAV_CMD_IMAGE_STOP_CAPTURE
		global_cmd_pub->publish (cmd);
		ROS_INFO ("Exec::ScanGroundSingle: Sent stop taking pictures command");

		if (!mission_succesfull && !flag_repeat){
			fail ("ScanGroundSingle: Timeout");
			return;
		}

		ROS_INFO ("Exec::ScanGroundSingle: FINISHED");

		wait_for_postwork_conditions ();
	}
	catch (boost::thread_interrupted) {
		ROS_ERROR("BOOST INTERUPTED IN ScanGroundSingle");
		set_succeeded_flag (node_ns, node_id, false);
		set_aborted_flag (node_ns, node_id, true);
		set_finished_flag (node_ns, node_id, true);
	}
}

bool Exec::ScanGroundSingle::abort () {
  bool res = false;
  ROS_ERROR("Exec::ScanGroundSingle::abort");
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

bool Exec::ScanGroundSingle::enough_execution () {
  bool res = true;
  ROS_ERROR ("Exec::ScanGroundSingle::enough_execution");
  enough_requested = true;
  return res;
}

bool Exec::ScanGroundSingle::request_pause () {
  bool res = true;
  ROS_ERROR ("Exec::ScanGroundSingle::request_pause");
  pause_requested = true;
  return res;
}

bool Exec::ScanGroundSingle::continue_execution () {
  bool res = true;
  ROS_ERROR ("Exec::ScanGroundSingle::request_continue");
  continue_requested = true;
  return res;
}
