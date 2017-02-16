#include "ros/ros.h"

#include <boost/thread.hpp>

#include "std_msgs/String.h"

#include <iostream>
#include <sstream>
#include <queue>

#include "lrs_msgs_tst/ConfirmReq.h"
#include "mms_msgs/Cmd.h"

#include "lrs_srvs_exec/TSTCreateExecutor.h"

#include "executors/flyto.h"
#include "executors/takeoff.h"
#include "executors/land.h"
#include "executors/takepictures.h"
#include "executors/scangroundsingle.h"
#include "executors/rotateyaw.h"
#include "executors/leashing.h"
#include "executors/start_data_stream.h"
#include "executors/stop_data_stream.h"
#include "executors/operator_control.h"
#include "executors/inairgoal.h"
#include "executors/inairtest.h"

#include "guidance_node_amsl/Reference.h"

#include "executil.h"

std::map<std::string, Executor *> execmap;
std::map<std::string, boost::thread *> threadmap;
ros::NodeHandle * global_nh;
ros::Publisher * global_confirm_pub;
ros::Publisher * global_cmd_pub;

int16_t global_seq = 1;
//reference_class global_last_reference;
guidance_node_amsl::Reference latest_reference;

boost::mutex mutex;
boost::mutex seq_mutex;         //mutex to handle shared variable seq
boost::mutex ref_mutex;         //mutex to handle shared class reference_class
boost::mutex thread_map_lock;

using namespace std;

void callbackReference(const guidance_node_amsl::Reference::ConstPtr& msg){
	{	
		boost::mutex::scoped_lock lock(ref_mutex);
		//ROS_INFO ("WaspExecutor::Received Reference: %d - %d",msg->Latitude, msg->Longitude);
		latest_reference.Latitude = msg->Latitude;
		latest_reference.Longitude = msg->Longitude;
		latest_reference.AltitudeRelative = msg->AltitudeRelative;
		latest_reference.Yawangle = msg->Yawangle;
		latest_reference.frame = msg->frame;
	}
}

bool create_executor (lrs_srvs_exec::TSTCreateExecutor::Request  &req,
		      lrs_srvs_exec::TSTCreateExecutor::Response &res ) {
  boost::mutex::scoped_lock lock(mutex);
  ROS_INFO("quadexecutor: create_executor: %s %d - %d", req.ns.c_str(), req.id, req.run_prepare);

  ostringstream os;
  os << req.ns << "-" << req.id;
  if (execmap.find (os.str()) != execmap.end()) {
    ROS_INFO ("Executor already exists, overwrites it: %s %d", req.ns.c_str(), req.id);
  }

  string type = get_node_type (req.ns, req.id);
  
  bool found = false;

  Executor * cres = check_exec(type, req.ns, req.id);
  if (cres) {
    execmap[os.str()] = cres;
    found = true;
  }

	if (type == "fly-to") {
		execmap[os.str()] = new Exec::FlyTo (req.ns, req.id);
		found = true;
	}
	if (type == "take-off") {
		execmap[os.str()] = new Exec::TakeOff (req.ns, req.id);
		found = true;
	}
	if (type == "land") {
		execmap[os.str()] = new Exec::Land (req.ns, req.id);
		found = true;
	}
	if (type == "take-pictures") {
		execmap[os.str()] = new Exec::TakePictures (req.ns, req.id);
		found = true;
	}
	if (type == "scan-ground-single") {
		execmap[os.str()] = new Exec::ScanGroundSingle (req.ns, req.id);
		found = true;
	}
	if (type == "yaw") {
		execmap[os.str()] = new Exec::RotateYaw (req.ns, req.id);
		found = true;
	}
	if (type == "leashing") {
		execmap[os.str()] = new Exec::Leashing (req.ns, req.id);
		found = true;
	}
	/*if (type == "stop-data-stream") {   ///FUCKING IMPOSSIBLE ERROR ON THIS
		execmap[os.str()] = new Exec::StopDataStream (req.ns, req.id);
		found = true;
	}*/
	if (type == "start-data-stream") {
		execmap[os.str()] = new Exec::StartDataStream (req.ns, req.id);
		found = true;
	}
	if (type == "operator-control") {
		execmap[os.str()] = new Exec::OperatorControl (req.ns, req.id);
		found = true;
	}
	if (type == "in-air-goal") {
		execmap[os.str()] = new Exec::InAirGoal (req.ns, req.id);
		found = true;
	}
	if (type == "in-air-test") {
		execmap[os.str()] = new Exec::InAirTest (req.ns, req.id);
		found = true;
	}
	
  if (found) {
    res.success = true;
    res.error = 0;
    if (req.run_prepare) {
      bool prep = execmap[os.str()]->prepare ();
      if (!prep) {
	res.success = false;
	res.error = 2;
      }
    }
  } else {
    ROS_ERROR ("Could not create executor of type: %s", type.c_str());
    res.success = false;
    res.error = 1;
  }

  return true;
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "waspexecutor");

  ros::NodeHandle n;
  global_nh = &n;

  ros::Publisher confirm_pub = n.advertise<lrs_msgs_tst::ConfirmReq>("confirm_request", 1, true); // queue size 1 and latched
  global_confirm_pub = &confirm_pub;

  ros::Publisher cmd_pub = n.advertise<mms_msgs::Cmd>("sent_command", 10); // command topic
  global_cmd_pub = &cmd_pub;

	ros::Subscriber reference_sub;   //subscriber to reference topic
	reference_sub = n.subscribe("reference", 10, callbackReference);

  std::vector<ros::ServiceServer> services;

  std::string prefix = "tst_executor/";

  services.push_back (n.advertiseService(prefix + "destroy_executor", destroy_executor));
  services.push_back (n.advertiseService(prefix + "create_executor", create_executor));
  services.push_back (n.advertiseService(prefix + "executor_check", executor_check));
  services.push_back (n.advertiseService(prefix + "executor_continue", executor_continue));
  services.push_back (n.advertiseService(prefix + "executor_expand", executor_expand));
  services.push_back (n.advertiseService(prefix + "executor_is_delegation_expandable", executor_is_delegation_expandable));
  services.push_back (n.advertiseService(prefix + "executor_request_pause", executor_request_pause));
  services.push_back (n.advertiseService(prefix + "executor_get_constraints", executor_get_constraints));
  services.push_back (n.advertiseService(prefix + "start_executor", start_executor));
  services.push_back (n.advertiseService(prefix + "abort_executor", abort_executor));
  services.push_back (n.advertiseService(prefix + "pause_executor", pause_executor));
  services.push_back (n.advertiseService(prefix + "continue_executor", continue_executor));
  services.push_back (n.advertiseService(prefix + "enough_executor", enough_executor));

  ROS_INFO("Ready to be an executor factory");

  ros::spin();

  return 0;
}

