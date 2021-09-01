#include <ros/ros.h>
#include <ros/package.h>

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <yaml-cpp/yaml.h>

//ros_communication_message type
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>


#include <vlc/vlc.h>


#define MP4_PLAY_CMD_NAME                   "mp4"
#define WAIT_ACTION_PLAY_FINISH_CMD_NAME    "wait"
#define SLEEP_CMD_NAME                      "sleep"

//ros communication
ros::Subscriber media_script_num_sub;
ros::Subscriber media_state_sub;
ros::Publisher  media_script_num_pub;

//ros msg

//variables
ros::Time count;

bool gazebo_check;

//Yaml
std::string media_script_file_path;

typedef struct
{
    std::string cmd_name;
    std::string cmd_arg_str;
    int         cmd_arg_int;
} media_script_cmd;

std::string convertIntToString(int num)
{
    std::ostringstream ostr;
    ostr << num;
    return ostr.str();
}

//GStreamer
// GstElement* pipeline;
// GstBus* bus;
// GstMessage* msg;
libvlc_instance_t * inst;
libvlc_media_player_t *mp;
libvlc_media_t *m;
std::string location;


//resource free flag
bool check_running_need;
bool check_pause;

//function
void initialize();
bool parseMediaScript(int media_script_index);

//callback
void mediaScriptNumberCallback  (const std_msgs::Int32::ConstPtr& msg);
void mediaStateCallback         (const std_msgs::Int32::ConstPtr& msg);