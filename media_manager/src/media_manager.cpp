#include <media_manager/media_manager.h>

/////////////////////////    FUNCTION    /////////////////////////

// Example https://wiki.videolan.org/LibVLC_Tutorial/
// sudo apt-get install libvlc-dev  

void initialize()
{
  gazebo_check = false;
  ros::Time now = ros::Time::now();

  check_running_need = false;
  check_pause  =false;
}

bool parseMediaScript(int media_script_index)
{
  YAML::Node media_script_file_doc;
  try
  {
    //load yaml
    media_script_file_doc = YAML::LoadFile(media_script_file_path.c_str());
  }
  catch (const std::exception &e)
  {
    ROS_ERROR("Failed to load media script file.");
    return false;
  }

  //find media script
  std::string script_index_key = "script" + convertIntToString(media_script_index);
  YAML::Node media_script_doc = media_script_file_doc[script_index_key];

  if (media_script_doc == NULL)
  {
    std::string status_msg = "Failed to find media script #" + convertIntToString(media_script_index);
    ROS_ERROR_STREAM(status_msg);
    return false;
  }

  int cmd_num = 1;
  std::string cmd_key = "";
  try
  {
    while (true)
    {
      cmd_key = "cmd" + convertIntToString(cmd_num);
      YAML::Node media_script_cmd_doc = media_script_doc[cmd_key];
      if (media_script_cmd_doc == NULL)
      {
        break;
      }

      //check validity of cmd_name
      media_script_cmd temp_cmd;
      if (media_script_cmd_doc["cmd_name"] == NULL)
      {
        std::string status_msg = "cmd#" + convertIntToString(cmd_num) + "of" + "script#" + convertIntToString(media_script_index) + " is invalid.";
        ROS_ERROR_STREAM(status_msg);
        return false;
      }

      //check validity of cmd_arg
      temp_cmd.cmd_name = media_script_cmd_doc["cmd_name"].as<std::string>();
      if ((temp_cmd.cmd_name != "wait") && (media_script_cmd_doc["cmd_arg"] == NULL))
      {
        std::string status_msg = "cmd#" + convertIntToString(cmd_num) + "of" + "script#" + convertIntToString(media_script_index) + "is invalid.";
        ROS_ERROR_STREAM(status_msg);
        return false;
      }

      //get cmd_arg
      if (temp_cmd.cmd_name == MP4_PLAY_CMD_NAME)
      {
        temp_cmd.cmd_arg_str = media_script_cmd_doc["cmd_arg"].as<std::string>();
      }
      else if (temp_cmd.cmd_name == WAIT_ACTION_PLAY_FINISH_CMD_NAME)
      {
        temp_cmd.cmd_arg_str = "";
        temp_cmd.cmd_arg_int = 0;
      }
      else if (temp_cmd.cmd_name == SLEEP_CMD_NAME)
      {
        temp_cmd.cmd_arg_int = media_script_cmd_doc["cmd_arg"].as<int>();
        if (temp_cmd.cmd_arg_int < 0)
        {
          std::string status_msg = "cmd#" + convertIntToString(cmd_num) + " of " + "script#" + convertIntToString(media_script_index) + " is invalid.";
          ROS_ERROR_STREAM(status_msg);
          return false;
        }
      }
      else
      {
        std::string status_msg = "cmd#" + convertIntToString(cmd_num) + " of " + "script#" + convertIntToString(media_script_index) + " is invalid.";
        ROS_ERROR_STREAM(status_msg);
        return false;
      }

      cmd_num++;
    }
  }
  catch (const std::exception &e)
  {
    std::string status_msg = "cmd#" + convertIntToString(cmd_num) + "of" + "script#" + convertIntToString(media_script_index) + "is invalid.";
    ROS_ERROR_STREAM(status_msg);
    return false;
  }

  return true;
}

/////////////////////////    CALLBACK    /////////////////////////
void mediaScriptNumberCallback(const std_msgs::Int32::ConstPtr &msg)
{
  if ((msg->data == -1)) //Stop
  {
    std_msgs::Int32 media_page_num_msg;
    media_page_num_msg.data = msg->data;

    libvlc_media_player_stop(mp);
    libvlc_media_player_release(mp);

    check_running_need = false;
    check_pause =false;
  }
  if ((msg->data == -2)) //Break
  {
    std_msgs::Int32 media_page_num_msg;
    media_page_num_msg.data = msg->data;
    if(check_running_need)  
    {

      libvlc_media_player_pause(mp);
      // libvlc_media_player_release(mp);
      check_pause = !check_pause;
    }

  }

  else if(check_running_need)
  {
    return;
  }

  else if (msg->data == 1)
  {

    // 영상 선택
    location = "file:///home/ijm20/catkin_ws/src/media_manager_for_HERoEHS/media_manager/data/test_shark.mp4";
    m = libvlc_media_new_location(inst, location.c_str());
    mp = libvlc_media_player_new_from_media(m);
    libvlc_media_release(m);
    libvlc_set_fullscreen(mp, true);
    libvlc_media_player_play(mp);
    check_running_need = true;
    check_pause =false;

  }
  else if (msg->data == 2)
  {
  }
  else if (msg->data == 3)
  {
  }
}

void mediaStateCallback(const std_msgs::Int32::ConstPtr &msg)
{
  if ((msg->data == -1) || (msg->data == -2)) //Stop or Break
  {
  }
  else if (msg->data == 1)
  {
  }
  else if (msg->data == 2)
  {
  }
  else if (msg->data == 3)
  {
  }
}
/////////////////////////    MAIN    /////////////////////////

int main(int argc, char **argv)
{
  ros::init(argc, argv, "motion_manager_node");
  ros::NodeHandle nh;
 ros::Rate loop_rate(10);

  // initialize vlc
  inst = libvlc_new(0, NULL);

  location = "";
  m = libvlc_media_new_location(inst, location.c_str());
  mp = libvlc_media_player_new_from_media(m);

  media_state_sub = nh.subscribe("/heroehs/media_state", 0, &mediaStateCallback);
  media_script_num_sub = nh.subscribe("/heroehs/media_script_num", 0, &mediaScriptNumberCallback);
  media_script_num_pub = nh.advertise<std_msgs::Int32>("/heroehs/media/info", 0);

  //setting media script file path
  std::string temp_media_script_file_path = ros::package::getPath("media_manager") + "/script/media_script.yaml";
  if (nh.getParam("media_script_file_path", media_script_file_path) == false)
  {
    media_script_file_path = temp_media_script_file_path;
    ROS_WARN("Failed to get media script file's path.");
    ROS_WARN("The default media script file path will be used.");
  }

  ROS_INFO("Start Media Script Manager");

  while (ros::ok())
  {
    int playing = libvlc_media_player_is_playing(mp);
    ROS_INFO("%d", playing);

    if (check_running_need && !check_pause)
    {
      if (!libvlc_media_player_is_playing(mp))
      {

        libvlc_media_player_stop(mp);
        libvlc_media_player_release(mp);

        check_running_need = false;
        check_pause = false;
      }
    }

    ros::spinOnce();
    loop_rate.sleep();

  }

  libvlc_media_player_stop(mp);
  libvlc_media_player_release(mp);

  libvlc_release(inst);
}
