#include <media_manager/media_manager.h>

/////////////////////////    FUNCTION    /////////////////////////

void initialize()
{
    gazebo_check = false;
    ros::Time now = ros::Time::now();

    free_resource_flag = true;
}

bool parseMediaScript(int media_script_index)
{
    YAML::Node media_script_file_doc;
    try
    {
        //load yaml
        media_script_file_doc = YAML::LoadFile(media_script_file_path.c_str());
    }catch (const std::exception& e)
    {
        ROS_ERROR("Failed to load media script file.");
        return false;
    }

    //find media script
    std::string script_index_key    = "script" + convertIntToString(media_script_index);
    YAML::Node media_script_doc     = media_script_file_doc[script_index_key];

    if(media_script_doc == NULL)
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
            if(media_script_cmd_doc == NULL)
            {
                break;
            }

            //check validity of cmd_name
            media_script_cmd temp_cmd;
            if(media_script_cmd_doc["cmd_name"] == NULL)
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
            if(temp_cmd.cmd_name == MP4_PLAY_CMD_NAME)
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
    } catch (const std::exception& e)
    {
        std::string status_msg = "cmd#" + convertIntToString(cmd_num) + "of" + "script#" + convertIntToString(media_script_index) + "is invalid.";
        ROS_ERROR_STREAM(status_msg);
        return false;
    }

    return true;

}


/////////////////////////    CALLBACK    /////////////////////////
void mediaScriptNumberCallback(const std_msgs::Int32::ConstPtr& msg)
{
    if  ((msg->data == -1) || (msg->data == -2)) //Stop or Break
    {
        std_msgs::Int32 media_page_num_msg;
        media_page_num_msg.data = msg->data;
        
        // 영상 stop
        gst_element_set_state (pipeline, GST_STATE_NULL);
        free_resource_flag = true;
    }
    else if(msg->data == 1)
    {
        // 영상 선택
        pipeline = gst_parse_launch("playbin uri=file:///home/xavier/catkin_ws/src/HERoEHS/ABLE/HERoEHS-ABLE-Operation/able_media_manager/data/test_shark2.mp4", NULL);
       
        ROS_INFO("Video Load Finish");

        if (gst_element_get_state(pipeline, NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE)
        {
            ROS_ERROR("Failed to PLAY during preroll.");
        }
        else {
            ROS_DEBUG("Stream is PLAYING in preroll.");
        }

        gst_element_set_state (pipeline, GST_STATE_PLAYING);
        
        if(gst_element_set_state(pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE)
        {
            ROS_ERROR("Could not start stream!");
        }
        ROS_INFO("play Video");

        if(gst_element_set_state(pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE)
        {
            ROS_ERROR("Could not start stream!");    
        }
    }
    else if(msg->data == 2)
    {
        // 영상 선택
        pipeline = gst_parse_launch("playbin uri=file:///home/xavier/catkin_ws/src/HERoEHS/ABLE/HERoEHS-ABLE-Operation/able_media_manager/data/tayo_wash_hand.mp4", NULL);
       
        ROS_INFO("Video Load Finish");

        if (gst_element_get_state(pipeline, NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE)
        {
            ROS_ERROR("Failed to PLAY during preroll.");
        }
        else {
            ROS_DEBUG("Stream is PLAYING in preroll.");
        }

        gst_element_set_state (pipeline, GST_STATE_PLAYING);
        
        if(gst_element_set_state(pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE)
        {
            ROS_ERROR("Could not start stream!");
        }
        ROS_INFO("play Video");

        if(gst_element_set_state(pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE)
        {
            ROS_ERROR("Could not start stream!");    
        }
    }

    else if (msg->data == 4)
    {
        // 영상 선택
        pipeline = gst_parse_launch("playbin uri=file:///home/xavier/catkin_ws/src/HERoEHS/ABLE/HERoEHS-ABLE-Operation/able_media_manager/data/script.mp4", NULL);

        ROS_INFO("Video Load Finish");

        if (gst_element_get_state(pipeline, NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE)
        {
            ROS_ERROR("Failed to PLAY during preroll.");
        }
        else {
            ROS_DEBUG("Stream is PLAYING in preroll.");
        }

        gst_element_set_state(pipeline, GST_STATE_PLAYING);

        if (gst_element_set_state(pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE)
        {
            ROS_ERROR("Could not start stream!");
        }
        ROS_INFO("play Video");

        if (gst_element_set_state(pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE)
        {
            ROS_ERROR("Could not start stream!");
        }
    }
}

void mediaStateCallback(const std_msgs::Int32::ConstPtr& msg)
{
    if  ((msg->data == -1) || (msg->data == -2)) //Stop or Break
    {
        // 영상 stop
        gst_element_set_state (pipeline, GST_STATE_NULL);
        free_resource_flag = true;
    }
    else if(msg->data == 1)
    {
        // 영상 선택
        pipeline = gst_parse_launch("playbin uri=file:///home/xavier/catkin_ws/src/HERoEHS/ABLE/HERoEHS-ABLE-Operation/able_media_manager/data/baby_shark_video.mp4", NULL);
       
        ROS_INFO("Video Load Finish");

        if (gst_element_get_state(pipeline, NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE)
        {
            ROS_ERROR("Failed to PLAY during preroll.");
        }
        else {
            ROS_DEBUG("Stream is PLAYING in preroll.");
        }

        gst_element_set_state (pipeline, GST_STATE_PLAYING);
        
        if(gst_element_set_state(pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE)
        {
            ROS_ERROR("Could not start stream!");
        }
        ROS_INFO("play Video");

        if(gst_element_set_state(pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE)
        {
            ROS_ERROR("Could not start stream!");    
        }
    }
    else if(msg->data == 2)
    {
        // 영상 선택
        pipeline = gst_parse_launch("playbin uri=file:///home/xavier/catkin_ws/src/HERoEHS/ABLE/HERoEHS-ABLE-Operation/able_media_manager/data/tayo_wash_hand.mp4", NULL);
       
        ROS_INFO("Video Load Finish");

        if (gst_element_get_state(pipeline, NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE)
        {
            ROS_ERROR("Failed to PLAY during preroll.");
        }
        else {
            ROS_DEBUG("Stream is PLAYING in preroll.");
        }

        gst_element_set_state (pipeline, GST_STATE_PLAYING);
        
        if(gst_element_set_state(pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE)
        {
            ROS_ERROR("Could not start stream!");
        }
        ROS_INFO("play Video");

        if(gst_element_set_state(pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE)
        {
            ROS_ERROR("Could not start stream!");    
        }
    }
    else if(msg->data == 3)
    {
        // 영상 선택
        pipeline = gst_parse_launch("playbin uri=file:///home/xavier/catkin_ws/src/HERoEHS/ABLE/HERoEHS-ABLE-Operation/able_media_manager/data/baby_shark_hand_wash.mp4", NULL);
       
        ROS_INFO("Video Load Finish");

        if (gst_element_get_state(pipeline, NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE)
        {
            ROS_ERROR("Failed to PLAY during preroll.");
        }
        else {
            ROS_DEBUG("Stream is PLAYING in preroll.");
        }

        gst_element_set_state (pipeline, GST_STATE_PLAYING);
        
        if(gst_element_set_state(pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE)
        {
            ROS_ERROR("Could not start stream!");
        }
        ROS_INFO("play Video");

        if(gst_element_set_state(pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE)
        {
            ROS_ERROR("Could not start stream!");    
        }
    }

    else if (msg->data == 4)
    {
        // 영상 선택
        pipeline = gst_parse_launch("playbin uri=file:///home/xavier/catkin_ws/src/HERoEHS/ABLE/HERoEHS-ABLE-Operation/able_media_manager/data/script.mp4", NULL);

        ROS_INFO("Video Load Finish");

        if (gst_element_get_state(pipeline, NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE)
        {
            ROS_ERROR("Failed to PLAY during preroll.");
        }
        else {
            ROS_DEBUG("Stream is PLAYING in preroll.");
        }

        gst_element_set_state(pipeline, GST_STATE_PLAYING);

        if (gst_element_set_state(pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE)
        {
            ROS_ERROR("Could not start stream!");
        }
        ROS_INFO("play Video");

        if (gst_element_set_state(pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE)
        {
            ROS_ERROR("Could not start stream!");
        }
    }
}
/////////////////////////    MAIN    /////////////////////////

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motion_manager_node");
    ros::NodeHandle nh;

    media_state_sub         = nh.subscribe("/heroehs/media_state", 0, &mediaStateCallback);
    media_script_num_sub    = nh.subscribe("/heroehs/media_script_num", 0, &mediaScriptNumberCallback);
    media_script_num_pub    = nh.advertise<std_msgs::Int32>("/heroehs/media/info", 0);

    /* Initialize GStreamer */
    gst_init(&argc, &argv);

    // 영상 선택
    //pipeline = gst_parse_launch("playbin uri=file:///home/jun/catkin_ws/src/HERoEHS/ABLE/HERoEHS-ABLE-Operation/able_media_manager/data/test_shark2.mp4", NULL);
       

    //setting media script file path
    std::string temp_media_script_file_path = ros::package::getPath("able_media_manager") + "/script/able_media_script.yaml";
    if  (nh.getParam("media_script_file_path", media_script_file_path) == false)
    {
        media_script_file_path = temp_media_script_file_path;
        ROS_WARN("Failed to get media script file's path.");
        ROS_WARN("The default media script file path will be used.");
    }

    ROS_INFO("Start ABLE Media Script Manager");

    /* Build the pipeline */
    //pipeline = gst_parse_launch("playbin uri=file:///home/jun/catkin_ws/src/HERoEHS/ABLE/HERoEHS-ABLE-Operation/able_media_manager/data/test_shark2.mp4", NULL);

    bus = gst_element_get_bus(pipeline);
    msg = gst_bus_timed_pop_filtered(bus, GST_CLOCK_TIME_NONE,(GstMessageType)(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));


    if(free_resource_flag)
    {
        /* Free resources */
        if (msg != NULL)
            gst_message_unref(msg);
        gst_object_unref(bus);
        gst_element_set_state(pipeline, GST_STATE_NULL);
        gst_object_unref(pipeline);

        free_resource_flag = false;
    }

    ros::spin();
}