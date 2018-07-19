/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandBool.h>
#include <cstdio>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

char getch()
{
    int flags = fcntl(0, F_GETFL, 0);
    fcntl(0, F_SETFL, flags | O_NONBLOCK);

    char buf = 0;
    struct termios old = {0};
    if (tcgetattr(0, &old) < 0) {
        perror("tcsetattr()");
    }
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(0, TCSANOW, &old) < 0) {
        perror("tcsetattr ICANON");
    }
    if (read(0, &buf, 1) < 0) {
        //perror ("read()");
    }
    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(0, TCSADRAIN, &old) < 0) {
        perror ("tcsetattr ~ICANON");
    }
    return (buf);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::ServiceClient takeoff_cl = nh.serviceClient<mavros_msgs::CommandTOL>
            ("/mavros/cmd/takeoff");
    ros::ServiceClient land_cl = nh.serviceClient<mavros_msgs::CommandTOL>
            ("/mavros/cmd/land");
   
    mavros_msgs::CommandTOL srv_takeoff;
    mavros_msgs::CommandTOL srv_land;
    
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(30.0);
    
    
    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

   

    system("rosrun mavros mavwp load /home/$USER/flightplan/ardu.txt");

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "GUIDED";
    set_mode_client.call(offb_set_mode);

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    arming_client.call(arm_cmd);

    if(!current_state.armed){
            arming_client.call(arm_cmd);
            ROS_INFO("ARMED");
        }

    system("rosrun mavros mavcmd takeoff 0 1 -35.362743 149.164932 10");
    sleep(20);

    offb_set_mode.request.custom_mode = "AUTO";
    set_mode_client.call(offb_set_mode);

    while(ros::ok()){
        int c = getch();
        
        
        if(!current_state.armed){
            arming_client.call(arm_cmd);
            ROS_INFO("ARMED");
        }
     
        ros::spinOnce();

        rate.sleep();

        
    }

    

    return 0;
}