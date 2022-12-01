/*!
* \file robot.cpp
* \brief This file contains a C++ program which represents the robot executing its behaviours based on these tasks (1.Autonomously reach a User entering Co-ordinates X and Y and, 2. Control or Move the robot using keyboard keys.)
* \author NIRMAL KUMAR RAVIKUMAR 5168055
* \date 26/09/2022
*
* \details
*
* Subscribes to: <BR>
* /move_base/feedback initial_position 
* /move_base/initial goal_goal 
* /scan assistance
* 
* Publishes to: <BR>
* /move_base/goal
* /move_base/cancel
* /cmd_vel
*
* Description :
* By this robot.cpp file we have the above mentioned  3 publishers and subscribers, which are declared position of the goal, goal coordinates and laser scanner for manual driving assistance respectively. 
* All these subscribers are run simultaneously by multithreading concept.
* So by these the robot can:Reach the goal autonomously -User enters the x and y coordinates where the robot should go.Publishes the goal enters to move_base/goal.
* User control using keybaord User can control the robot using the list of commands (keyboard keys) and move accordingly. 
*
**/


#include <iostream>
#include <string>
#include <chrono>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "actionlib_msgs/GoalID.h"
#include "move_base_msgs/MoveBaseActionGoal.h"
#include "move_base_msgs/MoveBaseActionFeedback.h"


ros::Publisher pub1; ///<Publishes the Move Base Action Goal for the robot to reach the goal for the given coordinates 
ros::Publisher pub2; ///<Publishes the Move Base cancel Goal for the robot to avoid the robot movement beyond the environment
ros::Publisher pub3; ///<Publishes the command the velocity for the robot  


float vel_linear = 0.0;   ///linear velocity of the robot
float vel_angular = 0.0;  ///angular velocity of the robot
char input;   ///user input on the key board

int flag1 = 0; ///assistance enable/diable
int flag2 = 0; ///time taken after current goal request

int flag3 = 0;
int flag4 = 0; 
int counter1 = 10; 
int counter2; 

float x_co_ord; ///x coordinate of the current goal
float y_co_ord; ///y coordinate of the current goal

std::string id = ""; 
std::chrono::high_resolution_clock::time_point t_start;  
std::chrono::high_resolution_clock::time_point t_end; 

#define DIST 0.30          ///minimum distance from the wall 
#define POS_ERROR 0.5      ///position range error
#define MAX_TIME 100000000 ///maximum time in order to reach the goal 

/*!
* \brief TAKE THE WHEEL - manual control with enable/disable option of driving assistance.
* \param msg Robot speed and control
* take the wheel -This function  allows the user to give their input in such a way that the robot can be moved correspondingly with respective keys in the keyboard.
* And as per the input keys, the robots linear angular velocities are changed
*/

void take_the_wheel() {
    
    geometry_msgs::Twist robot_speed;
    counter2 = 10;
    input = 'e';

    
    printf("\n--- Manual Control ---\n\n"
    "preferred Linear velocity : 0.5\n"
    "preferred Angular velocity : 1.0\n");

    while (input != 'f') {
        
        if (counter2 % 10 == 0) {
            printf("\n   Commands:   \n"
            "w : Go on\n"
            "s : Go back\n"
            "q : Curve left\n"
            "e : Curve right\n"
            "a : Turn left\n"
            "d : Turn right\n"
            "-----------------------------\n"
            "r : Increase linear velocity\n"
            "t : Decrease linear velocity\n"
            "y : Increase angular velocity\n"
            "u : Decrease angular velocity\n"
            "-----------------------------\n"
            "i : Emergency stop\n"
            "h : Enable/disable assistance\n"
            "p : Quit\n");
        }

        if (flag3 == 0) 
            printf("h - Enable driving assistance\n");
        else if (flag3 == 1)
            printf("h - Disable driving assistance\n");

        
        printf("\nCommand: ");
        std::cin >> input;

        flag4 = 0;
        
        switch (input)
        {
        case 'w':
                robot_speed.linear.x = vel_linear;
                robot_speed.angular.z = 0;
                break;
        
        case 'q':  
                robot_speed.linear.x = vel_linear;
                robot_speed.angular.z = vel_angular;
                break; 
        
        case 's':  
                robot_speed.linear.x = -vel_linear;
                robot_speed.angular.z = 0; 
                break;
        
        case 'e':  
                robot_speed.linear.x = vel_linear;
                robot_speed.angular.z = -vel_angular;
                break; 
        
        case 'a':  
                robot_speed.linear.x = 0;
                robot_speed.angular.z = vel_angular;
                break; 
        
        case 'd':  
                robot_speed.linear.x = 0;
                robot_speed.angular.z = -vel_angular; 
                break;
        
        case 'r':  
                vel_linear += 0.1;
                break;
        case 't':  
                vel_linear -= 0.1;
                break;
        case 'y':  
                vel_angular += 0.1;
                break;
        case 'u':  
                vel_angular -= 0.1;
                break;
        case 'i':  
                robot_speed.linear.x = 0;
                robot_speed.angular.z = 0; 
                break;
        case 'o':  
                if (flag3 == 0) 
                {
                    flag1 = 1;
                    flag3 = 1;
                }
                else if (flag3 == 1)
                {
                    flag1 = 0;
                    flag3 = 0;
                }
                break;
        case 'p':  
                robot_speed.linear.x = 0;
                robot_speed.angular.z = 0; 
                pub3.publish(robot_speed);
                counter1 = 10;
                break;
        }
        
        
        printf("Linear velocity: %f\n"
        "Angular velocity: %f\n", vel_linear, vel_angular);
        pub3.publish(robot_speed);
        counter2++;
    }
}

/*!
* \brief Assistance - robot can be able to scan the path using the laser scan.
* \param msg Robot speed and assistance
* ASSISTANCE-In this function, with help of the geometry_msg the robot can able to scan the path way to move freely by dividing it into three parts left, right and center.
*/

void assistance(const sensor_msgs::LaserScan::ConstPtr& msg) {
    
    geometry_msgs::Twist robot_speed;
    float left = 35.0;
    float center = 35.0;
    float right = 35.0;
    int i;
    
    
    for (i = 0; i < 360; i++) { 
        if (msg->ranges[i] < right)
            right = msg->ranges[i];
    }
    for (i = 300; i < 420; i++) { 
        if (msg->ranges[i] < center)
            center = msg->ranges[i];
    }
    for (i = 360; i < 720; i++) { 
        if (msg->ranges[i] < left)
            left = msg->ranges[i];
    }

    
    if (flag1== 1 & ((center < DIST && input == 'w') || (left < DIST && input == 'q') || (right < DIST && input == 'e'))) {
        if (flag4 == 0) {
            printf("\nALERT! - Wall approached\n");
            flag4 = 1;
        }
        robot_speed.linear.x = 0;
        robot_speed.angular.z = 0;
        pub3.publish(robot_speed);
    }

    
    if (flag2 == 1) {
        t_end = std::chrono::high_resolution_clock::now();
        auto time = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start).count();
        if (time > MAX_TIME) {
            actionlib_msgs::GoalID canc_goal;
            printf("\nCannot reach goal!\n");
            canc_goal.id = id;
            pub2.publish(canc_goal);
            printf("Goal cancelled.\n");
            flag2 = 0;
        }
    }   
}


/*!
* \brief initial position of the robot.
* \param msg MoveBaseActionFeedback
* INITIAL POSITION-In this function the Initial_position is used to calculate the difference between initial position of the robot and the go to position of the robot in terms of x and y coordinates .
* when the difference of x and y cords are not negative, then the entered user coordinates are sent as feedback for the robot to move to the desired position.
*/


void initial_position(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& msg) {
    
    float diff_x;
    float diff_y;
    float initial_x = msg->feedback.base_position.pose.position.x;
    float initial_y = msg->feedback.base_position.pose.position.y;

    
    if (initial_x < 0)
        initial_x *= -1;
    if (initial_y < 0)
        initial_y *= -1;
    
    
    if (initial_x >= x_co_ord)
        diff_x = initial_x - x_co_ord;
    else 
        diff_x = x_co_ord - initial_x;
    if (initial_y >= y_co_ord)
        diff_y = initial_y - y_co_ord;
    else 
        diff_y = y_co_ord - initial_y;

    
    if (diff_x <= POS_ERROR && diff_y <= POS_ERROR)
        flag2 = 0;

    
    if (id != msg->status.goal_id.id) {
        flag2 = 1;
        id = msg->status.goal_id.id;
        t_start = std::chrono::high_resolution_clock::now();
    }
}

/*!
* \brief INITIAL GOAL.
* \param msg MoveBaseActionFeedGoal
* INITIAL POSITION-Initial_goal - This function is used to assign the x and y go-to coordinates  of the robot.
*/

void initial_goal(const move_base_msgs::MoveBaseActionGoal::ConstPtr& msg) {
    x_co_ord = msg->goal.target_pose.pose.position.x;
    y_co_ord = msg->goal.target_pose.pose.position.y;

    
    if (x_co_ord < 0)
        x_co_ord *= -1;
    if (y_co_ord < 0)
        y_co_ord *= -1;
}

/*!
* \brief UI.
* ui - This is the function used to receive the choice of an action  the user want the the robot to operate by pressing respective keys for corresponding operations.
*/

void ui() {
    
    move_base_msgs::MoveBaseActionGoal goal_pos;
    actionlib_msgs::GoalID canc_goal;
    std::string X, Y;
    double x, y;
    char in;

    while (in != '0') {
        
        if (counter1 % 10 == 0) {
            printf("\nChoose an action:\n"
            "0 - Exit\n"
            "1 - Insert new coordinates to reach\n"
            "2 - Cancel the initial goal\n"
            "3 - Manual driving\n");
        }
        if (flag3 == 0) 
            printf("4 - Enable assistance\n");
        else if (flag3 == 1)
            printf("4 - Disable assistance\n");
        
        
        printf("Please Enter your choice of Action: ");
        std::cin >> in;

        
        if (in != '0' && in != '1' && in != '2' && in != '3' && in != '4') 
            printf("\nERROR: type '0', '1', '2', '3' or '4'.\n");

        counter1++;

        
        if (in == '0') {
            flag2 = 0;
            canc_goal.id = id;
            pub2.publish(canc_goal);
        }

        
        else if (in == '1') {
            
            printf("\nInsert coordinates to reach:\n");
            printf("X: ");
            std::cin >> X;
            printf("Y: ");
            std::cin >> Y;

            x = atof(X.c_str());
            y = atof(Y.c_str());

            
            goal_pos.goal.target_pose.header.frame_id = "map";
            goal_pos.goal.target_pose.pose.orientation.w = 1;
            
            goal_pos.goal.target_pose.pose.position.x = x;
            goal_pos.goal.target_pose.pose.position.y = y;

             
            pub1.publish(goal_pos);
        }
        
        
        else if (in == '2') {
            canc_goal.id = id;
            pub2.publish(canc_goal);
            printf("Goal cancelled.\n");
        }

        
        else if (in == '3') {
            flag2 = 0;
            canc_goal.id = in;
            pub2.publish(canc_goal);
            take_the_wheel();
        }

        
        else if (in == '4') {
            if (flag3 == 0) {
                flag1 = 1;
                flag3 = 1;
                printf("\n assistance enabled.\n");
            }
            else if (flag3 == 1) {
                flag1 = 0;
                flag3 = 0;
                printf("\n assistance disabled.\n");
            }
        }
    }
}

int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "finalrobot");
    ros::NodeHandle nh;
    
    
    pub1 = nh.advertise<move_base_msgs::MoveBaseActionGoal>("move_base/goal", 1000);
    pub2 = nh.advertise<actionlib_msgs::GoalID>("move_base/cancel", 1000);
    pub3 = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    
    
    ros::Subscriber sub_pos = nh.subscribe("/move_base/feedback", 1000, initial_position); 
    ros::Subscriber sub_goal = nh.subscribe("/move_base/goal", 1000, initial_goal); 
    ros::Subscriber sub_laser = nh.subscribe("/scan", 1000, assistance); 

    
    ros::AsyncSpinner spinner(3);
    spinner.start();
    ui();
    spinner.stop();
    ros::shutdown();
    ros::waitForShutdown();

    return 0;
}
