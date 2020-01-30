#include "ros/ros.h"
#include "std_msgs/Float64.h"

#include <sstream>

#include <stdlib.h>       // exit
#include <iostream>       // std::cout, std::endl
#include <thread>         // std::this_thread::sleep_for
#include <chrono>         // std::chrono::seconds
#define _USE_MATH_DEFINES
#include <math.h>
#include <vector>


using namespace std; 

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
g++ move_robot.cpp -o execute_move_robot -I/opt/ros/melodic/include -L/opt/ros/melodic/lib -lroscpp -lrostime -lrosconsole -lroscpp_serialization
 */


static bool arm_left_front_joint_1_expand ;
static bool arm_left_front_joint_2_expand ;
static bool arm_left_front_joint_3_expand ;

#define RADIAN_STEP M_PI/24


class Robot { 
    public: 
    string robot_name; 
      
    //Default Constructor 
    Robot() { 
        cout << "Default Constructor called" << endl;  
        robot_name='\0'; 
    } 
      
    //Parametrized Constructor 
    Robot(int argc, char **argv , string node_name ) {
	 
	std::cout << "Before init!" << std::endl;
	ros::init(argc, argv, node_name );
	ros::NodeHandle nh;

        robot_name=node_name; 
	Limb left_front( "left_front" , nh , "/my_spot/joint1_position_controller_left_front/command"
					   , "/my_spot/joint2_position_controller_left_front/command"
					   , "/my_spot/joint3_position_controller_left_front/command");
	Limb  left_rear( "left_rear" , nh  , "/my_spot/joint1_position_controller_left_rear/command"
					   , "/my_spot/joint2_position_controller_left_rear/command"
					   , "/my_spot/joint3_position_controller_left_rear/command");

	Limb right_front( "right_front" , nh, "/my_spot/joint1_position_controller_right_front/command"
					   , "/my_spot/joint2_position_controller_right_front/command"
					   , "/my_spot/joint3_position_controller_right_front/command");
	Limb  right_rear( "right_rear" , nh, "/my_spot/joint1_position_controller_right_rear/command"
					   , "/my_spot/joint2_position_controller_right_rear/command"
					   , "/my_spot/joint3_position_controller_right_rear/command");

	ros::Rate loop_rate(1000);

	int count = 0;
	bool is_left_side = true ;
	bool is_rear = true ;
	while (ros::ok())
	{
	  /**
	   * This is a message object. You stuff it with data, and then publish it.
	   */
	  std_msgs::Float64 msg;
	  ROS_INFO_STREAM("Msg: " << "InLoop  " << count);

	  //M_PI (3.1415926535897931/6) ; 

	  /**
	   * The publish() function is how you send messages. The parameter
	   * is the message object. The type of this object must agree with the type
	   * given as a template parameter to the advertise<>() call, as was done
	   * in the constructor above.
	   */
	if( count < 19 ){
	  left_front.full_anterior_extend_limb(  is_left_side = true  , is_rear = false );
	  right_front.full_anterior_extend_limb( is_left_side = false , is_rear = false );
	  left_rear.full_anterior_extend_limb(   is_left_side = true  , is_rear = true );
	  right_rear.full_anterior_extend_limb(  is_left_side = false , is_rear = true );

	     	  std::this_thread::sleep_for (std::chrono::seconds(1));
	}else if ( count < 25 ){
	  //left_front.full_cranial_extend_limb(   is_left_side = true );
	  right_front.full_cranial_extend_limb(  is_left_side = false , is_rear = false );
	  //   	  std::this_thread::sleep_for (std::chrono::seconds(3));
	}else if ( count < 38 ){
	  //left_front.full_cranial_extend_limb(   is_left_side = true );
	  //left_rear.full_cranial_extend_limb(  is_left_side = true , is_rear = true );
	  //   	  std::this_thread::sleep_for (std::chrono::seconds(3));
	} else { exit( 0 );}
	  ros::spinOnce();


	  ++count;
	}
	
    } 
    
    private:
	class Limb { 
	    public: 
		string name; 
		ros::Publisher shoulder_pub ; //arm_left_front_joint_1_pub
		ros::Publisher upperarm_pub ; //arm_left_front_joint_2_pub
		ros::Publisher  forearm_pub ; //arm_left_front_joint_3_pub
		std_msgs::Float64 shoulder_value ;
		std_msgs::Float64 upperarm_value ;
		std_msgs::Float64  forearm_value ; 

		bool full_anterior_extended = false ;           
		bool full_cranial_extended = false ;  
		bool full_lateral_extended = false ;   
	    //Default Constructor 
	    Limb() 
	    { 
		cout << "Default Constructor called" << endl;  
		name = '\0' ; 
	    } 
	      
	    //Parametrized Constructor 
	    Limb( string limb_name , 	ros::NodeHandle nh , const string j1cont , const string j2cont , const string j3cont) { 
		cout << "Parametrized Constructor called" << endl; 
		name = limb_name ; 
		shoulder_value.data = 0 ;
		upperarm_value.data = 0 ;
		forearm_value.data = 0 ;

		shoulder_pub = nh.advertise<std_msgs::Float64>(j1cont, 1000);
		upperarm_pub = nh.advertise<std_msgs::Float64>(j2cont, 1000);
		forearm_pub  = nh.advertise<std_msgs::Float64>(j3cont, 1000);
/*
		shoulder_pub = nh.advertise<std_msgs::Float64>("/my_spot/joint1_position_controller_left_front/command", 1000);
		upperarm_pub = nh.advertise<std_msgs::Float64>("/my_spot/joint2_position_controller_left_front/command", 1000);
		forearm_pub  = nh.advertise<std_msgs::Float64>("/my_spot/joint3_position_controller_left_front/command", 1000);
*/
	    } 


	    bool full_anterior_extend_limb(  bool is_left_side , bool is_rear ) ;
	    bool full_cranial_extend_limb(   bool is_left_side , bool is_rear ) ;
	    bool full_lateral_extend_limb(   bool is_left_side , bool is_rear ) ;
	}; 


};




bool Robot::Limb::full_anterior_extend_limb( bool is_left_side , bool is_rear ){
	
	if( is_left_side && this->shoulder_value.data > -1*M_PI/8 ){ 
	  this->shoulder_value.data -=RADIAN_STEP ;
	} else if (!is_left_side && this->shoulder_value.data < 1*M_PI/8){
	  this->shoulder_value.data -=RADIAN_STEP ;
	}	
	  this->shoulder_value.data = 0 ;
	  this->shoulder_pub.publish( this->shoulder_value );

	  ROS_INFO_STREAM(""<<this->name << ".shoulder_value : " << this->shoulder_value);
	if( ( !is_rear&&(is_left_side && this->upperarm_value.data > -1*M_PI/2+RADIAN_STEP*3) )
		|| ( is_rear&&(is_left_side && this->upperarm_value.data > -1*M_PI/2+RADIAN_STEP*6) ) ){ 
	  this->upperarm_value.data -=RADIAN_STEP;//= -1*M_PI/2 ;
	} else if ( ( !is_rear&&(!is_left_side && this->upperarm_value.data < 1*M_PI/2-RADIAN_STEP*3) ) 
		|| ( is_rear&&(!is_left_side && this->upperarm_value.data < 1*M_PI/2-RADIAN_STEP*6) ) ){
	  this->upperarm_value.data +=RADIAN_STEP;//= 1*M_PI/2 ;
	}
	  this->upperarm_pub.publish( this->upperarm_value );
	  ROS_INFO_STREAM(""<<this->name << ".upperarm_value : " << this->upperarm_value);

	if( is_left_side && this->forearm_value.data < 2*M_PI/3-RADIAN_STEP*3 ){ 
	  this->forearm_value.data +=RADIAN_STEP;//= M_PI ;
	}else if (!is_left_side && this->upperarm_value.data > -1*2*M_PI/3+RADIAN_STEP*3){
	  this->forearm_value.data -=RADIAN_STEP;//= 1*M_PI/2 ;
	}	
	  this->forearm_pub.publish( this->forearm_value );
	  ROS_INFO_STREAM(""<<this->name << ".forearm_value : " << this->forearm_value);

	  this->full_anterior_extended = true ;


	return true;
}


bool Robot::Limb::full_cranial_extend_limb( bool is_left_side , bool is_rear ){

	  this->shoulder_value.data = 0 ;
	  this->shoulder_pub.publish( this->shoulder_value );
	  ROS_INFO_STREAM(""<<this->name << ".shoulder_value : " << this->shoulder_value);

	  this->upperarm_value.data = -1*M_PI ;
	  this->upperarm_pub.publish( this->upperarm_value );
	  ROS_INFO_STREAM(""<<this->name << ".upperarm_value : " << this->upperarm_value);

	  this->forearm_value.data = M_PI ;
	  this->forearm_pub.publish( this->forearm_value );
	  ROS_INFO_STREAM(""<<this->name << ".forearm_value : " << this->forearm_value);
	  std::this_thread::sleep_for (std::chrono::seconds(1));

	  this->full_anterior_extended = true ;


	return true;
}












int main(int argc, char **argv){
	Robot my_robot( argc, argv , "my_spot" );

	return 0;
}















