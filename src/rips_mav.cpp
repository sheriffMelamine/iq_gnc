//Code developed for autonomous navigation of sound source localization project

/*
* Install iq_sim and iq_gnc according to the tutorials
* Copy this file to src/iq_gnc/
* Edit CMakeslist accordingly
* Run Catkin Build
* Run sitl and launch mavros related programs
* In a terminal put the following command (in catkin ws)
  > rosrun iq_gnc rips_mav
* In sitl console change the ardupilot to GUIDED mode with the following command
  > mode GUIDED
* Open another terminal
* Publish command with following command format
  > rostopic pub /ripscom std_msgs/Float64MultiArray "{layout: {dim: [] , data_offset: 0 }, data: [0.2,0.2,-1.]}"
* the brackets after 'data: ' needs to be modified for navigation
* the drone will return back and stop the program if you put the last number of the array as a non-negative number
* otherwise the 3 numbers define direction in cartersian coordinates
*/

#include <gnc_functions.hpp>
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/String.h"
 
class Rips{
private:	
	gnc_api_waypoint wp;
	float altitude;
	float speed;
	float tolerance;
	float radius;
	bool flag;
	bool landmode;
	bool camsw;
	
		
public: 
	bool picam;
	Rips(float i=2.,float j=.5,float t=0.3,float r=2.):altitude(i),speed(j),tolerance(t),radius(r),flag(false),landmode(false),wp{0,0,i,0}{}
	void callback(const std_msgs::Float64MultiArray msg);
	float get_speed();
	float get_altitude();
	void move();
};

void Rips::callback(const std_msgs::Float64MultiArray msg)
{
	std_msgs::Float64 a,b,c;
	float temp_x,temp_y;
	a.data = msg.data[0];
	b.data = msg.data[1];
	c.data = msg.data[2];
	if(c.data>=0.)
	{
		this->wp.x= 0.;
		this->wp.y= 0.;
		this->landmode = true;
		this->flag=true;
		
	}
	else
	{
		temp_x = this->altitude*a.data/c.data;
		temp_y = this->altitude*b.data/c.data;
		if(temp_x*temp_x+temp_y*temp_y<altitude*radius*radius)
		{
			this->wp.x+= temp_x;
			this->wp.y+= temp_y;
			this->flag=true;
			this->camsw=true;
		}
	}
	
}

float Rips::get_speed()
{
	return this->speed;
}

float Rips::get_altitude()
{
	return this->altitude;
}

void Rips::move()
{
	if(check_waypoint_reached(this->tolerance))
	
		if(this->flag) 
		{
			
			this->flag=false;
			set_destination(this->wp.x,this->wp.y,this->wp.z,this->wp.psi);
		}
		else if(this->landmode)
		{
			land();
			exit(0);
		}
		else if(this->camsw)
		{
			this->camsw=false;
			this->picam=true;
		}
}

int main(int argc, char** argv)
{
	Rips rips;
	int count=0;
	ros::init(argc, argv, "rips_mav");
	ros::NodeHandle gnc_node;
	ros::Subscriber sub = gnc_node.subscribe("/ripscom", 100, &Rips::callback,&rips);
	ros::Publisher pub = gnc_node.advertise<std_msgs::String>("/str_topic", 100);
	ros::Rate rate(2.0);
	
	std_msgs::String msg;
	msg.data = "Y";
	
	init_publisher_subscriber(gnc_node);

	wait4connect();
	wait4start();

	initialize_local_frame();
	set_speed(rips.get_speed());
	takeoff(rips.get_altitude());
	
	while(ros::ok())
	{
		
		ros::spinOnce();
		rate.sleep();
		rips.move();
		if(rips.picam){
			count++;
			pub.publish(msg);
			if(count==4){
			count=0;
			rips.picam=false;
			}
		}	
	}
	return 0;
}
