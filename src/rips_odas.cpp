#include <array>
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "odas_ros/OdasSst.h"
#include "odas_ros/OdasSstArrayStamped.h"
class Msg{
public:
	std_msgs::Float64MultiArray pms;
	Msg();
	void callback(const odas_ros::OdasSstArrayStamped& msg);
};
Msg::Msg()
{
	this->pms.data={0.,0.,-1.};
}
void Msg::callback(const odas_ros::OdasSstArrayStamped& msg)
{
	if(!msg.sources.empty())
	{
		odas_ros::OdasSst temp{msg.sources[0]};
		this->pms.data={-temp.x,-temp.y,-temp.z};
	}
	
}

int main(int argc, char** argv)
{
	
	Msg msg;
	ros::init(argc, argv, "rips_odas");
	ros::NodeHandle gnc_node;
	ros::Publisher pub = gnc_node.advertise<std_msgs::Float64MultiArray>("/ripscom", 100);
	ros::Subscriber sub = gnc_node.subscribe("/odas/sst", 100, &Msg::callback,&msg);
	ros::Rate rate(20);
	
	for(int i=0;i<200;i++){
		rate.sleep();
		ros::spinOnce();
		if(i%50==49)pub.publish(msg.pms);
	}
		
	
			
	
	return 0;
}
