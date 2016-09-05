#include "simulator.h"
#include "ros/ros.h"

Simulator::Simulator()
{
	ros::NodeHandle pn("~");
	pn.param("iterations", iterations, 1.0);
	pn.param("frequency", frequency, 10.0);
	pn.param("path", path, std::string("/home"));
	pn.getParam("iterations", iterations);
	pn.getParam("frequency", frequency);
	pn.getParam("path", path);

	path = path + "/simulator/results/results.txt";

    new_status = false;
    iteration = 1.0;
    med = -1.0;
    int errorCount = 0; 

	log.data.clear();
	log.data.push_back(0);

	status_topic = "/exploration_status";
	status_pub = n.advertise<std_msgs::Float64MultiArray>(status_topic, 128);
	status_sub = n.subscribe(status_topic, 128, &Simulator::statusReceived, this);

	ROS_INFO("SIMULATOR: I'm alive");
	

	double startTime = ros::Time::now().toSec() + 8;
		
	while( ros::Time::now().toSec() < startTime )
		ros::Duration(0.5/frequency).sleep();
	
	startTime = ros::Time::now().toSec() + 3;

	sendStatus(2.2, iteration, startTime-3.0, 0.0); //order for map_merger
	sendStatus(3.3, iteration, startTime-2.0, 0.0); //order for mapping
	sendStatus(4.4, iteration, startTime-1.0, 0.0); //order for frontier
	sendStatus(5.5, iteration, startTime, 0.0); //order for burgard

	ros::Rate loop_rate(frequency);
	while(ros::ok()){
		if(iteration<=iterations){
			ros::spinOnce();

			if(new_status){
				logData();
				ROS_INFO("SIMULATOR: errors: %d med= %lf", errorCount, med);
				iteration+=1.0;
				if(iteration>iterations){
					for (int i=0;i<log.data[0];i++)
						ROS_INFO("SIMULATOR: rep= %lf, time= %lf, ratio= %lf", log.data[1+i*3], log.data[2+i*3], log.data[3+i*3]);

					ROS_INFO("SIMULATOR: errors: %d med= %lf", errorCount, med);
					sendStatus(-2.0, iteration, 0.0, -2.0); //error for all
					continue;
				}
				startTime = ros::Time::now().toSec() + 12;
				sendStatus(2.2, iteration, startTime-3.0, 0.0); //order for map_merger
				sendStatus(3.3, iteration, startTime-2.0, 0.0); //order for mapping
				sendStatus(4.4, iteration, startTime-1.0, 0.0); //order for frontier
				sendStatus(5.5, iteration, startTime, 0.0); //order for burgard
				new_status = false;
			}

			if( ((ros::Time::now().toSec() - startTime) > (med*1.5)) && med > 0){
				errorCount++;
				/*if(errorCount>iterations){
					iteration = iterations;
					for (int i=0;i<log.data[0];i++)
						ROS_INFO("SIMULATOR: rep= %lf, time= %lf, ratio= %lf", log.data[1+i*3], log.data[2+i*3], log.data[3+i*3]);
					ROS_INFO("SIMULATOR: errors: %d med= %lf", errorCount, med);
					sendStatus(-2.0, iteration, 0.0, -2.0); //error for all
					continue;
				}*/
				ROS_WARN("SIMULATOR: error: %d med= %lf, time= %lf", errorCount, med, (ros::Time::now().toSec() - startTime) );
				sendStatus(-1.0, iteration, 0.0, -1.0); //error for all
				startTime = ros::Time::now().toSec() + 12;
				sendStatus(2.2, iteration, startTime-3.0, 0.0); //order for map_merger
				sendStatus(3.3, iteration, startTime-2.0, 0.0); //order for mapping
				sendStatus(4.4, iteration, startTime-1.0, 0.0); //order for frontier
				sendStatus(5.5, iteration, startTime, 0.0); //order for burgard
			}
		}

		loop_rate.sleep();
	}
}

Simulator::~Simulator()
{

}

void Simulator::sendStatus(double codeT, double repT, double timeT, double erroT)
{
	status_out.data.clear();
	status_out.data.push_back(codeT);
	status_out.data.push_back(repT);
	status_out.data.push_back(timeT);
	status_out.data.push_back(erroT);
	status_pub.publish(status_out);
}

void Simulator::statusReceived(const std_msgs::Float64MultiArray::ConstPtr& msg_ptr)
{
	std::vector<double>::const_iterator msg = msg_ptr->data.begin();

	double aux_code = *msg; 	msg++;
	double aux_rep = *msg;		msg++;
	double aux_time = *msg;	msg++;
	double aux_ratio = *msg;		

	if(aux_code == 1.1 && aux_rep == iteration){
		new_status = true;
		code = aux_code;
		rep = aux_rep;
		time = aux_time;
		ratio = aux_ratio; msg++;
		alfa = *msg;		
	}
}

void Simulator::logData(void)
{
	log.data.push_back(iteration);
	log.data.push_back(time);
	log.data.push_back(ratio);
	log.data[0]++;

	static double total = 0;
	total+=time;
	med = total/log.data[0];

	ROS_INFO("SIMULATOR: Finish test%1.0lf in %1.1lf(s), map ratio was %1.3lf", iteration, time, ratio);
	myfile.open (path.c_str(), std::ios::app); 
	myfile << time << " " << ratio << " " << alfa << "\n";
	myfile.close();
}

