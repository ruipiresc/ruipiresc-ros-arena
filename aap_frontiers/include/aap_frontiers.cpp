#include "aap_frontiers.h"
#include "ros/ros.h"

aap_frontiers::aap_frontiers()
{
	ros::NodeHandle pn("~");
	pn.param("number_of_robots", number_of_robots, 1);
	pn.param("frequency", frequency, 10.0);
	pn.param("resolution", resolution, 0.05);
	pn.param("height", height, 20.0);
	pn.param("length", length, 20.0);
	pn.param("robot_radius", robot_radius, 0.2);
	pn.param("robot_number", robot_number, 0);
	pn.param("robot_name", robot_name, std::string("robot_0"));
	pn.param("start_time", start_time, 0.0);
	pn.param("comunication", comunication, 0);
	pn.param("real", real, 0);
	pn.getParam("number_of_robots", number_of_robots);
	pn.getParam("frequency", frequency);
	pn.getParam("resolution", resolution);
	pn.getParam("height", height);
	pn.getParam("length", length);
	pn.getParam("robot_radius", robot_radius);
	pn.getParam("robot_number", robot_number);
	pn.getParam("robot_name", robot_name);
	pn.getParam("start_time", start_time);
	pn.getParam("comunication", comunication);
	pn.getParam("real", real);

	unknownCell = -1;
	freeCell = 0;
	occupiedCell = 50;
	inaccessibleCell = 75;
	frontierCell = 100;
	maxOccupancy = 100;
	updateFrontiersPeriod = 50 * (1/frequency);
	timeBufferSize = 20.0; // seconds
	sendBufferSize = 1; // no. messages
	receiveBufferSize = 128; // no. messages

	while(!ready()) ros::Duration(0.5/frequency).sleep();
    start();
}

aap_frontiers::~aap_frontiers()
{
	
}

bool aap_frontiers::ready(void)
{
	static bool configurated = false;

	if(!configurated){
		if(number_of_robots>1) multiRobot = true;
		else multiRobot = false;

		if(multiRobot && comunication==0) idle = true;
		else idle = false;

		close_all = false;

		robot_name = "/" + robot_name;
		status_topic = "/exploration_status";
		if(multiRobot){
			frontiers_frame = robot_name + "/frontiers";
			frontiers_metadata_topic =  robot_name + frontiers_frame + "_metadata";
			map_frame = robot_name + "/map";
			markCell_topic = robot_name + "/mark_cell";
		}else{
			frontiers_frame = "/frontiers";
			frontiers_metadata_topic =  frontiers_frame + "_metadata";
			map_frame = "/map";
			markCell_topic = "/mark_cell";
		}

		ncols = (int)length/resolution;
		nrows = (int)height/resolution;

		xOrigin = -length/2;
		yOrigin = -height/2;
		
		frontiers.info.resolution = resolution; // m/cell
		frontiers.info.width = ncols;
		frontiers.info.height = nrows;
		frontiers.info.origin.position.x = xOrigin;
		frontiers.info.origin.position.y = yOrigin;
		frontiers.info.origin.position.z = 0.0;
		frontiers.info.origin.orientation.x = 0.0;
		frontiers.info.origin.orientation.y = 0.0;
		frontiers.info.origin.orientation.z = 0.0;
		frontiers.info.origin.orientation.w = 1.0;
		frontiers.header.frame_id = frontiers_frame;
		frontiers.data.assign(ncols * nrows, unknownCell);
		frontiers_pub = n.advertise<nav_msgs::OccupancyGrid>(frontiers_frame,sendBufferSize,true);
		frontiers_metadata_pub = n.advertise<nav_msgs::MapMetaData>(frontiers_metadata_topic,sendBufferSize,true);
		listener = new tf::TransformListener(ros::Duration(timeBufferSize), true);	
		map_to_frontiers_transform = tf::StampedTransform( tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)), ros::Time::now(), map_frame, frontiers_frame);

		configurated = true;
	}
	
	if(configurated){
		if(!multiRobot && ros::Time::now().toSec() < start_time ) return(false);
		return(true);
	}
	
	return(false);
}

void aap_frontiers::start(void)
{
	// init subs
	map_sub = n.subscribe<nav_msgs::OccupancyGrid>(map_frame, sendBufferSize, &aap_frontiers::mapReceived, this);
	markCell_sub = n.subscribe(markCell_topic, receiveBufferSize, &aap_frontiers::markCellReceived, this);
	if(multiRobot && comunication==0) status_sub = n.subscribe(status_topic, receiveBufferSize, &aap_frontiers::statusReceived, this);
	
	ROS_INFO("FRONTIERS-robot_%d: I'm alive", robot_number);

	ros::Rate loop_rate(frequency);

	// cycle
	while (ros::ok()){
		if(!close_all){ 
			updateTf();
			ros::spinOnce();
		}
		loop_rate.sleep();
	}
}


void aap_frontiers::updateTf(void)
{
	map_to_frontiers_transform.stamp_ = ros::Time::now();
	map_to_frontiers_broadcaster.sendTransform(map_to_frontiers_transform);
}

void aap_frontiers::mapReceived(const nav_msgs::OccupancyGrid::ConstPtr& map_ptr)
{
	if(idle) return;
	map = *map_ptr;

	bool newParam = false; 

	if(ncols != map.info.width) newParam = true;
	if(nrows != map.info.height) newParam = true;
	if(map.info.origin.position.x != frontiers.info.origin.position.x) newParam = true;
	if(map.info.origin.position.y != frontiers.info.origin.position.y) newParam = true;

	if(newParam){
		nav_msgs::OccupancyGrid frontiers_copy = frontiers;

		int aux_ncols = map.info.width;
		int aux_nrows = map.info.height;

		double aux_xOrigin = map.info.origin.position.x;
		double aux_yOrigin = map.info.origin.position.y;

		frontiers.info.width = aux_ncols;
		frontiers.info.height = aux_nrows;
		frontiers.info.origin.position.x = aux_xOrigin;
		frontiers.info.origin.position.y = aux_yOrigin;
		frontiers.data.resize(aux_nrows * aux_ncols);
		frontiers.data.assign(aux_ncols * aux_nrows, unknownCell);

		for(int i=0; i<ncols; i++) for(int j=0; j<nrows; j++)
			if(frontiers_copy.data[index(i,j)]==inaccessibleCell)
				frontiers.data[convertIndex(i,j,aux_xOrigin,aux_yOrigin,aux_ncols)]=inaccessibleCell;

		ncols = aux_ncols;
		nrows = aux_nrows;
		xOrigin = aux_xOrigin;
		yOrigin = aux_yOrigin;
	}

	updateFrontiers();
}

void aap_frontiers::updateFrontiers(void)
{
	for (int i=0; i<ncols; i++) for (int j=0; j<nrows; j++)
		if (frontiers.data[index(i,j)]!=inaccessibleCell){
			if( map.data[index(i,j)] == unknownCell){ 
				if (isFrontier(i,j)) frontiers.data[index(i,j)]=frontierCell;
				else frontiers.data[index(i,j)] = unknownCell;
			}else if( map.data[index(i,j)] <= 50 ) frontiers.data[index(i,j)] = freeCell;
			else inflameObstacles(i,j);
		}

	frontiers_pub.publish(frontiers);
	frontiers_metadata_pub.publish(frontiers.info);
}

bool aap_frontiers::isFrontier(int i, int j)
{ 
	int cellRadius =  1;
	int i_min, i_max, j_min, j_max;
	if(i - cellRadius <= 0) i_min = 0; else i_min = i - cellRadius;
	if(i + cellRadius >= ncols) i_max = ncols - 1; else i_max = i + cellRadius;
	if(j - cellRadius <= 0) j_min = 0; else j_min = j - cellRadius;
	if(j + cellRadius >= nrows) j_max = nrows - 1; else j_max = j + cellRadius;

	for (i=i_min; i<=i_max; i++) for (j=j_min; j<=j_max; j++)
		if( 0 <= map.data[index(i,j)] && map.data[index(i,j)] <= 50)
			return(true);

	return(false);
}

void aap_frontiers::inflameObstacles(int i,int j)
{
	int cellRadius = 2;
	int i_min, i_max, j_min, j_max;
	if(i - cellRadius <= 0) i_min = 0; else i_min = i - cellRadius;
	if(i + cellRadius >= ncols) i_max = ncols - 1; else i_max = i + cellRadius;
	if(j - cellRadius <= 0) j_min = 0; else j_min = j - cellRadius;
	if(j + cellRadius >= nrows) j_max = nrows - 1; else j_max = j + cellRadius;

	for (i=i_min; i<=i_max; i++) for (j=j_min; j<=j_max; j++)
		if (frontiers.data[index(i,j)]!=inaccessibleCell)
			frontiers.data[index(i,j)]=occupiedCell;
}

void aap_frontiers::markCellReceived(const std_msgs::Int32MultiArray::ConstPtr& msg_ptr)
{
	if(idle) return;

	std::vector<int>::const_iterator msg = msg_ptr->data.begin();
	int numberOfCells = msg_ptr->data[0];
	int value = msg_ptr->data[1];

	if(value==inaccessibleCell) for (int i=0; i<numberOfCells; i++)
		frontiers.data[index(msg_ptr->data[2+i*2],msg_ptr->data[3+i*2])]=inaccessibleCell;		
}

void aap_frontiers::statusReceived(const std_msgs::Float64MultiArray::ConstPtr& msg_ptr)
{
	std::vector<double>::const_iterator msg = msg_ptr->data.begin();

	double aux_code = *msg; 	msg++; msg++;
	double aux_time = *msg;	msg++;
	double aux_code2 = *msg;
	
	if( aux_code == -2.0 && aux_code2 == -2.0){
		close_all = true;
	}else if( aux_code == -1.0 && aux_code2 == -1.0){
		frontiers.data.assign(ncols * nrows, unknownCell);
		map.data.assign(ncols * nrows, unknownCell);
		frontiers_pub.publish(frontiers);
		frontiers_metadata_pub.publish(frontiers.info);
		idle = true;
	}else if( aux_code == 4.4){
		while( ros::Time::now().toSec() < aux_time )
			ros::Duration(0.5/frequency).sleep();
		idle = false;
	}
}

int aap_frontiers::index(int i, int j)
{
	return(j*ncols + i);
}

double aap_frontiers::getX(int i)
{
	return(i*resolution + xOrigin);
}

double aap_frontiers::getY(int j)
{
	return(j*resolution + yOrigin);
}

int aap_frontiers::convertIndex(int i, int j, double xOaux, double yOaux, double ncolsAux){
	i = (int)((getX(i) - xOaux)/resolution);
	j = (int)((getY(j) - yOaux)/resolution);
	return( j*ncolsAux + i);
}