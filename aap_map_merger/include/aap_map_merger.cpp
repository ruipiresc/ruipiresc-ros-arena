#include "aap_map_merger.h"
#include "ros/ros.h"

aap_map_merger::aap_map_merger()
{
	ros::NodeHandle pn("~");
	pn.param("number_of_robots", number_of_robots, 1);
	pn.param("frequency", frequency, 10.0);
	pn.param("merge_frequency", merge_frequency, 0.5);
	pn.param("resolution", resolution, 0.05);
	pn.param("height", height, 26.0);
	pn.param("length", length, 26.0);
	pn.param("robot_radius", robot_radius, 0.2);
	pn.param("comunication", comunication, 0);
	pn.param("real", real, 0);
	pn.getParam("number_of_robots", number_of_robots);
	pn.getParam("frequency", frequency);
	pn.getParam("merge_frequency", merge_frequency);
	pn.getParam("resolution", resolution);
	pn.getParam("height", height);
	pn.getParam("length", length);
	pn.getParam("robot_radius", robot_radius);
	pn.getParam("comunication", comunication);
	pn.getParam("real", real);

	pn.param("path", path, std::string("/home"));
	pn.getParam("path", path);

	path_positions = path + "/simulator/positions/";
	path_grids = path + "/simulator/grids/";

	if(number_of_robots==1) return;

	minOccupancy = 0;
	maxOccupancy = 100;
	unknownOccupancy = -1;
	
	map_buffer = 1; // no. messages
	positions_buffer = 128;

	if(comunication==0) idle = true;
	else idle = false;
	close_all = false;

	inicializer();
	startSubscribing();

	ROS_INFO("aap_map_merger: I'm alive");

	iterations = (int)(frequency/merge_frequency);

	publishMap();

	int i = 0;
	ros::Rate loop_rate(frequency);
	while (ros::ok()){
		i++;
		
		if(i==iterations){
			i=0;

			if(comunication == 1) updateGlobalMap();
			else{
				if(close_all){ 
					loop_rate.sleep();
					continue;
				}
				ros::spinOnce();
				if(newMap) publishMap();
			}
			
		}
		
		loop_rate.sleep();
	}
}

aap_map_merger::~aap_map_merger()
{
	
}

void aap_map_merger::updateGlobalMap(void)
{
	updatePositions(); 

	for(int aux_robot_number = 0; aux_robot_number<number_of_robots; aux_robot_number++)
		while(readMap(aux_robot_number)==false)
			ros::Duration(0.5).sleep();

	publishMap();
}

int aap_map_merger::index(int i, int j)
{
	return(j*ncols + i);
}

void aap_map_merger::inicializer(void)
{
	ncols = (int)length/resolution;
	nrows = (int)height/resolution;

	xOrigin = -length/2;
	yOrigin = -height/2;

	global_map.info.resolution = resolution; // m/cell
	global_map.info.width = ncols; // number of width cells
	global_map.info.height = nrows; // number of height cells
	
	global_map.info.origin.position.x = xOrigin;
	global_map.info.origin.position.y = yOrigin;
	global_map.info.origin.position.z = 0.0;
	global_map.info.origin.orientation.x = 0.0;
	global_map.info.origin.orientation.y = 0.0;
	global_map.info.origin.orientation.z = 0.0;
	global_map.info.origin.orientation.w = 1.0;

	global_map_frame = "/map";
	global_map_metadata_topic =  global_map_frame + "_metadata";
	position_topic = "/position"; 
	status_topic = "/exploration_status";

	for (int i = 0; i < number_of_robots; i++){
		std::ostringstream name_ostringstream;
		name_ostringstream << "/robot_" << i;
		const std::string name_string(name_ostringstream.str());
		robots_map_frames[i] = name_string +  "/map";
	}

	global_map.header.frame_id = global_map_frame;
	
	global_map.data.assign(ncols * nrows, unknownOccupancy);

	global_map_pub = n.advertise<nav_msgs::OccupancyGrid>(global_map_frame,map_buffer,true);
	global_map_metadata_pub = n.advertise<nav_msgs::MapMetaData>(global_map_metadata_topic,map_buffer,true);

	newMap = false;

	global_map_pub.publish(global_map);
	global_map_metadata_pub.publish(global_map.info);

	robotsPosition.data.clear();
	robotsPosition.data.resize(number_of_robots*3);
	robotsPosition.data.assign(number_of_robots*3, 0);	
}

void aap_map_merger::startSubscribing(void)
{
	if(comunication==0) status_sub = n.subscribe(status_topic, positions_buffer, &aap_map_merger::statusReceived, this);
	for(int i = 0; i<number_of_robots;i++)
		map_sub[i] = n.subscribe<nav_msgs::OccupancyGrid>(robots_map_frames[i] , map_buffer, &aap_map_merger::mapReceived, this);
	if(comunication==0) position_sub = n.subscribe(position_topic, positions_buffer, &aap_map_merger::positionReceived, this);
}

void aap_map_merger::publishMap(void)
{
	newMap = false;
	global_map_pub.publish(global_map);

	if(comunication==1){
		std::string file = path_grids + "global.grid";
	
		std::ofstream map_file;
		map_file.open (file.c_str(), std::ios::out | std::ios::trunc); 

		for (int i=0; i<ncols; i++) for (int j=0; j<nrows; j++){
			int aux = global_map.data[index(i,j)];
			map_file << aux << "\n";
		}
		map_file.close();

		ROS_INFO("aap_map_merger: new GlobalMap");
	}
}

int aap_map_merger::closestRobot(int i, int j)
{ 
	double x = i*resolution, y = j*resolution, minDist = nrows*ncols;
	int closest_robot=-1;

	for (int k=0; k<number_of_robots; k++){
		double dist = sqrt(pow(x-robotsPosition.data[k*3+1],2)+pow(y-robotsPosition.data[k*3+2],2));
		if(dist<minDist) {
			minDist = dist;
			closest_robot = k; 
		}
	}

	return(closest_robot);
}

void aap_map_merger::statusReceived(const std_msgs::Float64MultiArray::ConstPtr& msg_ptr)
{
	std::vector<double>::const_iterator msg = msg_ptr->data.begin();

	double aux_code = *msg; 	msg++; msg++;
	double aux_time = *msg;	msg++;
	double aux_code2 = *msg;
	
	if( aux_code == -2.0 && aux_code2 == -2.0){
		close_all = true;
	}else if( aux_code == -1.0 && aux_code2 == -1.0){
		global_map.data.assign(ncols * nrows, unknownOccupancy);
		global_map_pub.publish(global_map);
		idle = true;
	}else if( aux_code == 2.2){
		while( ros::Time::now().toSec() < aux_time )
			ros::Duration(0.5/frequency).sleep();
		idle = false;
	}
}

void aap_map_merger::mapReceived(const nav_msgs::OccupancyGrid::ConstPtr& map_ptr)
{
	if(idle) return;

	newMap = true;

	int aux_robot_number = 0;
	std::string aux1 = map_ptr->header.frame_id;
	for (int i = 0; i < number_of_robots; i++){
		std::ostringstream name_ostringstream;
		name_ostringstream << "/robot_" << i;
		const std::string name_string(name_ostringstream.str());
		std::string aux2 = name_string +  "/map";
		if(aux1 == aux2) aux_robot_number = i;
	}
 
	for (int i=0; i<ncols; i++) for (int j=0; j<nrows; j++){
		if (global_map.data[index(i,j)] == unknownOccupancy)
			global_map.data[index(i,j)] = map_ptr->data[index(i,j)];
		else if( closestRobot(i,j) == aux_robot_number )
			global_map.data[index(i,j)] = map_ptr->data[index(i,j)];
	}
}

void aap_map_merger::positionReceived (const std_msgs::Float64MultiArray::ConstPtr& msg_ptr)
{
	if(idle) return;

	std::vector<double>::const_iterator msg = msg_ptr->data.begin();

	int aux_robot_number = (int)(*msg); 
	robotsPosition.data[aux_robot_number*3] = aux_robot_number;
	msg++;
	robotsPosition.data[aux_robot_number*3+1] = *msg;
	msg++;
	robotsPosition.data[aux_robot_number*3+2] = *msg;
}

void aap_map_merger::updatePositions(void)
{
	for(int i = 0;i<number_of_robots;i++)
		while(readPos(i) == false)
			ros::Duration(0.5).sleep();
}

bool aap_map_merger::readPos(int i)
{
	std::ostringstream robot_name_ostr;
	robot_name_ostr << "robot_" << i;
	const std::string robot_name(robot_name_ostr.str());
	std::string file = path_positions + robot_name + ".pos";

    std::ifstream ifile(file.c_str(), std::ios::in);

    if (!ifile.is_open()){
    	ROS_WARN("aap_map_merger: There was a problem opening the file %s", file.c_str());
    	ifile.close();
    	return(false);
    }else{
    	double num = 0.0;
    	robotsPosition.data[i*3] = i;
    	ifile >> num;
		robotsPosition.data[i*3+1] = num;
		ifile >> num;
		robotsPosition.data[i*3+2] = num;
		ifile.close();
    	return(true);
    }
}

bool aap_map_merger::readMap(int i)
{
	std::ostringstream robot_name_ostr;
	robot_name_ostr << "robot_" << i;
	const std::string robot_name_aux(robot_name_ostr.str());
	std::string file = path_grids + robot_name_aux + ".grid"; 
	
	std::ifstream map_file(file.c_str(), std::ios::in);
    if (!map_file.is_open()) {
    	ROS_WARN("MAPPING: There was a problem opening the file %s", file.c_str());
    	map_file.close();
		return(false);
    }else{
    	int aux = -1;
    	for (int i=0; i<ncols; i++) for (int j=0; j<nrows; j++){
    		map_file >> aux;
			if (global_map.data[index(i,j)] == unknownOccupancy)
				global_map.data[index(i,j)] = aux;
			else if( closestRobot(i,j) == i && aux!= unknownOccupancy )
				global_map.data[index(i,j)] = aux;
		}
		map_file.close();
		return(true);
    }
}

