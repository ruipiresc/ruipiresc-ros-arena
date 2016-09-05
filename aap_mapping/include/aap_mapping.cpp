#include "aap_mapping.h"
#include "ros/ros.h"

aap_mapping::aap_mapping()
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

	pn.param("laser_topic", laser_topic, std::string("base_scan"));
	pn.param("laser_frame", laser_frame, std::string("base_laser_link"));

	pn.param("inicial_x", inicial_x, 0.0);
	pn.param("inicial_y", inicial_y, 0.0);
	pn.param("inicial_theta", inicial_theta, 0.0);
	pn.param("start_time", start_time, 0.0);
	pn.param("occupiedCellProb", occupiedCellProb, 0.5);
	pn.param("emptyCellProb", emptyCellProb, 0.5);
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

	pn.getParam("laser_topic", laser_topic);
	pn.getParam("laser_frame", laser_frame);

	pn.getParam("inicial_x", inicial_x);
	pn.getParam("inicial_y", inicial_y);
	pn.getParam("inicial_theta", inicial_theta);
	pn.getParam("start_time", start_time);
	pn.getParam("occupiedCellProb", occupiedCellProb);
	pn.getParam("emptyCellProb", emptyCellProb);
	pn.getParam("comunication", comunication);
	pn.getParam("real", real);

	pn.param("path", path, std::string("/home"));
	pn.getParam("path", path);

	path_positions = path + "/simulator/positions/";
	path_grids = path + "/simulator/grids/";

	global_grid_name = path_grids + "global.grid";
	grid_name = path_grids + robot_name + ".grid";

	if(number_of_robots>1) multiRobot = true;
	else multiRobot = false;

	minOccupancy = 0;
	maxOccupancy = 100;
	unknownOccupancy = -1;

	max_laser_dist = 4.00;
	min_laser_dist = 0.02;

	obstacle_extender = ceil(0.5*robot_radius/resolution);

	listener_buffer = 10.0; // seconds
	map_buffer = 1; // no. messages
	laser_buffer = 1; // no. messages

	if(multiRobot && comunication==0) idle = true;
	else idle = false;
	close_all = false;

	inicializer();

	if(!multiRobot)
		while( ros::Time::now().toSec() < start_time )
			ros::Duration(0.5/frequency).sleep();

	startSubscribing();

	ROS_INFO("aap_mapping-robot_%d: I'm alive", robot_number);

	int iterations = ((int)(frequency))*5;
	int i = iterations-1;

	publishMap();

	ros::Rate loop_rate(frequency);
	while (ros::ok()){
		map_to_odom_transform.stamp_ = ros::Time::now();
		map_to_odom_broadcaster.sendTransform(map_to_odom_transform);
	
		if(multiRobot){
			global_map_to_map_transform.stamp_ = ros::Time::now();
			global_map_to_map_broadcaster.sendTransform(global_map_to_map_transform);
		}

		if(close_all){ 
			loop_rate.sleep();
			continue;
		}
	
		if(!idle) updateRobotPosition();
		if(comunication==1 && i==iterations){
			checkGlobalMap();
			i=0;
		}else i++;
		
		ros::spinOnce();

		if(newMap) publishMap();
		
		loop_rate.sleep();
	}
}

aap_mapping::~aap_mapping()
{
	
}

int aap_mapping::index(int i, int j)
{
	return(j*ncols + i);
}

void aap_mapping::inicializer(void)
{
	newMap = false;

	ncols = (int)length/resolution;
	nrows = (int)height/resolution;

	xOrigin = -length/2;
	yOrigin = -height/2;

	map.info.resolution = resolution; // m/cell
	map.info.width = ncols; // number of width cells
	map.info.height = nrows; // number of height cells
	
	map.info.origin.position.x = xOrigin;
	map.info.origin.position.y = yOrigin;
	map.info.origin.position.z = 0.0;
	map.info.origin.orientation.x = 0.0;
	map.info.origin.orientation.y = 0.0;
	map.info.origin.orientation.z = 0.0;
	map.info.origin.orientation.w = 1.0;

	position_topic = "/position";
	status_topic = "/exploration_status";
	if(multiRobot){
		map_frame = "/" + robot_name + "/map";
		laser_topic = "/" + robot_name + "/" + laser_topic;
		base_link_frame = "/" + robot_name + "/base_link";
		laser_frame = "/" + robot_name + "/" + laser_frame;
		odom_frame = "/" + robot_name + "/odom";
		map_metadata_topic =  "/" + robot_name + map_frame + "_metadata";
		markCell_topic = "/" + robot_name + "/mark_cell";
		global_map_frame = "/map";
		cmd_vel_topic = "/" + robot_name + "/cmd_vel";
	}else{
		map_frame = "/map";
		laser_topic = "/" + laser_topic;
		base_link_frame = "/base_link";
		laser_frame = "/" + laser_frame;
		odom_frame = "/odom";
		map_metadata_topic =  map_frame + "_metadata";
		markCell_topic = "/mark_cell";
		global_map_frame = "/map";
		cmd_vel_topic = "/cmd_vel";
	}

	map.header.frame_id = map_frame;
	
	map.data.assign(ncols * nrows, unknownOccupancy);

	map_pub = n.advertise<nav_msgs::OccupancyGrid>(map_frame,map_buffer,true);
	map_metadata_pub = n.advertise<nav_msgs::MapMetaData>(map_metadata_topic,map_buffer,true);

	int i = getI(inicial_x);
	int j = getJ(inicial_y); 

	if(i!=0 && j!=0){
		int cellRadius = ceil(robot_radius/resolution);
		int i_min, i_max, j_min, j_max;
		if(i - cellRadius <= 0) i_min = 0; else i_min = i - cellRadius;
		if(i + cellRadius >= ncols) i_max = ncols - 1; else i_max = i + cellRadius;
		if(j - cellRadius <= 0) j_min = 0; else j_min = j - cellRadius;
		if(j + cellRadius >= nrows) j_max = nrows - 1; else j_max = j + cellRadius;
		
		for (i=i_min; i<=i_max; i++) for (j=j_min; j<=j_max; j++)
			map.data[index(i,j)]=minOccupancy;
	}

	map_pub.publish(map);
	map_metadata_pub.publish(map.info);

	listener_base_laser_link = new tf::TransformListener(ros::Duration(listener_buffer), true);
	listener_base_link = new tf::TransformListener(ros::Duration(listener_buffer), true);

	map_to_odom_transform.frame_id_ = std::string(map_frame);
	map_to_odom_transform.child_frame_id_ = std::string(odom_frame);
	map_to_odom_transform.setOrigin(tf::Vector3(inicial_x, inicial_y, 0.0));
	tf::Quaternion rotation;
	rotation.setRPY(0, 0, ((inicial_theta*M_PI)/180) );
	map_to_odom_transform.setRotation(rotation);

	global_map_to_map_transform = tf::StampedTransform( tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)), ros::Time::now(), global_map_frame, map_frame);

	robotsPosition.data.clear();
	robotsPosition.data.resize(number_of_robots*3);
	robotsPosition.data.assign(number_of_robots*3, 0);

	position_pub = n.advertise<std_msgs::Float64MultiArray>(position_topic, 100);

	angularSpeed = 0;
}

void aap_mapping::startSubscribing(void)
{
	if(multiRobot){
		if(comunication==0)status_sub = n.subscribe(status_topic, 128, &aap_mapping::statusReceived, this);
		global_map_sub = n.subscribe<nav_msgs::OccupancyGrid>(global_map_frame , map_buffer, &aap_mapping::globalMapReceived, this);
		if(comunication==0)position_sub = n.subscribe(position_topic, 100, &aap_mapping::positionReceived, this);
	}

	laser_sub = n.subscribe<sensor_msgs::LaserScan>(laser_topic, laser_buffer, &aap_mapping::scanReceived, this);
	cmd_vel_sub = n.subscribe(cmd_vel_topic, map_buffer, &aap_mapping::velReceived , this);
}

void aap_mapping::scanReceived(const sensor_msgs::LaserScan::ConstPtr& data_ptr)
{
	if(idle) return;

	int size = data_ptr->ranges.size();
	int obstacle_extension;
	double distance = 0, angle = 0;
	bool increase;

	geometry_msgs::PointStamped base_laser_link_point;
	base_laser_link_point.header.frame_id = laser_frame;
	base_laser_link_point.header.stamp = data_ptr->header.stamp;
	base_laser_link_point.point.z = 0.0; 

	geometry_msgs::PointStamped map_point;
	map_point.header.frame_id = map_frame;
	map_point.header.stamp = data_ptr->header.stamp;

	if(!listener_base_laser_link->waitForTransform(map_frame, laser_frame, data_ptr->header.stamp,ros::Duration(0.001))){
		//ROS_WARN("aap_mapping-robot_%d: no transform at this time", robot_number);
		return;
	}

	newMap = true;

	for (int i = 0; i < size; i++){
		if( data_ptr->ranges[i]<=min_laser_dist || data_ptr->ranges[i] > max_laser_dist || isnan(data_ptr->ranges[i]) || isinf(data_ptr->ranges[i])) continue;

		distance = data_ptr->ranges[i];

		if (distance == max_laser_dist){
			increase = false;
			distance -=obstacle_extender*resolution;
		}else increase = true;

		angle = data_ptr->angle_min + i*data_ptr->angle_increment;

		obstacle_extension = obstacle_extender;
		while (distance>=0){
			base_laser_link_point.point.x = distance*cos(angle);
			base_laser_link_point.point.y = distance*sin(angle);
			
			try{
				geometry_msgs::PointStamped map_point;
				listener_base_laser_link->transformPoint(map_frame, data_ptr->header.stamp + ros::Duration().fromSec(i*data_ptr->time_increment), base_laser_link_point, map_frame, map_point);
				newData(map_point.point.x, map_point.point.y, increase); 
			}catch(tf::TransformException& ex){
				ROS_ERROR("aap_mapping-robot_%d: %s", robot_number, ex.what());
			}

			distance -= resolution;
			if(obstacle_extension!=0) obstacle_extension--;
			else increase = false;
		}

	}

	listener_base_laser_link->clear();
}

void aap_mapping::globalMapReceived(const nav_msgs::OccupancyGrid::ConstPtr& global_map_ptr)
{
	if(idle) return;

	newMap = true;

	for (int i=0; i<ncols; i++) for (int j=0; j<nrows; j++){
		if (map.data[index(i,j)] == unknownOccupancy)
			map.data[index(i,j)] = global_map_ptr->data[index(i,j)];
		else if( closestRobot(i,j) != robot_number )
			map.data[index(i,j)] = global_map_ptr->data[index(i,j)];
	}
}

void aap_mapping::velReceived(const geometry_msgs::Twist& cmd_vel_ptr)
{
	if(idle) return;

	angularSpeed = cmd_vel_ptr.angular.z;
}

void aap_mapping::newData(double xx, double yy, bool increase)
{
	int i = getI(xx), j = getJ(yy);

	if(i>=0 && i<ncols && j>=0 && j<nrows){
		if(map.data[index(i,j)] == unknownOccupancy) 
			map.data[index(i,j)] = 50;

		double speed = fabs(angularSpeed);
		if(speed>= 0 && speed<1){
			if(increase) inflateObstacles(i,j,(occupiedCellProb*(1-speed)));
			else changeData(i,j,(-emptyCellProb*(1-speed)));
		}
	}
}
void aap_mapping::changeData(int i, int j, double prob)
{
	if(map.data[index(i,j)] == minOccupancy) map.data[index(i,j)] = minOccupancy+1;
	if(map.data[index(i,j)] == maxOccupancy) map.data[index(i,j)] = maxOccupancy-1;
	double pMx = (1+prob)/2;
	double pOccLast = map.data[index(i,j)]/100.0;
	if(pOccLast>=1) return;
	double lastData = log(1.0/(1.0-pOccLast)-1.0);
	if(pMx>=1) return;
	double newData = log(pMx/(1.0-pMx));
	double pMlt = lastData + newData;
	if(pMlt<=-9999) pMlt = -9999;
	double pOcc = 1.0-(1.0/(1.0+exp(pMlt)));
	double roundData = round(pOcc*100.0);
	int result = (int)(roundData);
	if(result>=minOccupancy && result<=maxOccupancy)
		map.data[index(i,j)] = result;
}

void aap_mapping::inflateObstacles(int i, int j, double prob)
{
	int cellRadius =  (int)(0.5*robot_radius/resolution);
	int i_min, i_max, j_min, j_max;
	if(i - cellRadius <= 0) i_min = 0; else i_min = i - cellRadius;
	if(i + cellRadius >= ncols) i_max = ncols - 1; else i_max = i + cellRadius;
	if(j - cellRadius <= 0) j_min = 0; else j_min = j - cellRadius;
	if(j + cellRadius >= nrows) j_max = nrows - 1; else j_max = j + cellRadius;

	for (i=i_min; i<=i_max; i++) for (j=j_min; j<=j_max; j++)
		changeData(i,j,prob);	
}

void aap_mapping::updateRobotPosition(void)
{	
	double now = ros::Time::now().toSec();
	static double last_time = now;
	if(last_time + 0.5 >= now) return;
	last_time = now;

	ros::Time time_stamp = ros::Time::now();
	geometry_msgs::PointStamped base_link_point;
	base_link_point.header.frame_id = base_link_frame;
	base_link_point.header.stamp = time_stamp;
	base_link_point.point.x = 0.0;
	base_link_point.point.y = 0.0;
	base_link_point.point.z = 0.0;

	geometry_msgs::PointStamped map_point;
	map_point.header.frame_id = map_frame;
	map_point.header.stamp = time_stamp;

	if(!listener_base_link->waitForTransform(map_frame, base_link_frame, time_stamp,ros::Duration(0.001))){
		ROS_WARN("aap_mapping-robot_%d: no Position at this time", robot_number);
		return;
	}

	try{
		listener_base_link->transformPoint(map_frame, time_stamp, base_link_point, map_frame, map_point);
		robotsPosition.data[robot_number*3] = robot_number;
		robotsPosition.data[robot_number*3+1] = map_point.point.x;
		robotsPosition.data[robot_number*3+2] = map_point.point.y;
		if(multiRobot) publishPosition();
	}catch(tf::TransformException& ex){
		ROS_ERROR("aap_mapping-robot_%d: %s", robot_number, ex.what());
	}
}

int aap_mapping::closestRobot(int i, int j)
{ 
	double x = getX(i), y = getY(j), minDist = nrows*ncols;
	int closest_robot=-1;

	for (int k=0; k<number_of_robots; k++){
		double dist = sqrt(pow(x-robotsPosition.data[k*3+1],2)+pow(y-robotsPosition.data[k*3+2],2));
		if(dist<minDist){
			minDist = dist;
			closest_robot = k;
		}
	}

	return(closest_robot);
}

void aap_mapping::positionReceived (const std_msgs::Float64MultiArray::ConstPtr& msg_ptr)
{
	if(idle) return;

	std::vector<double>::const_iterator msg = msg_ptr->data.begin();

	int aux_robot_number = (int)(*msg); 
	if(robot_number == aux_robot_number) return;

	robotsPosition.data[aux_robot_number*3] = aux_robot_number;
	msg++;
	robotsPosition.data[aux_robot_number*3+1] = *msg;
	msg++;
	robotsPosition.data[aux_robot_number*3+2] = *msg;
}

void aap_mapping::updatePositions(void)
{
	for(int i = 0;i<number_of_robots;i++){
		if(i==robot_number)continue;

		std::ostringstream robot_name_ostr;
		robot_name_ostr << "robot_" << i;
		const std::string robot_name_aux(robot_name_ostr.str());
		std::string file = path_positions + robot_name_aux + ".pos";

	    std::ifstream read_file(file.c_str(), std::ios::in);

	    if (!read_file.is_open()) {
	    	ROS_WARN("aap_mapping-robot_%d: There was a problem opening the file %s", robot_number, file.c_str());
	    }else{
	    	double num = 0.0;
	    	robotsPosition.data[i*3] = i;
	    	read_file >> num;
			robotsPosition.data[i*3+1] = num;
			read_file >> num;
			robotsPosition.data[i*3+2] = num;
	    }
	}
}

void aap_mapping::publishMap(void)
{
	int i = getI(robotsPosition.data[robot_number*3+1]);
	int j = getJ(robotsPosition.data[robot_number*3+2]); 

	if(i!=getI(0.0) && j!=getJ(0.0)){
		int cellRadius = ceil(robot_radius/resolution);
		int i_min, i_max, j_min, j_max;
		if(i - cellRadius <= 0) i_min = 0; else i_min = i - cellRadius;
		if(i + cellRadius >= ncols) i_max = ncols - 1; else i_max = i + cellRadius;
		if(j - cellRadius <= 0) j_min = 0; else j_min = j - cellRadius;
		if(j + cellRadius >= nrows) j_max = nrows - 1; else j_max = j + cellRadius;
		
		for (i=i_min; i<=i_max; i++) for (j=j_min; j<=j_max; j++)
			map.data[index(i,j)]=minOccupancy;
	}

	map_pub.publish(map);
	map_metadata_pub.publish(map.info);

	if(comunication==1){
    	static int iterations = ((int)(frequency))*5;
		static int i = iterations - 1;
		i++;
		if(i==iterations){
			i=0;
			std::ofstream map_file;
			map_file.open(grid_name.c_str(), std::ios::out | std::ios::trunc); 

			int aux = -1;
			for (int i=0; i<ncols; i++) for (int j=0; j<nrows; j++){
				aux = map.data[index(i,j)];
				map_file << aux << "\n";
			}
			map_file.close();
		}
	}

	newMap = false;
}

void aap_mapping::checkGlobalMap(void)
{
	updatePositions(); 

	std::ifstream global_map_file(global_grid_name.c_str(), std::ios::in);

    if (!global_map_file.is_open()) {
    	ROS_WARN("aap_mapping-robot_%d: There was a problem opening the file %s", robot_number, global_grid_name.c_str());
    }else{
    	int aux = -1;
    	for (int i=0; i<ncols; i++) for (int j=0; j<nrows; j++){
    		global_map_file >> aux;
			if (map.data[index(i,j)] == unknownOccupancy)
				map.data[index(i,j)] = aux;
			else if( closestRobot(i,j) != robot_number && aux!= unknownOccupancy)
				map.data[index(i,j)] = aux;
		}
		newMap = true;
		publishMap();
    }
}

void aap_mapping::publishPosition(void)
{
	if(comunication==0){
		std_msgs::Float64MultiArray msg;
		msg.data.clear();
		msg.data.resize(3);
		msg.data.assign(3,-1);
		msg.data[0] = robot_number;
		msg.data[1] = robotsPosition.data[robot_number*3+1];
		msg.data[2] = robotsPosition.data[robot_number*3+2];
		position_pub.publish(msg);
	}else{
		static int iterations = ((int)(frequency))*1;
		static int i = iterations - 1;
		i++;
		if(i==iterations){
			i=0;
			std::string file = path_positions + robot_name + ".pos";
			std::ofstream myfile;
			myfile.open(file.c_str(), std::ios::out | std::ios::trunc); 
			myfile << robotsPosition.data[robot_number*3+1] << "\n" << robotsPosition.data[robot_number*3+2] << "\n";
			myfile.close();
		}
	}
}

void aap_mapping::statusReceived(const std_msgs::Float64MultiArray::ConstPtr& msg_ptr)
{
	std::vector<double>::const_iterator msg = msg_ptr->data.begin();

	double aux_code = *msg; 	msg++; msg++;
	double aux_time = *msg;	msg++;
	double aux_code2 = *msg;
	
	if( aux_code == -2.0 && aux_code2 == -2.0){
		close_all = true;
	}else if( aux_code == -1.0 && aux_code2 == -1.0){
		map.data.assign(ncols * nrows, unknownOccupancy);
		map_pub.publish(map);
		map_metadata_pub.publish(map.info);
		idle = true;
	}else if( aux_code == 3.3){
		while( ros::Time::now().toSec() < aux_time )
			ros::Duration(0.5/frequency).sleep();
		idle = false;
	}
}

double aap_mapping::getX(int i)
{
	return(i*resolution + xOrigin);
}

double aap_mapping::getY(int j)
{
	return(j*resolution + yOrigin);
}

int aap_mapping::getI(double x)
{
	int i = (int)((x - xOrigin )/resolution);
	return(i);
}

int aap_mapping::getJ(double y)
{
	int j = (int)((y - yOrigin )/resolution);
	return(j);
}




