#include "coopexp.h"
#include "ros/ros.h"

coopexp::coopexp()
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
	pn.param("inicial_x", inicial_x, 0.0);
	pn.param("inicial_y", inicial_y, 0.0);
	pn.param("inicial_theta", inicial_theta, 0.0);
	pn.param("start_time", start_time, 0.0);
	pn.param("alfa", alfa, 0.125);
	pn.param("beta", beta, 0.8);
	pn.param("gama", gama, 1.0);
	pn.param("omega", omega, 1.0);
	pn.param("comunication", comunication, 0);
	pn.param("real", comunication, 0);
	pn.getParam("number_of_robots", number_of_robots);
	pn.getParam("frequency", frequency);
	pn.getParam("resolution", resolution);
	pn.getParam("height", height);
	pn.getParam("length", length);
	pn.getParam("robot_radius", robot_radius);
	pn.getParam("robot_number", robot_number);
	pn.getParam("robot_name", robot_name);
	pn.getParam("inicial_x", inicial_x);
	pn.getParam("inicial_y", inicial_y);
	pn.getParam("inicial_theta", inicial_theta);
	pn.getParam("start_time", start_time);
	pn.getParam("alfa", alfa);
	pn.getParam("beta", beta);
	pn.getParam("gama", gama);
	pn.getParam("omega", omega);
	pn.getParam("comunication", comunication);
	pn.getParam("real", real);

	pn.param("path", path, std::string("/home"));
	pn.getParam("path", path);

	unknownCell = -1;
	freeCell = 0;
	inaccessibleCell = 75;
	frontierCell = 100;

	timeBufferSize = 20.0; // seconds
	receiveBufferSize = 1; // no. messages
	sendBufferSize = 128; // no. messages

	while(!ready()) ros::Duration(0.5/frequency).sleep();
    start();
}

coopexp::~coopexp()
{

}

bool coopexp::ready(void)
{
	static bool configurated = false;

	if(!configurated){
		configurated = true;
		if(number_of_robots>1) multiRobot = true;
		else multiRobot = false;
		if(multiRobot && comunication==0) idle = true;
		else idle = false;
		hibernate = false;
		errorCostmap = false;
		new_frontiers =  false;
		new_position = false;

		path_positions = path + "/simulator/positions/";
		path_grids = path + "/simulator/grids/";
		robot_name = "/" + robot_name;
		status_topic = "/exploration_status";
		position_topic = "/position";
		if(multiRobot){
			map_frame = robot_name + "/map";
			frontiers_frame = robot_name + "/frontiers";
			costmap_frame = robot_name + "/costmap";
			costmap_metadata_topic =  robot_name + costmap_frame + "_metadata";
			utilitymap_frame = robot_name + "/utilitymap";
			utilitymap_metadata_topic =  robot_name + utilitymap_frame + "_metadata";
			base_link_frame = robot_name + "/base_link";
			markCell_topic = robot_name + "/mark_cell";
		}else{
			map_frame = "/map";
			frontiers_frame = "/frontiers";
			costmap_frame = "/costmap";
			costmap_metadata_topic =  costmap_frame + "_metadata";
			utilitymap_frame = "/utilitymap";
			utilitymap_metadata_topic =  utilitymap_frame + "_metadata";
			base_link_frame = "/base_link";
			markCell_topic = "/mark_cell";
		}

		max_dist = sqrt(height*height+length*length);
		max_cost = max_dist;
		max_utility = 0;

		ncols = (int)length/resolution;
		nrows = (int)height/resolution;

		xOrigin = -length/2;
		yOrigin = -height/2;

		robotsPosition.data.clear();
		robotsPosition.data.resize(number_of_robots*3);
		robotsPosition.data.assign(number_of_robots*3, 0);

		costmap.data.clear();
		costmap.data.resize(nrows*ncols);
		costmap.data.assign(ncols * nrows, unknownCell);

		utilitymap.data.clear();
		utilitymap.data.resize(nrows*ncols);
		utilitymap.data.assign(ncols * nrows, unknownCell);

		costmapOccupancyGrid.info.resolution = resolution; // m/cell
		costmapOccupancyGrid.info.width = ncols; 
		costmapOccupancyGrid.info.height = nrows;
		costmapOccupancyGrid.info.origin.position.x = xOrigin;
		costmapOccupancyGrid.info.origin.position.y = yOrigin;
		costmapOccupancyGrid.info.origin.position.z = 0.0;
		costmapOccupancyGrid.info.origin.orientation.x = 0.0;
		costmapOccupancyGrid.info.origin.orientation.y = 0.0;
		costmapOccupancyGrid.info.origin.orientation.z = 0.0;
		costmapOccupancyGrid.info.origin.orientation.w = 1.0;
		costmapOccupancyGrid.header.frame_id = costmap_frame;
		costmapOccupancyGrid.data.assign(ncols * nrows, unknownCell);
		costmap_pub = n.advertise<nav_msgs::OccupancyGrid>(costmap_frame,sendBufferSize,true);
		costmap_metadata_pub = n.advertise<nav_msgs::MapMetaData>(costmap_metadata_topic,sendBufferSize,true);
		costmap_pub.publish(costmapOccupancyGrid);
		costmap_metadata_pub.publish(costmapOccupancyGrid.info);
		
		utilitymapOccupancyGrid.info.resolution = resolution; // m/cell
		utilitymapOccupancyGrid.info.width = ncols; 
		utilitymapOccupancyGrid.info.height = nrows;
		utilitymapOccupancyGrid.info.origin.position.x = xOrigin;
		utilitymapOccupancyGrid.info.origin.position.y = yOrigin;
		utilitymapOccupancyGrid.info.origin.position.z = 0.0;
		utilitymapOccupancyGrid.info.origin.orientation.x = 0.0;
		utilitymapOccupancyGrid.info.origin.orientation.y = 0.0;
		utilitymapOccupancyGrid.info.origin.orientation.z = 0.0;
		utilitymapOccupancyGrid.info.origin.orientation.w = 1.0;
		utilitymapOccupancyGrid.header.frame_id = utilitymap_frame;
		utilitymapOccupancyGrid.data.assign(ncols * nrows, 0);
		utilitymap_pub = n.advertise<nav_msgs::OccupancyGrid>(utilitymap_frame,sendBufferSize,true);
		utilitymap_metadata_pub = n.advertise<nav_msgs::MapMetaData>(utilitymap_metadata_topic,sendBufferSize,true);
		utilitymap_pub.publish(utilitymapOccupancyGrid);
		utilitymap_metadata_pub.publish(utilitymapOccupancyGrid.info);
		
		map_to_utilitymap_transform = tf::StampedTransform( tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)), ros::Time::now(), map_frame, utilitymap_frame);
		map_to_costmap_transform = tf::StampedTransform( tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)), ros::Time::now(), map_frame, costmap_frame);

		listener = new tf::TransformListener(ros::Duration(timeBufferSize), true);

		if(multiRobot) status_pub = n.advertise<std_msgs::Float64MultiArray>(status_topic, sendBufferSize);
		markCell_pub = n.advertise<std_msgs::Int32MultiArray>(markCell_topic, sendBufferSize);

		ac = new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>("move_base",true);
		goal.target_pose.header.frame_id = map_frame;
	}

	if(configurated){
		if(!multiRobot && ros::Time::now().toSec() < start_time ) return(false);
		return(true);
	}
	
	return(false);
}

void coopexp::start(void)
{
	if(comunication==0 && multiRobot) status_sub = n.subscribe(status_topic, sendBufferSize, &coopexp::statusReceived, this);
	while(!ac->waitForServer(ros::Duration(5.0)))
		ROS_WARN("coopexp-robot_%d: Waiting for the move_base action server to come up", robot_number);
	frontiers_sub = n.subscribe<nav_msgs::OccupancyGrid>(frontiers_frame, receiveBufferSize, &coopexp::frontiersReceived, this);
	if(multiRobot && comunication==0) position_sub = n.subscribe(position_topic, sendBufferSize, &coopexp::positionReceived, this);

	ROS_INFO("coopexp-robot_%d: I'm alive", robot_number);

	ros::Rate loop_rate(frequency);
	while (ros::ok()){
		if(!hibernate){
			updateTf(); 
			if(!idle) updateRobotPosition();
			if(comunication == 1) updatePositions();

			ros::spinOnce();

			if(new_frontiers && new_position){
				new_frontiers = false;
				new_position = false;
				initCostmap();
				updateCostmap(base_link_i,base_link_j,0);
				publishCostmap();
				updateUtilitymap();	
				generateGoal();
			}
		}
		loop_rate.sleep();
	}
}

void coopexp::updateTf(void)
{
	map_to_costmap_transform.stamp_ = ros::Time::now();
	map_to_costmap_broadcaster.sendTransform(map_to_costmap_transform);

	map_to_utilitymap_transform.stamp_ = ros::Time::now();
	map_to_utilitymap_broadcaster.sendTransform(map_to_utilitymap_transform);
}

void coopexp::frontiersReceived(const nav_msgs::OccupancyGrid::ConstPtr& frontiers_ptr)
{
	if(idle) return;
	frontiers = *frontiers_ptr;

	bool newParam = false;

	if(ncols != frontiers.info.width) newParam = true;
	if(nrows != frontiers.info.height) newParam = true;
	if(frontiers.info.origin.position.x != frontiers.info.origin.position.x) newParam = true;
	if(frontiers.info.origin.position.y != frontiers.info.origin.position.y) newParam = true;

	if(newParam){ 
		ncols = frontiers.info.width; 
		nrows = frontiers.info.height;

		xOrigin = frontiers.info.origin.position.x; 
		yOrigin = frontiers.info.origin.position.y;

		frontiers.info.width = ncols; 
		frontiers.info.height = nrows;
		frontiers.info.origin.position.x = xOrigin;
		frontiers.info.origin.position.y = yOrigin;
		frontiers.data.resize(nrows * ncols);
		frontiers.data.assign(ncols * nrows, unknownCell);

		costmap.data.clear();
		costmap.data.resize(nrows*ncols);
		costmap.data.assign(ncols * nrows, unknownCell);

		costmapOccupancyGrid.info.width = ncols; 
		costmapOccupancyGrid.info.height = nrows;
		costmapOccupancyGrid.info.origin.position.x = xOrigin;
		costmapOccupancyGrid.info.origin.position.y = yOrigin;
		costmapOccupancyGrid.data.assign(ncols * nrows, unknownCell);
		
		utilitymap.data.clear();
		utilitymap.data.resize(nrows*ncols);
		utilitymap.data.assign(ncols * nrows, unknownCell);

		utilitymapOccupancyGrid.info.width = ncols; 
		utilitymapOccupancyGrid.info.height = nrows;
		utilitymapOccupancyGrid.info.origin.position.x = xOrigin;
		utilitymapOccupancyGrid.info.origin.position.y = yOrigin;
		utilitymapOccupancyGrid.data.assign(ncols * nrows, 0);
	}

	new_frontiers = true;
}

void coopexp::initCostmap(void)
{
	costmap.data.assign(ncols * nrows, unknownCell);

	int i = base_link_i, j =base_link_j;
	int cellRadius =  ceil(robot_radius/resolution);
	int i_min, i_max, j_min, j_max;
	if(i - cellRadius <= 0) i_min = 0; else i_min = i - cellRadius;
	if(i + cellRadius >= ncols) i_max = ncols - 1; else i_max = i + cellRadius;
	if(j - cellRadius <= 0) j_min = 0; else j_min = j - cellRadius;
	if(j + cellRadius >= nrows) j_max = nrows - 1; else j_max = j + cellRadius;

	for (int i=i_min; i<=i_max; i++) for (int j=j_min; j<=j_max; j++)
		costmap.data[index(i, j)] = 0;
}

void coopexp::updateCostmap(int i, int j, int counter){
	int noOfIterations = nrows, squareSize = 3, positiveCostCellsCounter = 0;
	if(ncols>nrows) noOfIterations = ncols;
	bool incompleteCostmap = false;

	i--;j--;
	for (int ite=0; ite<noOfIterations; ite++,i--,j--,squareSize+=2)
		for (int n=0; n<2; n++) for (int m=0; m<4; m++) for(int k=0; k<squareSize; k++){
			if(i>=0 && i<ncols && j>=0 && j<nrows){
				costmap.data[index(i,j)] = minCost(i,j);
				if(n==1){
					if(costmap.data[index(i,j)] == -1) incompleteCostmap = true;
					if(costmap.data[index(i,j)] >= 0) positiveCostCellsCounter++;
				}
			}

			if(n==0 && k<squareSize-1){
				if(m==0) i++;
				else if(m==1) j++;
				else if(m==2) i--;
				else if(m==3) j--;
			}
			else if(n==1 && k<squareSize-1){
				if(m==0) j++;
				else if(m==1) i++;
				else if(m==2) j--;
				else if(m==3) i--;
			}
		}

	if(positiveCostCellsCounter >= (0.1*ncols*nrows)){
		errorCostmap = false;
		if(incompleteCostmap){
			double minCostValue = max_cost*2;
			int startPoint_i, startPoint_j;
			bool newStartPoint = false;
			for (i=0; i<ncols; i++) for (j=0; j<nrows; j++) if(costmap.data[index(i,j)] == -1){
				costmap.data[index(i,j)] = minCost(i,j);
				if( 0<=costmap.data[index(i,j)] && costmap.data[index(i,j)]<minCostValue){ 
						minCostValue = costmap.data[index(i,j)];
						startPoint_i = i;
						startPoint_j = j;
						newStartPoint = true;
					}
			}	

			if(newStartPoint && counter<20) {
				//ROS_INFO("coopexp-robot_%d: counter %d", robot_number, counter);
				counter++;
				updateCostmap(startPoint_i, startPoint_j, counter);
			}/*else{
				for (i=ncols-1; i>=0; i--) for (j=nrows-1; j>=0; j--)
					costmap.data[index(i,j)] = minCost(i,j);
			}*/
		}
	}else{
		errorCostmap = true;
		ROS_WARN("coopexp-robot_%d: errorCostmap", robot_number);
	}
}

void coopexp::publishCostmap(void){
	double local_max = 0, cost;
	for (int i=0; i<ncols; i++) for (int j=0; j<nrows; j++){
		cost = costmap.data[index(i,j)];
		if(cost > local_max ) local_max = cost;
		if(cost == -1) costmapOccupancyGrid.data[index(i,j)] = -1;
		else if(cost == -2) costmapOccupancyGrid.data[index(i,j)] = 100;
		else costmapOccupancyGrid.data[index(i,j)] = (int)(cost*100/max_cost);
	}

	if(!errorCostmap) max_cost = local_max;

	costmap_pub.publish(costmapOccupancyGrid);
}

double coopexp::minCost(int i, int j)
{
	if(frontiers.data[index(i,j)]==50) return(-2);

	double value, dist, minCostValue = max_cost*2;
	int i_c = i, j_c = j;

	int cellRadius = 1;
	int i_min, i_max, j_min, j_max;
	if(i - cellRadius <= 0) i_min = 0; else i_min = i - cellRadius;
	if(i + cellRadius >= ncols) i_max = ncols - 1; else i_max = i + cellRadius;
	if(j - cellRadius <= 0) j_min = 0; else j_min = j - cellRadius;
	if(j + cellRadius >= nrows) j_max = nrows - 1; else j_max = j + cellRadius;

	for (i=i_min; i<=i_max; i++) for (j=j_min; j<=j_max; j++)
		if(frontiers.data[index(i,j)]!=50 || costmap.data[index(i,j)]==0) if(costmap.data[index(i,j)]>=0){
			dist = sqrt( pow(getX(i_c)-getX(i),2) + pow(getY(j_c)-getY(j),2) );
			value = costmap.data[index(i,j)] + dist;
			if(minCostValue>value) minCostValue = value;
		}
		
	if(minCostValue != (max_cost*2)){
		if(minCostValue > max_cost) max_cost = minCostValue;
		return(minCostValue);
	}else return(-1);
}

void coopexp::updateUtilitymap(void){
	max_utility = 0;
	utilitymap.data.assign(ncols * nrows, 0);

	for (int i=0; i<ncols; i++) for (int j=0; j<nrows; j++)
		if(frontiers.data[index(i,j)]==frontierCell){
			if(multiRobot) utilitymap.data[index(i,j)] = noFrontiers(i,j)*alfa + minDistOtherRobots(i,j)*beta;
			else utilitymap.data[index(i,j)] = noFrontiers(i,j)*alfa;
			if(utilitymap.data[index(i,j)] > max_utility)
				max_utility = utilitymap.data[index(i,j)];
		}
	

	for (int i=0; i<ncols; i++) for (int j=0; j<nrows; j++)
		utilitymapOccupancyGrid.data[index(i,j)] = (int)(utilitymap.data[index(i,j)]*100/max_utility);

	utilitymap_pub.publish(utilitymapOccupancyGrid);
}

double coopexp::minDistOtherRobots(int i, int j)
{ 
	double x = getX(i), y = getY(j), minDist = max_dist;

	for (int k=0; k<number_of_robots; k++) if(k!=robot_number) {
		double dist = sqrt(pow(x-robotsPosition.data[k*3+1],2)+pow(y-robotsPosition.data[k*3+2],2));
		if(dist<minDist) minDist=dist;
	}

	return(minDist);
}

int coopexp::noFrontiers(int i, int j)
{ 
	int count = 0;
	int cellRadius =  ceil(2*robot_radius/resolution);
	int i_min, i_max, j_min, j_max;
	if(i - cellRadius <= 0) i_min = 0; else i_min = i - cellRadius;
	if(i + cellRadius >= ncols) i_max = ncols - 1; else i_max = i + cellRadius;
	if(j - cellRadius <= 0) j_min = 0; else j_min = j - cellRadius;
	if(j + cellRadius >= nrows) j_max = nrows - 1; else j_max = j + cellRadius;

	for (i=i_min; i<=i_max; i++) for (j=j_min; j<=j_max; j++)
		if(frontiers.data[index(i,j)]==frontierCell) count++;

	return(count);
}

void coopexp::generateGoal(void)
{
	bool haveGoal = false;
	int i_goal, j_goal;
	double x, y, theta, cost, maxGoalValue = -max_dist*3, goalValue;

	for (int i=0; i<ncols; i++) for (int j=0; j<nrows; j++)
		if(frontiers.data[index(i,j)]==frontierCell){
			cost = costmap.data[index(i,j)];
			if (errorCostmap) cost = 0;
			else if (cost <= -1) cost = max_cost*2;
			goalValue = utilitymap.data[index(i,j)]*gama - cost*omega;
			if(goalValue>maxGoalValue){
				maxGoalValue = goalValue;
				haveGoal = true;
				x = getX(i);
				y = getY(j);
				i_goal=i;
				j_goal=j;
				theta = atan2(y - base_link_y, x - base_link_x);
			}
		}

	if(haveGoal){  
		//ROS_INFO("coopexp-robot_%d: haveGoal %lf %lf", robot_number,x,y);
		//ROS_INFO("coopexp-robot_%d: utility= %lf  cost= %lf ", robot_number, utilitymap.data[index(i_goal,j_goal)], costmap.data[index(i_goal,j_goal)] ); 
		//ROS_INFO("coopexp-robot_%d: noFrontiers= %d and minDistOtherRobots= %lf ", robot_number, noFrontiers(i_goal,j_goal), minDistOtherRobots(i_goal,j_goal) );
		if(errorCostmap==false && costmap.data[index(i_goal,j_goal)]<0)
			markFrontiers();
		sendGoal(x,y,theta); 
	}else if( ros::Time::now().toSec() > 30.0 ){
		int unknownCells = 0;
		for (int i=0; i<ncols; i++) for (int j=0; j<nrows; j++)
			if(frontiers.data[index(i,j)]==unknownCell)
				unknownCells++;

		double auxTime = (ros::Time::now().toSec()) - startTime;
		double auxRatio = (nrows*ncols);
		auxRatio = (auxRatio-unknownCells)/auxRatio;
		if(multiRobot && comunication==0) sendStatus(1.1,rep,auxTime,auxRatio);
		if(multiRobot && comunication==0) sendStatus(-1.0, 0.0, 0.0, -1.0);
		if(comunication==1) ROS_INFO("coopexp-robot_%d: Finish in %lf(s), map ratio is %lf", robot_number, auxTime, auxRatio);
		sendGoal(getX(base_link_i),getY(base_link_j),theta);
	} 
}

void coopexp::updateRobotPosition(void)
{
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

	if(!listener->waitForTransform(map_frame, base_link_frame, time_stamp,ros::Duration(1.0))){
		ROS_WARN("coopexp-robot_%d: no transform at this time", robot_number);
		return;
	}	

	try{
		listener->transformPoint(map_frame, time_stamp, base_link_point, map_frame, map_point);
		base_link_x = map_point.point.x;
		base_link_y = map_point.point.y;
		base_link_i = getI(base_link_x);
		base_link_j = getJ(base_link_y);
		robotsPosition.data[robot_number*3] = robot_number;
		robotsPosition.data[robot_number*3+1] = base_link_x;
		robotsPosition.data[robot_number*3+2] = base_link_y;
		new_position = true;
	}catch(tf::TransformException& ex){
		new_position = false;
		ROS_ERROR("coopexp-robot_%d: %s", robot_number, ex.what());
	}
}

void coopexp::sendGoal(double x, double y, double theta)
{
	//ROS_INFO("coopexp-robot_%d: sendGoal(%lf, %lf)", robot_number, x, y);

	goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose.pose.position.x = x;
	goal.target_pose.pose.position.y = y;
	goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);

	static double last_goal_x = 0;
	static double last_goal_y = 0;
	static double last_position_x = 0;
	static double last_position_y = 0;

	ac->sendGoal(goal);
	if(last_goal_x == x && last_goal_y == y && base_link_x == last_position_x && base_link_y == last_position_y)
		markCell(getI(x),getJ(y),inaccessibleCell);

	last_position_x = base_link_x;
	last_position_y = base_link_y;
	last_goal_x = x;
	last_goal_y = y;
}

void coopexp::sendStatus(double codeT, double repT, double timeT, double cellT)
{
	std_msgs::Float64MultiArray status_out;
	status_out.data.clear();
	status_out.data.push_back(codeT);
	status_out.data.push_back(repT);
	status_out.data.push_back(timeT);
	status_out.data.push_back(cellT);
	status_out.data.push_back(omega);
	status_pub.publish(status_out);
}

void coopexp::markCell(int i, int j, int value)
{
	std_msgs::Int32MultiArray msg;
	msg.data.clear();
	msg.data.push_back(0);
	msg.data.push_back(inaccessibleCell);

	int cellRadius = ceil(2*robot_radius/resolution);
	int i_min, i_max, j_min, j_max;
	if(i - cellRadius <= 0) i_min = 0; else i_min = i - cellRadius;
	if(i + cellRadius >= ncols) i_max = ncols - 1; else i_max = i + cellRadius;
	if(j - cellRadius <= 0) j_min = 0; else j_min = j - cellRadius;
	if(j + cellRadius >= nrows) j_max = nrows - 1; else j_max = j + cellRadius;

	for (i=i_min; i<=i_max; i++) for (j=j_min; j<=j_max; j++)
		if(frontiers.data[index(i,j)]== frontierCell){
			msg.data.push_back(i);
			msg.data.push_back(j);
			msg.data[0]++;
		}

	if(msg.data[0]>0) markCell_pub.publish(msg);
}

void coopexp::markFrontiers(void)
{
	std_msgs::Int32MultiArray msg;
	msg.data.clear();
	msg.data.push_back(0);
	msg.data.push_back(inaccessibleCell);

	for (int i=0; i<ncols; i++) for (int j=0; j<nrows; j++)
		if(frontiers.data[index(i,j)]==frontierCell)
			if(costmap.data[index(i,j)]<0){
				msg.data.push_back(i);
				msg.data.push_back(j);
				msg.data[0]++;
			}

	if(msg.data[0]>0) markCell_pub.publish(msg);
}

int coopexp::index(int i, int j)
{
	return(j*ncols + i);
}

void coopexp::statusReceived(const std_msgs::Float64MultiArray::ConstPtr& msg_ptr)
{
	std::vector<double>::const_iterator msg = msg_ptr->data.begin();

	double aux_code = *msg; 	msg++; 
	double aux_rep = *msg;		msg++;
	double aux_time = *msg;		msg++;
	double aux_code2 = *msg;
	
	if( aux_code == -2.0 && aux_code2 == -2.0){
		hibernate = true;
		sendGoal(getX(base_link_i),getY(base_link_j),0.0);
	}else if( aux_code == -1.0 && aux_code2 == -1.0){
		costmap.data.assign(ncols * nrows, unknownCell);
		utilitymap.data.assign(ncols * nrows, unknownCell);
		frontiers.data.assign(ncols * nrows, unknownCell);
		costmap_pub.publish(costmapOccupancyGrid);
		utilitymap_pub.publish(utilitymapOccupancyGrid);
		new_frontiers = false;
		new_position = false;
		idle = true;
	}else if( aux_code == 5.5){
		rep = aux_rep;
		startTime = aux_time;
		while( ros::Time::now().toSec() < startTime )ros::Duration(0.5/frequency).sleep();
		idle = false;
	}
}

void coopexp::positionReceived (const std_msgs::Float64MultiArray::ConstPtr& msg_ptr)
{
	if(idle) return;

	std::vector<double>::const_iterator msg = msg_ptr->data.begin();

	int aux_robot_number = (int)(*msg); 
	if(robot_number == aux_robot_number) return;

	robotsPosition.data[aux_robot_number*3] = *msg;
	msg++;
	robotsPosition.data[aux_robot_number*3+1] = *msg;
	msg++;
	robotsPosition.data[aux_robot_number*3+2] = *msg;
}

void coopexp::updatePositions(void)
{
	for(int i = 0;i<number_of_robots;i++)
		if(i!=robot_number)
			while(readPos(i) == false)
				ros::Duration(0.5).sleep();
}

bool coopexp::readPos(int i)
{
	std::ostringstream robot_name_ostr;
	robot_name_ostr << "robot_" << i;
	const std::string robot_name_aux(robot_name_ostr.str());
	std::string file = path_positions + robot_name_aux + ".pos";

    std::ifstream ifile(file.c_str(), std::ios::in);

    if (!ifile.is_open()) {
    	ROS_WARN("coopexp-robot_%d: There was a problem opening the file %s", robot_number, file.c_str());
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

double coopexp::getX(int i)
{
	return(i*resolution + xOrigin);
}

double coopexp::getY(int j)
{
	return(j*resolution + yOrigin);
}

int coopexp::getI(double x)
{
	int i = (int)((x - xOrigin )/resolution);
	return(i);
}

int coopexp::getJ(double y)
{
	int j = (int)((y - yOrigin )/resolution);
	return(j);
}

