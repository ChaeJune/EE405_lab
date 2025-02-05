#include "motion_primitives_planner/motion_primitives_planner_node.hpp"

/* ----- Class Functions ----- */
MotionPlanner::MotionPlanner(ros::NodeHandle& nh) : nh_(nh)
{
  // Subscriber
  subOccupancyGrid = nh.subscribe("/map/local_map/obstacle",1, &MotionPlanner::CallbackOccupancyGrid, this);
  // Publisher
  pubSelectedMotion = nh_.advertise<sensor_msgs::PointCloud2>("/points/selected_motion", 1, true);
  pubMotionPrimitives = nh_.advertise<sensor_msgs::PointCloud2>("/points/motion_primitives", 1, true);
  //pubCommand = nh_.advertise<ackermann_msgs::AckermannDrive>("/car_1/command", 1, true);
  pubCommandSt = nh_.advertise<std_msgs::Int16>("/auto_cmd/steer", 1, true);
  pubCommandTh = nh_.advertise<std_msgs::Int16>("/auto_cmd/throttle", 1, true);
  pubCommandMo = nh_.advertise<std_msgs::Bool>("/auto_mode", 1, true);
  
};

MotionPlanner::~MotionPlanner() 
{    
    ROS_INFO("MotionPlanner destructor.");
}

/* ----- ROS Functions ----- */

void MotionPlanner::CallbackOccupancyGrid(const nav_msgs::OccupancyGrid& msg)
{
  // Subscribe to the map messages
  localMap = msg;
  // The origin of the map [m, m, rad].  This is the real-world pose of the cell (0,0) in the map.
  this->origin_x = msg.info.origin.position.x;
  this->origin_y = msg.info.origin.position.y;
  // Frame id of the map
  this->frame_id = msg.header.frame_id;
  // The map resolution [m/cell]
  this->mapResol = msg.info.resolution;
  // message flag
  bGetMap = true;
}

void MotionPlanner::PublishSelectedMotion(std::vector<Node> motionMinCost)
{
  // publish selected motion primitive as point cloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in_ptr(new pcl::PointCloud<pcl::PointXYZI>);

  for (auto motion : motionMinCost) {
    pcl::PointXYZI pointTmp;
    pointTmp.x = motion.x;
    pointTmp.y = motion.y;
    cloud_in_ptr->points.push_back(pointTmp);
  }

  sensor_msgs::PointCloud2 motionCloudMsg;
  pcl::toROSMsg(*cloud_in_ptr, motionCloudMsg);
  motionCloudMsg.header.frame_id = this->frame_id;
  motionCloudMsg.header.stamp = ros::Time::now();
  pubSelectedMotion.publish(motionCloudMsg);
}

void MotionPlanner::PublishMotionPrimitives(std::vector<std::vector<Node>> motionPrimitives)
{
  // publish motion primitives as point cloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in_ptr(new pcl::PointCloud<pcl::PointXYZI>);

  for (auto& motionPrimitive : motionPrimitives) {
    for (auto motion : motionPrimitive) {
      pcl::PointXYZI pointTmp;
      pointTmp.x = motion.x;
      pointTmp.y = motion.y;
      cloud_in_ptr->points.push_back(pointTmp);
    }
  }
  
  sensor_msgs::PointCloud2 motionPrimitivesCloudMsg;
  pcl::toROSMsg(*cloud_in_ptr, motionPrimitivesCloudMsg);
  motionPrimitivesCloudMsg.header.frame_id = this->frame_id;
  motionPrimitivesCloudMsg.header.stamp = ros::Time::now();
  pubMotionPrimitives.publish(motionPrimitivesCloudMsg);
}

void MotionPlanner::PublishCommand(std::vector<Node> motionMinCost)
{
  // Check if the motion primitive is not empty
  if (!motionMinCost.empty()) {
    // Get the last node of the motion primitive (assuming it represents the final state)
    Node finalState = motionMinCost.back();

    // Example: Simple proportional controller for steering angle
    double steeringAngle = finalState.delta;
    
    // Example: Constant speed command
    double speed = 0.2882;
    // 0.2

    // Create and publish the AckermannDrive message
   // ackermann_msgs::AckermannDrive command;
   // command.steering_angle = steeringAngle;
   // command.speed = speed;
    std_msgs::Int16 steer;
    std_msgs::Int16 throttle;
    std_msgs::Bool mode;
    steer.data = 1500 - 400 * (steeringAngle)/(10*M_PI/180);
    throttle.data = 1450;
    mode.data = true;
    pubCommandSt.publish(steer);
    pubCommandTh.publish(throttle);
    pubCommandMo.publish(mode);

    std::cout << "Command steer/speed: " << steer.data  << " " << throttle.data << std::endl;
  }
  else {
      std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! mincost is empty"<< std::endl;

  }
}


/* ----- Algorithm Functions ----- */

void MotionPlanner::Plan()
{
  // Motion generation
  //std::cout << "before GenerateMotionPrimitives" << std::endl;
  std::vector<std::vector<Node>> motionPrimitives = GenerateMotionPrimitives(this->localMap);
  
  // Select motion
  //std::cout << "before SelectMotion" << std::endl;
  std::vector<Node> motionMinCost = SelectMotion(motionPrimitives);

  // Publish data
  //std::cout << "before PublishData" << std::endl;
  PublishData(motionMinCost, motionPrimitives);
}

std::vector<std::vector<Node>> MotionPlanner::GenerateMotionPrimitives(nav_msgs::OccupancyGrid localMap)
{
  /*
    TODO: Generate motion primitives
    - you can change the below process if you need.
    - you can calculate cost of each motion if you need.
  */

  // initialize motion primitives
  std::vector<std::vector<Node>> motionPrimitives;

  // number of candidates
  int num_candidates = this->MAX_DELTA*2 / this->DELTA_RESOL; // *2 for considering both left/right direction

  // max progress of each motion
  double maxProgress = this->MAX_PROGRESS;
  for (int i=0; i<num_candidates+1; i++) {
    // current steering delta
    double angle_delta = this->MAX_DELTA - i * this->DELTA_RESOL;

    // init start node
    Node startNode(0, 0, 0, angle_delta, 0, 0, 0, -1, false);
    
    // rollout to generate motion
    std::vector<Node> motionPrimitive = RolloutMotion(startNode, maxProgress, localMap);

    // add current motionPrimitive
    motionPrimitives.push_back(motionPrimitive);
  }

  return motionPrimitives;
}

std::vector<Node> MotionPlanner::RolloutMotion(Node startNode,
                                              double maxProgress,
                                              nav_msgs::OccupancyGrid localMap)
{
  // Initialize motion primitive
  std::vector<Node> motionPrimitive;

  // Initialize current motion node
  Node currMotionNode(startNode.x, startNode.y, 0, startNode.delta, 0, 0, 0, -1, false);

  // Start from small progress value
  double progress = this->DIST_RESOL;
  Node prevMotionNode(currMotionNode.x, currMotionNode.y, 0, currMotionNode.delta, 0, 0, 0, -1, false);

  // While loop until maximum progress of a motion
  while (progress < maxProgress) {
    // Update motion node using the current steering angle delta based on the vehicle kinematics equation
    double steeringAngle = currMotionNode.delta;
    double dt = this->TIME_RESOL;

    double vel= this->MOTION_VEL * cos(currMotionNode.yaw) * dt;
    double uel = this->MOTION_VEL * sin(currMotionNode.yaw) * dt;
    currMotionNode.x += vel; 
    currMotionNode.y += uel;
    currMotionNode.yaw += (this->MOTION_VEL / this->WHEELBASE) * tan(steeringAngle) * dt;
    //currMotionNode.delta += atan2(currMotionNode.y - prevMotionNode.y, currMotionNode.x - prevMotionNode.x);

    // Collision checking
    
    

    Node collisionPointNode(currMotionNode.x, currMotionNode.y, currMotionNode.yaw, currMotionNode.delta, 0, 0, 0, -1, false);
    Node collisionPointNodeMap = LocalToMapCorrdinate(collisionPointNode);
    
    if (CheckCollision(collisionPointNodeMap, this->localMap)) {
      // Do some process when collision occurs.
      // You can save collision information & calculate collision cost here.
      // You can break and return the current motion primitive or keep generating rollout.
        currMotionNode.cost_colli = 1;
        motionPrimitive.push_back(currMotionNode);
        break;
    }


    // Range checking
    
    /*
    double LOS_DIST = sqrt(currMotionNode.x * currMotionNode.x + currMotionNode.y * currMotionNode.y);
    double LOS_YAW = atan2(currMotionNode.y, currMotionNode.x);

    if (LOS_DIST > this->MAX_SENSOR_RANGE || std::abs(LOS_YAW) > this->FOV * 0.5) {
      // Do some process when out-of-range occurs.
      // You can break and return the current motion primitive or keep generating rollout.
    }
    */

    // Append collision-free motion in the current motionPrimitive
    motionPrimitive.push_back(currMotionNode);

    // Update progress of motion
    progress += this->DIST_RESOL;
  }

  // Return the current motion primitive
  return motionPrimitive;
}

std::vector<Node> MotionPlanner::SelectMotion(std::vector<std::vector<Node>> motionPrimitives)
{
  // Initialization
  double minCost = std::numeric_limits<double>::max();
  std::vector<Node> motionMinCost; // Initialize as empty motion

  // Check the size of motion primitives
  if (!motionPrimitives.empty()) {
    // Iterate through all motion primitives
    std::cout << "before motionPrimitives" << std::endl;
    for (auto& motionPrimitive : motionPrimitives) {
      // 1. Calculate cost terms
      //std::cout << "before CalculateCollisionCost" << std::endl;
      //double collisionCost = CalculateCollisionCost(motionPrimitive);
      double collisionCost = motionPrimitive.back().cost_colli;
      //std::cout << "before CalculateSteeringCost" << std::endl;
      double steeringCost = CalculateSteeringCost(motionPrimitive);
      //std::cout << "before CalculateProgressCost" << std::endl;
      double progressCost = CalculateProgressCost(motionPrimitive);

      // 2. Calculate total cost (weighted sum of all cost terms)
      WCOLLISION = 1.47;
      //0.23
      WSTEERING = 1.07;
      WPROGRESS = 0.78;

      double totalCost = WCOLLISION * collisionCost + WSTEERING * steeringCost + WPROGRESS * progressCost;

      std::cout << "Collision Cost" << collisionCost << std::endl;
      std::cout << "SteeringCost" << steeringCost<< std::endl;
      std::cout << "ProgressCost" << progressCost<<std::endl;
      std::cout << "totalCost : " << totalCost << std::endl;

      // 3. Compare & Find minimum cost & minimum cost motion
      if (totalCost < minCost) {
        motionMinCost = motionPrimitive;
        minCost = totalCost;
      }
    }
    std::cout << "after motionPrimitives" << std::endl;
  }

  // 4. Return minimum cost motion
  return motionMinCost;
}

double MotionPlanner::CalculateCollisionCost(const std::vector<Node>& motionPrimitive)
{ 
  double collisionCost = 0.0;

  for (const auto& motionNode : motionPrimitive) {
    // Check if the current motion node collides with an obstacle
    Node collisionPointNodeMap = LocalToMapCorrdinate(motionNode);
    if (CheckCollision(collisionPointNodeMap, this->localMap)) {
      // Collision detected, calculate distance to the nearest obstacle
      double distanceToObstacle = CalculateDistanceToObstacle(motionNode, this->localMap);
      
      // Accumulate the collision cost based on the distance to the nearest obstacle
      if(distanceToObstacle > 0.2){
      collisionCost += AccumulateOfDistance(distanceToObstacle);
      }
    }
  }

  return collisionCost;
}

double MotionPlanner::CalculateDistanceToObstacle(const Node& motionNode, const nav_msgs::OccupancyGrid& localMap)
{
    // Get the size of the local map
    int mapWidth = localMap.info.width;
    int mapHeight = localMap.info.height;

    // Get the coordinates of the motion node in map coordinates
    Node collisionPointNodeMap = LocalToMapCorrdinate(motionNode);
    int mapX = collisionPointNodeMap.x;
    int mapY = collisionPointNodeMap.y;

    // Iterate through the map and find the nearest obstacle
    double minDistance = std::numeric_limits<double>::max();
    for (int i = 0; i < mapWidth; ++i) {
        for (int j = 0; j < mapHeight; ++j) {
            int mapIndex = j * mapWidth + i;
            int mapValue = static_cast<int>(localMap.data[mapIndex]);

            if (mapValue > OCCUPANCY_THRES) {  // Assuming OCCUPANCY_THRES represents occupied space
                // Calculate Euclidean distance to the obstacle
                double distance = std::sqrt(std::pow((i - mapX),2) + std::pow((j - mapY),2));

                // Update the minimum distance if the current obstacle is closer
                if (distance < minDistance) {
                    minDistance = distance;
                }
            }
        }
    }

    return minDistance;
}

double MotionPlanner::AccumulateOfDistance(double distance)
{
    const double scaleFactor = 1.0;  // Adjust this scale factor as needed
    return scaleFactor / distance;
}

double MotionPlanner::CalculateSteeringCost(const std::vector<Node>& motionPrimitive)
{
    double steeringCost = 0.0;

    // Iterate through the motion primitive to calculate the steering cost
    /*
    for (size_t i = 1; i < motionPrimitive.size(); ++i) {
        // Calculate the change in steering angle between consecutive nodes
        double deltaSteering = std::abs(motionPrimitive[i].delta);
	std::cout << "deltaSteering : " << deltaSteering << std::endl;
        // Accumulate the steering cost based on the change in steering angle
        steeringCost = deltaSteering;
    }
    */
    double deltaSteering = std::abs(motionPrimitive[0].delta);
    steeringCost = deltaSteering;
    return steeringCost;
}

double MotionPlanner::AccumulateOfSteering(double deltaSteering)
{
    const double divd = 2.52F; 
    return divd / deltaSteering;
}

double MotionPlanner::CalculateProgressCost(const std::vector<Node>& motionPrimitive)
{
    double progressCost = 0.0;

    // Iterate through the motion primitive to calculate the progress cost
    for (size_t i = 1; i < motionPrimitive.size(); ++i) {
        // Calculate the distance between consecutive nodes
        double distanceCovered = CalculateDistanceBetweenNodes(motionPrimitive[i - 1], motionPrimitive[i]);

        // Accumulate the progress cost based on the distance covered
        progressCost -= AccumulateOfProgress(distanceCovered);
    }

    return progressCost;
}

double MotionPlanner::CalculateDistanceBetweenNodes(const Node& node1, const Node& node2)
{
    // Calculate Euclidean distance between two nodes
    return std::sqrt((node2.x - node1.x) * (node2.x - node1.x) +
                     (node2.y - node1.y) * (node2.y - node1.y));
}

double MotionPlanner::AccumulateOfProgress(double distanceCovered)
{

    const double mult = 1;  
    return distanceCovered;
}




/* ----- Util Functions ----- */

bool MotionPlanner::CheckCollision(Node currentNodeMap, nav_msgs::OccupancyGrid localMap)
{
  // Get the size of the local map
  int mapWidth = localMap.info.width;
  int mapHeight = localMap.info.height;

  // Define the inflation size and occupancy threshold
  int inflationSize = this->INFLATION_SIZE;
  int occupancyThres = this->OCCUPANCY_THRES;

  // Loop through the inflation area around the current node
  for (int i = 0; i < inflationSize; ++i) {
    for (int j = 0; j < inflationSize; ++j) {
      // Calculate temporary coordinates
      double tmpX = currentNodeMap.x + i - 0.5 * inflationSize;
      double tmpY = currentNodeMap.y + j - 0.5 * inflationSize;

      // Check if the temporary coordinates are within the map boundaries
      if (tmpX >= 0 && tmpX < mapWidth && tmpY >= 0 && tmpY < mapHeight) {
        // Calculate the index of the grid at the temporary position
        int mapIndex = static_cast<int>(tmpY) * mapWidth + static_cast<int>(tmpX);

        // Get the map value at the calculated index
        int mapValue = static_cast<int>(localMap.data[mapIndex]);

        // Check for collision based on occupancy threshold
        if (mapValue > occupancyThres || mapValue < 0) {
          return true; // Collision detected
        }
      }
    }
  }

  return false; // No collision detected
}

bool MotionPlanner::CheckRunCondition()
{
  if (this->bGetMap) {
    return true;
  }
  else {
    std::cout << "Run condition is not satisfied!!!" << std::endl;
    return false;
  }
}

Node MotionPlanner::LocalToMapCorrdinate(Node nodeLocal)
{
  // Copy data from nodeLocal to nodeMap
  Node nodeMap;
  memcpy(&nodeMap, &nodeLocal, sizeof(struct Node));

  // Get map resolution
  double mapResolution = this->mapResol;

  // Transform x and y from local coordinates to map coordinates
  nodeMap.x = static_cast<int>((nodeLocal.x - this->origin_x) / mapResolution);
  nodeMap.y = static_cast<int>((nodeLocal.y - this->origin_y) / mapResolution);

  return nodeMap;
}


/* ----- Publisher ----- */

void MotionPlanner::PublishData(std::vector<Node> motionMinCost, std::vector<std::vector<Node>> motionPrimitives)
{
  // Publisher
  // - visualize selected motion primitive
  PublishSelectedMotion(motionMinCost);

  // - visualize motion primitives
  PublishMotionPrimitives(motionPrimitives);

  // - publish command
  


PublishCommand(motionMinCost);
}

/* ----- Main ----- */

int main(int argc, char* argv[])
{ 
  std::cout << "start main process" << std::endl;

  ros::init(argc, argv, "motion_primitives_planner");
  // for subscribe
  ros::NodeHandle nh;
  ros::Rate rate(50.0);
  MotionPlanner MotionPlanner(nh);

  // Planning loop
  while (MotionPlanner.nh_.ok()) {
      // Spin ROS
      ros::spinOnce();
      // check run condition
      if (MotionPlanner.CheckRunCondition()) {
        // Run algorithm
        MotionPlanner.Plan();
      }
      rate.sleep();
  }

  return 0;

}
