#include "rrt_planner/rrt_planner.h"

namespace rrt_planner
{

RRTPlanner::RRTPlanner(ros::NodeHandle * node)
: nh_(node),
  private_nh_("~"),
  map_received_(false),
  init_pose_received_(false),
  goal_received_(false)
{
  // Get map and path topics from parameter server
  std::string map_topic, path_topic;
  bool visualize, use_star;
  int expand_dis, disk_size; //expanding distance
  private_nh_.param<std::string>("map_topic", map_topic, "/map");
  private_nh_.param<std::string>("path_topic", path_topic, "/path");
  nh_->param<bool>("visualize_plan",visualize,true);
  nh_->param<bool>("use_star",use_star,false);
  nh_->param<int>("expand_dis", expand_dis,10);
  nh_->param<int>("disk_size",disk_size,50);


  // Subscribe to map topic
  map_sub_ = nh_->subscribe<const nav_msgs::OccupancyGrid::Ptr &>(
    map_topic, 1, &RRTPlanner::mapCallback, this);

  // Subscribe to initial pose topic that is published by RViz
  init_pose_sub_ = nh_->subscribe<const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &>(
    "/initialpose", 1, &RRTPlanner::initPoseCallback, this);

  // Subscribe to goal topic that is published by RViz
  goal_sub_ = nh_->subscribe<const geometry_msgs::PoseStamped::ConstPtr &>(
    "/move_base_simple/goal", 1, &RRTPlanner::goalCallback, this);

  // Advertise topic where calculated path is going to be published
  path_pub_ = nh_->advertise<nav_msgs::Path>(path_topic, 1, true);

  // This loops until the node is running, will exit when the node is killed
  while (ros::ok()) {
    // if map, initial pose, and goal have been received
    // build the map image, draw initial pose and goal, and plan
    if (map_received_ && init_pose_received_ && goal_received_) {
      buildMapImage();
      drawGoalInitPose();
      plan(expand_dis, disk_size, visualize, use_star);
    } else {
      if (map_received_) {
        displayMapImage();
      }
      ros::Duration(0.1).sleep();
      ros::spinOnce();
    }
  }
}

void RRTPlanner::mapCallback(const nav_msgs::OccupancyGrid::Ptr & msg)
{
  map_grid_ = msg;

  // Build and display the map image
  buildMapImage();
  displayMapImage();

  // Reset these values for a new planning iteration
  map_received_ = true;
  init_pose_received_ = false;
  goal_received_ = false;

  //ROS_INFO("Map obtained successfully. Please provide initial pose and goal through RViz.");
}

void RRTPlanner::initPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & msg)
{
  if (init_pose_received_) {
    buildMapImage();
  }

  // Convert mas to Point2D
  poseToPoint(init_pose_, msg->pose.pose);

  // Reject the initial pose if the given point is occupied in the map
  if (!isPointUnoccupied(init_pose_)) {
    init_pose_received_ = false;
    ROS_WARN(
      "The initial pose specified is on or too close to an obstacle please specify another point");
  } else {
    init_pose_received_ = true;
    drawGoalInitPose();
    ROS_INFO("Initial pose obtained successfully.");
  }

  displayMapImage();
}

void RRTPlanner::goalCallback(const geometry_msgs::PoseStamped::ConstPtr & msg)
{
  if (goal_received_) {
    buildMapImage();
  }

  // Convert msg to Point2D
  poseToPoint(goal_, msg->pose);

  // Reject the goal pose if the given point is occupied in the map
  if (!isPointUnoccupied(goal_)) {
    goal_received_ = false;
    ROS_WARN("The goal specified is on or too close to an obstacle please specify another point");
  } else {
    goal_received_ = true;
    drawGoalInitPose();
    ROS_INFO("Goal obtained successfully.");
  }

  displayMapImage();
}

void RRTPlanner::drawGoalInitPose()
{
  if (goal_received_) {
    drawCircle(goal_, 3, cv::Scalar(12, 255, 43));
  }
  if (init_pose_received_) {
    drawCircle(init_pose_, 3, cv::Scalar(255, 200, 0));
  }
}

void RRTPlanner::plan(int expand_dis, int disk_size, bool visualize, bool use_star)
{
  // Reset these values so planning only happens once for a
  // given pair of initial pose and goal points
  goal_received_ = false;
  init_pose_received_ = false;

  // TODO: Fill out this function with the RRT algorithm logic to plan a collision-free
  int iter = 0;
  goal_reached_ = false; // Goal flag
  node_list.push_back(init_pose_);
  parent_list.push_back(0);
  cost_list.push_back(0);
  
  while (!path_to_goal()){
    if (iter%10 == 0){ 
      bias(goal_, expand_dis,disk_size, use_star);
      if (visualize){
        drawCircle(node_list.back(),3,cv::Scalar(0,0,0));
        drawLine(node_list.back(),node_list[parent_list.back()],cv::Scalar(255,0,0),1);
        displayMapImage();
      }
    }
    else{
      expand(expand_dis,disk_size, use_star);
      if (visualize){
        drawCircle(node_list.back(),3,cv::Scalar(0,0,0));
        drawLine(node_list.back(),node_list[parent_list.back()],cv::Scalar(255,0,0),1);
        displayMapImage();
      }
    }
    iter +=1;
  }
  for (int i=1; i<path_coord.size();i++){
    drawLine(path_coord[i],path_coord[i-1],cv::Scalar(0,255,0),1);
    displayMapImage();
  }
  double total_cost = std::accumulate(cost_list.begin(),cost_list.end(),0);
  ROS_INFO("Path generated, Total node number: %ld, Final path segment: %ld, Final path cost: %.2f", node_list.size(), path_coord.size(), total_cost);
  //       path through the map starting from the initial pose and ending at the goal pose
  publishPath();
  plan_reset();
}

/* 
* Node manipulation functions for planning
*/
void RRTPlanner::add_node(int n, Point2D n_){
  node_list.insert(node_list.begin() + n, n_);
}
void RRTPlanner::remove_node(int n){
  node_list.erase(node_list.begin() + n);
}
void RRTPlanner::add_edge(int parent, int child){
  parent_list.insert(parent_list.begin() + child, parent);
  double cost = euc_dist(node_list[parent],node_list[child]) + cost_list[parent]; 
  cost_list.insert(cost_list.begin() + child, cost);
}
void RRTPlanner::remove_edge(int n){
  parent_list.erase(parent_list.begin() + n);
  cost_list.erase(cost_list.begin() + n);
}
void RRTPlanner::step(int nnear, int nrand, int dmax){
  //if distance too far, set intermediate point at farthest reach distance
  double d = euc_dist(node_list[nnear],node_list[nrand]);
  if (d > dmax){
    double u = dmax/d;
    Point2D near = node_list[nnear];
    Point2D rand = node_list[nrand];
    double px = rand.x() - near.x();
    double py = rand.y() - near.y(); 
    double theta = std::atan2(py,px);
    // Calc intermediate node_id 
    Point2D temp((int) (near.x() + dmax * std::cos(theta)), (int) (near.y() + dmax * std::sin(theta)));
    remove_node(nrand);

    // Check goal in reachable distance, if reachable set goal node change goal flag
    if (abs(temp.x() - goal_.x()) < dmax && abs(temp.y() - goal_.y()) < dmax){
      add_node(nrand,goal_);
      goal_state = nrand;
      goal_reached_ = true;
    } 
    else{
      add_node(nrand,temp);
    }
  }
}
bool RRTPlanner::connect(int n1, int n2){
  if (CrossObstacle(node_list[n1],node_list[n2])){
    remove_node(n2);
    return false;
  }
  else {
    add_edge(n1,n2);
    return true;
  }
}
void RRTPlanner::bias(Point2D ngoal, int exp_dis, int disk_size, bool use_star){
  // Add goal and remove goal to node 
  int n = number_node();
  add_node(n,goal_);
  int nnear = GetNearestNode(n);
  step(nnear, n , exp_dis);
  if (connect(nnear,n)){
    if (use_star) {
      star(nnear,n,disk_size);
    }
  }
}
void RRTPlanner::expand(int exp_dis,int disk_size, bool use_star){
  int n = number_node();
  Point2D new_node = sample_rand();
  add_node(n,new_node);
  if (isFree()){
    int nearest = GetNearestNode(n);
    step(nearest,n,exp_dis);
    if (connect(nearest,n)){
      if (use_star) {
        star(nearest, n,disk_size);
      }
    }
  }
}
bool RRTPlanner::path_to_goal(){
  path_id.clear();
  path_coord.clear();
  // Back track parent and add to path
  if (goal_reached_){
      path_coord.push_back(node_list[goal_state]);
      int newpos = parent_list[goal_state];
      while (newpos != 0){
        path_coord.push_back(node_list[newpos]);
        newpos = parent_list[newpos];
      }
      path_coord.push_back(node_list[0]);
  }
  return goal_reached_;
}
void RRTPlanner::plan_reset(){
  goal_state = 0;
  node_list.clear();
  parent_list.clear();
  path_id.clear();
  path_coord.clear();
  goal_reached_ = false;
}
void RRTPlanner::star(int nearest, int n, int disk_size){
  std::vector<int> nearby;
  double radius = disk_size;
  for (int i = 0; i<n; i++){
    if (!CrossObstacle(node_list[i],node_list[n]) && euc_dist(node_list[i],node_list[n]) <= radius){
        nearby.push_back(i);
    }
  }
  int q_min = nearest;
  double C_min = cost_list[n];
  for (int i = 0; i<nearby.size(); i++){
    if (!CrossObstacle(node_list[nearby[i]],node_list[n]) && (cost_list[nearby[i]] + euc_dist(node_list[nearby[i]],node_list[n])) < C_min){
      q_min = nearby[i];
    }
  }
  remove_edge(n);
  add_edge(q_min,n);
  rewire(nearby);
}
void RRTPlanner::rewire(std::vector<int> nearby){
  int last_node = number_node() - 1;
  for (int node_idx : nearby){
    int par = last_node;
    int idx = node_idx;
    while (cost_list[par] + euc_dist(node_list[par],node_list[idx]) - cost_list[idx] <= 1e-6){
      int old_par = parent_list[idx];
      parent_list[idx] = par;
      cost_list[idx] = cost_list[par] + euc_dist(node_list[par],node_list[idx]);
      par = idx, idx = old_par;
    }
  }
}

/* 
* Support functions for planning
*/
int RRTPlanner::number_node(){
  return node_list.size();
}
Point2D RRTPlanner::sample_rand(){
  //Initialize random  device and sample random idx from height and width as Point2D
  std::random_device rd_; 
  std::uniform_int_distribution<int> area_x = std::uniform_int_distribution<int>(0,map_grid_->info.height - 1);
  std::uniform_int_distribution<int> area_y = std::uniform_int_distribution<int>(0,map_grid_->info.width - 1);
  int rx = area_x(rd_);
  int ry = area_y(rd_);
  Point2D pos(rx,ry);
  return pos;
}
double RRTPlanner::euc_dist(Point2D n1, Point2D n2){
  int x1 = n1.x();
  int y1 = n1.y();
  int x2 = n2.x();
  int y2 = n2.y();
  return std::sqrt(std::pow((x1 - x2),2) + std::pow((y1 - y2),2));
}
int RRTPlanner::GetNearestNode(int n){
  double dmin = euc_dist(node_list[0],node_list[n]);
  int nnear = 0;
  for (int i=0; i<n; i++){
    double dist = euc_dist(node_list[i],node_list[n]);
    if (dist < dmin){
      dmin = dist;
      nnear = i;
    }
  }
  return nnear;
}
bool RRTPlanner::isFree(){
  int n = number_node() - 1;
  if (!isPointUnoccupied(node_list[n])){
    remove_node(n);
    return false;
  }
  return true;
}
bool RRTPlanner::CrossObstacle(Point2D src, Point2D tgt){
  std::vector<Point2D> line;
  if (abs(tgt.y() - src.y()) < abs(tgt.x() - src.x())){
    if (src.x() > tgt.x()) {line = loslow(tgt,src);}
    else line = loslow(src,tgt);
  }
  else{
    if (src.y() > tgt.y()) {line = loshigh(tgt,src);}
    else line = loshigh(src,tgt);
  }
  for (Point2D point: line){
    if (isPointUnoccupied(point)){
      continue;
    }
    else return true;
  }
  return false;
}
std::vector<Point2D> RRTPlanner::loslow(Point2D src, Point2D tgt){
  int dx = tgt.x() - src.x();
  int dy = tgt.y() - src.y();
  int yi = 1;
  if (dy <0){
    yi = -1;
    dy = -dy;
  }
  int D = 2*dy - dx;
  int y = src.y();
  std::vector<Point2D> line;
  for (int x=src.x(); x<tgt.x(); x++){
    line.push_back(Point2D(x,y));
    if(D>=0){
      y=y+yi;
      D=D+2*dy-2*dx;
    }
    else{
      D=D+2*dy;
    }
  }
  return line;
}
std::vector<Point2D> RRTPlanner::loshigh(Point2D src, Point2D tgt){
  int dx = tgt.x() - src.x();
  int dy = tgt.y() - src.y();
  int xi = 1;
  if (dx <0){
    xi = -1;
    dx = -dx;
  }
  int D = 2*dx - dy;
  int x = src.x();
  std::vector<Point2D> line;
  for (int y=src.y(); y<tgt.y(); y++){
    line.push_back(Point2D(x,y));
    if(D>=0){
      x=x+xi;
      D=D+2*dx-2*dy;
    }
    else{
      line.push_back(Point2D(x,y));
      D=D+2*dx;
    }
  }
  return line;
}


void RRTPlanner::publishPath()
{
  // Create new Path msg
  nav_msgs::Path path;
  path.header.frame_id = map_grid_->header.frame_id;
  path.header.stamp = ros::Time::now();

  // TODO: Fill nav_msgs::Path msg with the path calculated by RRT
  for (int i = path_coord.size()-1; i >= 0; i--){
    path.poses.push_back(pointToPose(path_coord[i]));
  }
  // Publish the calculated path
  path_pub_.publish(path);

  displayMapImage();
}

bool RRTPlanner::isPointUnoccupied(const Point2D & p)
{
  // TODO: Fill out this function to check if a given point is occupied/free in the map
  if (map_grid_->data[toIndex(p.x(),p.y())]){
    return false;
  }
  return true;
}

void RRTPlanner::buildMapImage()
{
  // Create a new opencv matrix with the same height and width as the received map
  map_ = std::unique_ptr<cv::Mat>(new cv::Mat(map_grid_->info.height,
                                              map_grid_->info.width,
                                              CV_8UC3,
                                              cv::Scalar::all(255)));

  // Fill the opencv matrix pixels with the map points
  for (int i = 0; i < map_grid_->info.height; i++) {
    for (int j = 0; j < map_grid_->info.width; j++) {
      if (map_grid_->data[toIndex(i, j)]) {
        map_->at<cv::Vec3b>(map_grid_->info.height - i - 1, j) = cv::Vec3b(0, 0, 0);
      } else {
        map_->at<cv::Vec3b>(map_grid_->info.height - i - 1, j) = cv::Vec3b(255, 255, 255);
      }
    }
  }
}

void RRTPlanner::displayMapImage(int delay)
{
  cv::imshow("Output", *map_);
  cv::waitKey(delay);
}

void RRTPlanner::drawCircle(Point2D & p, int radius, const cv::Scalar & color)
{
  cv::circle(
    *map_,
    cv::Point(p.y(), map_grid_->info.height - p.x() - 1),
    radius,
    color,
    -1);
}

void RRTPlanner::drawLine(Point2D & p1, Point2D & p2, const cv::Scalar & color, int thickness)
{
  cv::line(
    *map_,
    cv::Point(p2.y(), map_grid_->info.height - p2.x() - 1),
    cv::Point(p1.y(), map_grid_->info.height - p1.x() - 1),
    color,
    thickness);
}

inline geometry_msgs::PoseStamped RRTPlanner::pointToPose(const Point2D & p)
{
  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = p.y() * map_grid_->info.resolution;
  pose.pose.position.y = p.x() * map_grid_->info.resolution;
  pose.header.stamp = ros::Time::now();
  pose.header.frame_id = map_grid_->header.frame_id;
  return pose;
}

inline void RRTPlanner::poseToPoint(Point2D & p, const geometry_msgs::Pose & pose)
{
  p.x(pose.position.y / map_grid_->info.resolution);
  p.y(pose.position.x / map_grid_->info.resolution);
}

inline int RRTPlanner::toIndex(int x, int y)
{
  return x * map_grid_->info.width + y;
}

}  // namespace rrt_planner
