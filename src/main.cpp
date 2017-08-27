#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

#define MINIMUM_PATH_SIZE 2
#define POINTS_SPACING 30 // meters
#define HORIZON_VALUE 30 // meters
#define SIMULATOR_UPDATE_RATE 0.02 // seconds
#define NUM_PATH_PLANNER_PTS 50


using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

  // Define reference velocity in mph
  double ref_vel = 49.5;

  // Define initial lane
  int lane = 1;
  
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&lane,&ref_vel](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;

		// Get previous path size to help with the transition
		int prev_path_size = previous_path_x.size();

		/*
		// if prev_path_size has more than one point, change the car location to it
		if(prev_path_size > 0){
		  car_s = end_path_s;
		}

		// create a flag for car in front of us
		bool is_car_infront = false;

		// go through the sensor fusion list to check for cars
		for(int i=0; i < sensor_fusion.size(); i++){
		  // get d value of other car
		  float d = sensor_fusion[i][6];

		  // check for cars in own lane
		  if( d > (4*lane) && d < (4+4*lane)){
		    // get that car's parameters
		    double vx = sensor_fusion[i][3];
		    
		  }

		}
		*/

		// Define vector of waypoints that are POINTS_SPACING meters apart to be used by spline
		vector<double> waypoints_x;
		vector<double> waypoints_y;

		// Define reference values of the car's location
		double ref_x = car_x;
		double ref_y = car_y;
		double ref_yaw = deg2rad(car_yaw);

		// check if previous path size has less than minimum number of points needed
		if( prev_path_size < MINIMUM_PATH_SIZE ){
		  // create a path tangent to the car using two points
		  double prev_x = car_x - cos(car_yaw);
		  double prev_y = car_y - sin(car_yaw);

		  // push these points and the reference values on the waypoints vector
		  waypoints_x.push_back(prev_x);
		  waypoints_y.push_back(prev_y);

		  waypoints_x.push_back(car_x);
		  waypoints_y.push_back(car_y);
		}
		// use previous path's end point as starting point for continunity
		else{
		  // assign reference value as previous end point
		  ref_x = previous_path_x[prev_path_size-1];
		  ref_y = previous_path_y[prev_path_size-1];

		  double ref_x_prev = previous_path_x[prev_path_size-2];
		  double ref_y_prev = previous_path_y[prev_path_size-2];

		  // calculate ref_yaw by using the slope
		  ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

		  // create a path tangent to prev_path end point using two points
		  waypoints_x.push_back(ref_x_prev);
		  waypoints_y.push_back(ref_y_prev);
		  
		  waypoints_x.push_back(ref_x);
		  waypoints_y.push_back(ref_y);
		}

		// add three more points that are POINTS_SPACING meters spaced using frenet
		vector<double> wp1 = getXY(car_s+POINTS_SPACING, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
		vector<double> wp2 = getXY(car_s+2*POINTS_SPACING, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
		vector<double> wp3 = getXY(car_s+3*POINTS_SPACING, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

		waypoints_x.push_back(wp1[0]);
		waypoints_x.push_back(wp2[0]);
		waypoints_x.push_back(wp3[0]);

		waypoints_y.push_back(wp1[1]);
		waypoints_y.push_back(wp2[1]);
		waypoints_y.push_back(wp3[1]);

		// transform waypoints to local car's coordinates
		for(int i = 0; i < waypoints_x.size(); i++){
		  // set car's reference angle to 0 degrees
		  double shift_x = waypoints_x[i] - ref_x;
		  double shift_y = waypoints_y[i] - ref_y;

		  waypoints_x[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
		  waypoints_y[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));
		}

		// create a new spline
		tk::spline sp;

		// feed the spline with the waypoints
		sp.set_points(waypoints_x, waypoints_y);
		
		// Define the actual path planer (x,y) points
		vector<double> next_x_vals;
		vector<double> next_y_vals;

		// add points from previous path to path planner to help with transition
		for(int i = 0; i < prev_path_size; i++)
		  {
		    next_x_vals.push_back(previous_path_x[i]);
		    next_y_vals.push_back(previous_path_y[i]);
		  }

		// calculate spline number of points to travel at reference speed
		double goal_x = HORIZON_VALUE;
		double goal_y = sp(goal_x);
		double goal_distance = sqrt(goal_x*goal_x + goal_y*goal_y);

		// define start point
		double x_add_on = 0;

		// fill up the rest of the path planner
		// use the concept provided by Aaron Brown in walkthrough
		for(int i = 1; i <= NUM_PATH_PLANNER_PTS - prev_path_size; i++){

		  // divide velocity by 2.24 to convert to m/s
		  double N = goal_distance / (SIMULATOR_UPDATE_RATE*ref_vel/2.24);
		  double xp = x_add_on + goal_x/N;
		  double yp = sp(xp);

		  // set new reference
		  x_add_on = xp;

		  double x_ref = xp;
		  double y_ref = yp;

		  // rotate back to global coordinates
		  xp = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
		  yp = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

		  xp += ref_x;
		  yp += ref_y;

		  next_x_vals.push_back(xp);
		  next_y_vals.push_back(yp);
		}
		
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
















































































