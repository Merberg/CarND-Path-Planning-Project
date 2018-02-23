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

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

enum LaneState
{
  LANE_KEEP = 0,
  LANE_PREPARE_CHANGE,
  LANE_CHANGE
};


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
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
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

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
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
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
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

// Check the neighboring car for passing clearance
bool checkForPassingRoom(double car_s, double prepare_vel, double check_car_s, double check_car_speed)
{
  static const double S_SLOW_CAR_AHEAD_m = 40.0;
  static const double S_PASSING_AHEAD_m = 20.0;
  static const double S_PASSING_BEHIND_m = 10;

  double clearance = abs(car_s-check_car_s);

  bool aheadTooClose = (car_s < check_car_s) && (clearance < S_PASSING_AHEAD_m);
  bool behindTooClose = (car_s > check_car_s) && (clearance < S_PASSING_BEHIND_m);
  bool aheadSlower = (car_s < check_car_s) && (clearance < S_SLOW_CAR_AHEAD_m) && (check_car_speed < prepare_vel);
  bool passingRoom = (aheadTooClose || behindTooClose || aheadSlower) ? false : true;
  cout << "\t PassingRoom:" << passingRoom << "\t" << aheadTooClose << ":" << behindTooClose << ":" << aheadSlower << endl;

  return passingRoom;
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

  // Lane information, 0 = left, 1 = middle, 2 = right
  LaneState laneState = LANE_KEEP;
  int lane = 1;
  int lane_desired = 1;


  // Reference velocity to target
  double ref_vel = 0.0;  //mph
  double prepare_vel = 0;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&laneState,&lane,&lane_desired,&ref_vel, &prepare_vel](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
          	double car_lane = 2 + 4*lane;

            // Main car's reference data
            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = deg2rad(car_yaw);

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	int previous_size = previous_path_x.size();

          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];
            double passing_lane = 2 + 4*lane_desired;
          	bool passing_allowed = true;
          	bool pass_on_right = (lane == 1) ? true : false;
          	bool collision_warning = false;

          	// Create evenly spaced anchor waypoints
            vector<double> anchor_x;
            vector<double> anchor_y;
            static const double S_SPACING_m = 30.0;

            if (previous_size > 0)
            {
              car_s = end_path_s;
            }
            cout << "Lane:" << lane << "\tState:" << laneState << "\tSpeed:" << car_speed << "\tS:" << car_s << endl;

            // Act base on other cars
            for (int i=0; i< sensor_fusion.size(); i++)
            {
              double id = sensor_fusion[i][0];
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx*vx+vy*vy);
              double check_car_s = sensor_fusion[i][5];
              check_car_s += ((double)previous_size*0.02*check_speed);
              float d = sensor_fusion[i][6];

              // Car in lane
              if ((d < car_lane+2) && (d > car_lane-2))
              {
                //Slow car that needs to be passed
                if ((check_car_s > car_s) && ((check_car_s-car_s) < S_SPACING_m))
                {
                  collision_warning = true;
                  prepare_vel = check_speed;

                  if (ref_vel > prepare_vel)
                    ref_vel -= 0.224;
                }
              }
              // Trying to change lanes
              else if (laneState == LANE_PREPARE_CHANGE)
              {
                // Record no gap if desired lane is occupied
                if ((d < passing_lane+2) && (d > passing_lane-2))
                {
                  passing_allowed &= checkForPassingRoom(car_s, prepare_vel, check_car_s, check_speed);
                }
                // Otherwise look for a right opening from center
                else if ((lane == 1) && (d > lane+2))
                {
                  pass_on_right &= checkForPassingRoom(car_s, prepare_vel, check_car_s, check_speed);
                }
              }
            }


            if (previous_size < 2)
            {
              // Use two points that make the path tangent to the car using
              // the cars current position
              double prev_car_x = car_x - cos(car_yaw);
              double prev_car_y = car_y - sin(car_yaw);
              anchor_x.push_back(prev_car_x);
              anchor_x.push_back(car_x);
              anchor_y.push_back(prev_car_y);
              anchor_y.push_back(car_y);
            }
            else
            {
              // Or use two points from the previous position
              ref_x = previous_path_x[previous_size-1];
              ref_y = previous_path_y[previous_size-1];
              double prev_ref_x = previous_path_x[previous_size-2];
              double prev_ref_y = previous_path_y[previous_size-2];
              ref_yaw = atan2(ref_y-prev_ref_y, ref_x-prev_ref_x);
              anchor_x.push_back(prev_ref_x);
              anchor_x.push_back(ref_x);
              anchor_y.push_back(prev_ref_y);
              anchor_y.push_back(ref_y);
            }

            // Deal with lane changes
            double next_d = car_lane;
            switch (laneState)
            {
              case LANE_PREPARE_CHANGE:
                if (passing_allowed)
                {
                  laneState = LANE_CHANGE;
                }
                else if (pass_on_right)
                {
                  lane_desired = lane + 1;
                  laneState = LANE_CHANGE;
                }
                break;

              case LANE_CHANGE:
                lane = lane_desired;
                next_d = 2 + 4*lane;
                laneState = LANE_KEEP;
                break;

              case LANE_KEEP:
              default:
                if (collision_warning)
                {
                  laneState = LANE_PREPARE_CHANGE;
                  lane_desired = (lane == 0) ? (lane + 1) : (lane - 1);
                  passing_lane = 2 + 4*lane_desired;
                }
                break;
            }

            // Add evenly spaced (in Frenet) points ahead
            for (double i=S_SPACING_m; i<=3*S_SPACING_m; i+=S_SPACING_m)
            {
              vector<double> next_wp = getXY((car_s+i), next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
              anchor_x.push_back(next_wp[0]);
              anchor_y.push_back(next_wp[1]);
            }

            // Transform to the vehicle's perspective
            for (int i=0; i <anchor_x.size(); i++)
            {
              double shift_x = anchor_x[i]-ref_x;
              double shift_y = anchor_y[i]-ref_y;

              anchor_x[i] = shift_x*cos(0-ref_yaw) - shift_y*sin(0-ref_yaw);
              anchor_y[i] = shift_x*sin(0-ref_yaw) + shift_y*cos(0-ref_yaw);
            }

            // Create the spline with the anchor points
            tk::spline s;
            s.set_points(anchor_x, anchor_y);

            vector<double> next_x_vals;
            vector<double> next_y_vals;

            // Add the previous points to the path to help with the transition
            for (int i=0; i<previous_size; i++)
            {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }

            // Breakup the spline points to travel at the reference velocity
            double horizon_x = S_SPACING_m;
            double horizon_y = s(horizon_x);
            double target_dist = sqrt((horizon_x*horizon_x)+(horizon_y*horizon_y));
            double prev_x = 0;

            // Fill up the path while adjusting velocity
            for (int i=0; i < 50-previous_size; i++)
            {
              if (!collision_warning && (ref_vel < 49.5))
              {
                ref_vel += 0.224;
              }

              double N = target_dist/(0.02*ref_vel/2.24); //convert to m
              double x = prev_x + horizon_x/N;
              prev_x = x;
              double y = s(x);

              //translate back to world
              double x_world = ref_x + x*cos(ref_yaw) - y*sin(ref_yaw);
              double y_world = ref_y + x*sin(ref_yaw) + y*cos(ref_yaw);

              next_x_vals.push_back(x_world);
              next_y_vals.push_back(y_world);
            }

            json msgJson;
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
