#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"
#include "json.hpp"
#include "spline.h"
#include "vehicle.h"
#include "PID.h"
//#include "data_structs.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// Some flags
const bool DEBUG = false;
const bool LANE_CHANGE_BUFFER = true;
const bool ENABLE_LANE_CHANGE = true;

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

// TODO - complete this function
vector<double> JMT(vector< double> start, vector <double> end, double T)
{
  /*
  Calculate the Jerk Minimizing Trajectory that connects the initial state
  to the final state in time T.

  INPUTS

  start - the vehicles start location given as a length three array
      corresponding to initial values of [s, s_dot, s_double_dot]

  end   - the desired end state for vehicle. Like "start" this is a
      length three array.

  T     - The duration, in seconds, over which this maneuver should occur.

  OUTPUT
  an array of length 6, each value corresponding to a coefficent in the polynomial
  s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5

  EXAMPLE

  > JMT( [0, 10, 0], [10, 10, 0], 1)
  [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
  */

  using Eigen::MatrixXd;
  using Eigen::VectorXd;

  MatrixXd A = MatrixXd(3, 3);
  A << T*T*T, T*T*T*T, T*T*T*T*T,
          3*T*T, 4*T*T*T,5*T*T*T*T,
          6*T, 12*T*T, 20*T*T*T;

  MatrixXd B = MatrixXd(3,1);
  B << end[0]-(start[0]+start[1]*T+.5*start[2]*T*T),
          end[1]-(start[1]+start[2]*T),
          end[2]-start[2];

  MatrixXd Ai = A.inverse();

  MatrixXd C = Ai*B;

  vector <double> result = {start[0], start[1], .5*start[2]};
  for(int i = 0; i < C.size(); i++)
  {
    result.push_back(C.data()[i]);
  }

  return result;

}

int main() {
  uWS::Hub h;
	chrono::high_resolution_clock::time_point start_time = chrono::high_resolution_clock::now();

    /************************** HIGHWAY MAP ************************
        x-g-cord y-g-cord       s-cord  dx-cord         dy-cord
        784.6001    1135.571    0       -0.02359831     -0.9997216
        815.2679    1134.93     30.6    -0.01099479     -0.9999396
     ***************************************************************/

    /************************* SENSOR FUSION *************************
     *  [ id, x, y, vx, vy, s, d]
     *  [0,909.0637,1124.814,21.88802,0.109534,124.3841,10.0168]
        [1,876.1334,1128.834,20.54134,-0.09512661,91.54198,5.973429]
        [2,1075.357,1172.027,16.15231,6.794888,295.8416,6.031289]
        [3,823.8144,1124.725,18.63423,-0.5323493,39.22773,10.19955]
        [4,900.7066,1132.781,20.00806,0.02420581,116.114,2.019067]
        [5,1068.804,1164.974,14.66237,6.106757,287.0748,10.00994]
        [6,1068.592,1173.504,15.83609,6.608347,290.1703,2.058725]
        [7,871.9068,1124.844,20.86737,-0.06608441,87.34735,9.973951]
        [8,1029.135,1153.198,17.97057,8.828606,245.9818,5.82229]
        [9,878.798,1132.806,8.453919,-0.02093844,94.20541,2.000587]
        [10,924.6902,1132.917,21.02633,0.2166646,140.08,2.049147]
        [11,831.3856,1123.989,20.27186,-0.1007643,46.79944,10.93051]
     *****************************************************************/



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
  double ref_vel = 0;

  int target_lane = 1;

  vector<int> lane_buffer;
  if (LANE_CHANGE_BUFFER){
    for (int i = 0; i < LANE_BUFFER_SIZE; i++){
      lane_buffer.push_back(1);
    }
  }


  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
    double s;
    double d_x;
    double d_y;
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

  h.onMessage([&ref_vel, &target_lane, &lane_buffer, &map_waypoints_x, &map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy, &start_time](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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

            const int N_POINTS = 50;
            const int LANE_WIDTH = 4;


            //PID pid;
            // TODO: Initialize the pid variable.
            //pid.Init(0.1, 0.0001, 4.0);

            int prev_size = previous_path_x.size();

            Vehicle ego = {-1, car_x, car_x, car_speed*cos(rad2deg(car_yaw)), \
              car_speed*sin(rad2deg(car_yaw)), car_s, car_d};

            ego.current_lane = int(car_d/4);

            vector<Vehicle> traffic;

            traffic.push_back(ego);

            bool too_close = false;
            if(prev_size > 0){
              car_s = end_path_s;
            }

            // find ref_v to use
            vector<prediction> pred;
            double closest_car = 9999;

            for(int i = 0; i < sensor_fusion.size();i++) {
              // car is in my lane
              int sf_id = sensor_fusion[i][0];
              double sf_x = sensor_fusion[i][1];
              double sf_y = sensor_fusion[i][2];
              double sf_vx = sensor_fusion[i][3];
              double sf_vy = sensor_fusion[i][4];
              double sf_s = sensor_fusion[i][5];
              double sf_d = sensor_fusion[i][6];

              Vehicle car = {sf_id, sf_x, sf_y, sf_vx, sf_vy, sf_s, sf_d};
              traffic.push_back(car);

              // if car at same lane
              if (int(sf_d / 4) == int(car_d / 4)) {

                double check_speed = sqrt(sf_vx * sf_vx + sf_vy * sf_vy);
                sf_s += ((double) prev_size * .02 * check_speed);

                double sf_car_dist = sf_s - car_s;

                if (sf_car_dist > 0 && sf_car_dist < closest_car && sf_car_dist <= CONTROL_SPEED_THOLD) {
                  closest_car = sf_car_dist;
                }
              }
            }

            if(closest_car < 9999){
              //double cte = closest_car - KEEP_DISTANCE;
              //pid.UpdateError(cte);
              double delta_v = ego.speed_controller(closest_car, car_speed);
              cout << "distance closest_car: " << closest_car << " delta_v: " << delta_v << endl;
              if (ref_vel + delta_v < SPEED_LIMIT){
                ref_vel += delta_v;
              }
            }


            if(closest_car == 9999 && ref_vel < SPEED_LIMIT){
              if (ref_vel < 10){
                  ref_vel+=0.8;
              }else if(10 <= ref_vel < 20){
                ref_vel+=0.6;
              }else if(20 <= ref_vel < 30){
                ref_vel+=0.5;
              }else{
                  ref_vel+=0.4;
              }
            }

            /* **********************************
                      PREDICT POSITION
               *********************************/
            for(auto & t: traffic) {
              prediction p;
              p.id = t.id;
              p.predictions = t.Predict();
              pred.push_back(p);
            }

              /* **********************************
                      NEW STATE DEFINITION
                **********************************/

              vector<int> next_lane = ego.get_next_lane(pred);
              const bool keep_dist = next_lane[1];

              if (LANE_CHANGE_BUFFER && ENABLE_LANE_CHANGE){
                if (next_lane[0] >= 0 && next_lane[0] <= 2){
                  cout << "just inserted :" << next_lane[0] << endl;
                  lane_buffer.push_back(next_lane[0]);
                }
                // Buffer Lane to make sure of lane change
                if(lane_buffer.size() > LANE_BUFFER_SIZE){
                  // remove oldest item
                  lane_buffer.erase(lane_buffer.begin());
                }

                double lane_prod = 1;
                int lane_sum = 0;

                for(int i = 0; i<lane_buffer.size(); i++){
                  cout << lane_buffer[i] << " ";
                  lane_prod*=lane_buffer[i];
                  lane_sum+=lane_buffer[i];
                }

                cout << endl;

                if(lane_prod == 0 && lane_sum == 0){
                  target_lane = 0;
                }else if(lane_prod == 1 && lane_sum == LANE_BUFFER_SIZE){
                  target_lane = 1;
                }else if(lane_prod == pow(2,LANE_BUFFER_SIZE) && lane_sum == LANE_BUFFER_SIZE*2){
                  target_lane = 2;
                }

                cout << "buffer_lane - sum: " << lane_sum << " prod: " << lane_prod << endl;
              }else if(ENABLE_LANE_CHANGE){
                target_lane = next_lane[0];
                cout << "my lane: " << int(car_d / 4) << endl;
              }



          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
            // This vector will receive the points which will be used by spline function
            // to interpolate between them and create a smoother path.
            vector<double> ptsx;
            vector<double> ptsy;

            // Reference states
            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = deg2rad(car_yaw);

            // if previous values is to short, define a small segment tangent to car's path
            if (prev_size < 2){
              double prev_car_x = car_x - cos(car_yaw);
              double prev_car_y = car_y - sin(car_yaw);

              ptsx.push_back(prev_car_x);
              ptsy.push_back(prev_car_y);

              ptsx.push_back(car_x);
              ptsy.push_back(car_y);

            }
            else
            {
              // if path is long enough, also add tangent points

              ref_x = previous_path_x[prev_size - 1];
              ref_y = previous_path_y[prev_size - 1];

              double ref_prev_x = previous_path_x[prev_size - 2];
              double ref_prev_y = previous_path_y[prev_size - 2];


              ptsx.push_back(ref_prev_x);
              ptsx.push_back(ref_x);

              ptsy.push_back(ref_prev_y);
              ptsy.push_back(ref_y);

              ref_yaw = atan2(ref_y - ref_prev_y,ref_x - ref_prev_x);
            }

             // Now add three more points 30m between each other

             vector<double> next_wp0 =  getXY(car_s+30, (target_lane*4 + 2), map_waypoints_s, map_waypoints_x, map_waypoints_y);
             vector<double> next_wp1 =  getXY(car_s+60, (target_lane*4 + 2), map_waypoints_s, map_waypoints_x, map_waypoints_y);
             vector<double> next_wp2 =  getXY(car_s+90, (target_lane*4 + 2), map_waypoints_s, map_waypoints_x, map_waypoints_y);

             ptsx.push_back(next_wp0[0]);
             ptsx.push_back(next_wp1[0]);
             ptsx.push_back(next_wp2[0]);

             ptsy.push_back(next_wp0[1]);
             ptsy.push_back(next_wp1[1]);
             ptsy.push_back(next_wp2[1]);

            for (int i = 0; i < ptsx.size(); i++){
              // shift car angle reference to 0 degress
              double shift_x = ptsx[i]-ref_x;
              double shift_y = ptsy[i]-ref_y;

              ptsx[i] = shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw);
              ptsy[i] = shift_x*sin(0-ref_yaw)+shift_y*cos(0-ref_yaw);
            }

            // create a spline object

            tk::spline spline_s;

            spline_s.set_points(ptsx, ptsy);

            vector<double> next_x_vals;
            vector<double> next_y_vals;


          // start with all previous points, from last time
            for(int i = 0; i < previous_path_x.size(); i++){
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }

            // Calculate how to break up spline points so that we travel at desired velocity
            double target_x = 30.0;
            double target_y = spline_s(target_x);

            double target_dist = sqrt(target_x*target_x + target_y*target_y);
            double x_add_on = 0;

            // fill up the rest of path planner points
            for (int i = 0; i< N_POINTS - previous_path_x.size(); i++){

              double N = (target_dist/(0.02*ref_vel/2.24));
              double x_point = x_add_on + (target_x)/N;
              double y_point = spline_s(x_point);

              x_add_on = x_point;

              double x_ref = x_point;
              double y_ref = y_point;

              // rotate back to global

              x_point = (x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw));
              y_point = (x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw));

              x_point += ref_x;
              y_point += ref_y;

              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);

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
