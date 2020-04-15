#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

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

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
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

  double target_v = 22; // in m/s, around 50 mph [speed limit]
  double delta_t = 0.02;
  double cur_v = 1;
  double delta_v = 0.2;
  int lane = 1;
  bool aggresive_mode = false;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,
               &target_v, &cur_v, &delta_v, &delta_t, &lane,
               &aggresive_mode]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
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

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

          size_t path_size = 50;
          size_t prev_path_size = previous_path_x.size();

          if (prev_path_size > 0) {
            car_s = end_path_s;
          }

          // Car's current position will be new origin
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          std::vector<double> ptsx;
          std::vector<double> ptsy;

          if (prev_path_size < 2) {
            double prev_x = car_x - cos(ref_yaw);
            double prev_y = car_y - sin(ref_yaw);

            ptsx.push_back(prev_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_y);
            ptsy.push_back(car_y);
          } else {
            ref_x = previous_path_x[prev_path_size-1];
            ref_y = previous_path_y[prev_path_size-1];

            ptsx.push_back(previous_path_x[prev_path_size-2]);
            ptsx.push_back(ref_x);

            ptsy.push_back(previous_path_y[prev_path_size-2]);
            ptsy.push_back(ref_y);

            ref_yaw = atan2(ptsy[1]-ptsy[0], ptsx[1]-ptsx[0]);
          }

          vector<double> next_wp1 = getXY(car_s+30, lane*4+2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+60, lane*4+2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp3 = getXY(car_s+90, lane*4+2, map_waypoints_s, map_waypoints_x, map_waypoints_y);

          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);
          ptsx.push_back(next_wp3[0]);

          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);
          ptsy.push_back(next_wp3[1]);

          for (size_t i = 0; i < ptsx.size(); ++i) {
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;

            ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
            ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));
          }

          tk::spline s;
          s.set_points(ptsx, ptsy);

          const double target_x = 30.0;
          const double target_y = s(target_x);
          const double target_dist = sqrt(target_x*target_x + target_y*target_y);
          // how many updates we need until we reach (target_x, target_y)
          const double target_delta_ts = target_dist / (cur_v * delta_t);

          for (size_t i = 0; i < prev_path_size; ++i) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          for (size_t i = 1; i <= path_size - prev_path_size; ++i) {
            double x_ref = (i / target_delta_ts) * target_x;
            double y_ref = s(x_ref);

            double x_point = (x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw));
            double y_point = (x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw));

            next_x_vals.push_back(x_point+ref_x);
            next_y_vals.push_back(y_point+ref_y);
          }

          // Initialize distances to large constants
          vector<double> dist_closest_car_in_front{10000, 10000, 10000};
          vector<double> dist_closest_car_behind{10000, 10000, 10000};
          // Initializing velocities to target_v, since it is the speed limit
          // and all cars in the sim obey it.
          vector<double> vel_closest_car_in_front{target_v, target_v, target_v};
          vector<double> vel_closest_car_behind{target_v, target_v, target_v};

          for (size_t i = 0; i < sensor_fusion.size(); ++i) {
            float d = sensor_fusion[i][6];
            if (d >= 0) {
              int cur_lane = floor(d / 4);

              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx*vx+vy*vy);
              double check_car_s = sensor_fusion[i][5];
              double dist_car = fabs(check_car_s - car_s);

              if (check_car_s - car_s > 0) { // in front
                if (dist_closest_car_in_front[cur_lane] > dist_car) {
                  dist_closest_car_in_front[cur_lane] = dist_car;
                  vel_closest_car_in_front[cur_lane] = check_speed;
                }
              } else { // behind
                if (dist_closest_car_behind[cur_lane] > dist_car) {
                  dist_closest_car_behind[cur_lane] = dist_car;
                  vel_closest_car_behind[cur_lane] = check_speed;
                }
              }
            }
          }

          double temp_target_v = target_v;

          if (dist_closest_car_in_front[lane] < 20) {
            double vel_closest = vel_closest_car_in_front[lane];
            temp_target_v = dist_closest_car_in_front[lane] < 5 ?
              vel_closest - 1 : vel_closest + 4;
            temp_target_v = temp_target_v < target_v ? temp_target_v : target_v;
          }

          // We lane change to a lane, for which closest car in front is more distant.
          // If aggresive_mode is on, we always look for lane change opportunity,
          // otherwise we only look for the opportunity if the car in front of us is close.
          if (aggresive_mode || dist_closest_car_in_front[lane] < 20) {
            int next_lane = lane;

            if (lane-1 >= 0) {
              if (dist_closest_car_in_front[lane-1] > dist_closest_car_in_front[next_lane] && dist_closest_car_behind[lane-1] > 20) {
                next_lane = lane-1;
              }
            }

            if (lane+1 <= 2) {
              if (dist_closest_car_in_front[lane+1] > dist_closest_car_in_front[next_lane] && dist_closest_car_behind[lane+1] > 20) {
                next_lane = lane+1;
              }
            }

            lane = next_lane;
          }

          if (cur_v < temp_target_v) {
            cur_v += path_size * delta_t * delta_v;
          } else {
            cur_v -= path_size * delta_t * delta_v;
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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