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

  const double REF_SPEED = 49.0*0.44704; //kept 1mph below 50mph for safety and is converted to m/s
  double current_speed = 0.0; //initial speed of the car
  int lane_no = 1; //initially in the centre lane

  h.onMessage([&REF_SPEED, &current_speed, &lane_no, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
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

	  int prev_path_size = previous_path_x.size();
          
	  double current_ref_x = car_x;
	  double current_ref_y = car_y;
	  double current_ref_yaw_rad = deg2rad(car_yaw);

	  vector<double> anchor_pts_x;
	  vector<double> anchor_pts_y;
          
	  vector<double> next_x_vals;
          vector<double> next_y_vals;

	  bool apply_brakes = false; //in initial default case, brakes are not applied

	  std::cout<<"s = "<<car_s<<"\n";
	  std::cout<<"previous s = "<<end_path_s<<"\n";
	  std::cout<<"x = "<<car_x<<"\n";
	  std::cout<<"yaw = "<<car_yaw<<"\n";
          std::cout<<"No. of points from previous path: "<<prev_path_size<<"\n";
	
	  double current_ref_s = car_s; 
	  if (prev_path_size > 0) {
              current_ref_s = end_path_s; 
	  }

	  if (prev_path_size < 2) {
	      double prev_car_x = car_x - REF_SPEED*0.02*cos(deg2rad(car_yaw));
	      double prev_car_y = car_y - REF_SPEED*0.02*sin(deg2rad(car_yaw));

	      anchor_pts_x.push_back(prev_car_x);
	      anchor_pts_x.push_back(car_x);
              anchor_pts_y.push_back(prev_car_y);
	      anchor_pts_y.push_back(car_y);
	  }

	  else {
	     
              double prev2_car_x = previous_path_x[prev_path_size-2];
	      double prev_car_x = previous_path_x[prev_path_size-1];
              double prev2_car_y = previous_path_y[prev_path_size-2];
	      double prev_car_y = previous_path_y[prev_path_size-1];

	      //update the current reference to the last point from the previous set of points
	      current_ref_x = prev_car_x;
	      current_ref_y = prev_car_y;
              current_ref_yaw_rad = atan2((prev_car_y - prev2_car_y),(prev_car_x - prev2_car_x));

	      anchor_pts_x.push_back(prev2_car_x);
	      anchor_pts_x.push_back(prev_car_x);
              anchor_pts_y.push_back(prev2_car_y);
	      anchor_pts_y.push_back(prev_car_y);
	  }
          
	  //add points from the previous path to the next_vals 
	  for(int i=0;i<prev_path_size;i++) {
              next_x_vals.push_back(previous_path_x[i]);
	      next_y_vals.push_back(previous_path_y[i]); 
	  }

	  //add remaining points from the future path using an interpolated spline
	  //create 3 additional anchor points for the spline by calculating far spaced future positions
	  vector<double> next_anchor_pt_1 = getXY(current_ref_s + 25.0,(lane_no*4)+2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
	  vector<double> next_anchor_pt_2 = getXY(current_ref_s + 50.0,(lane_no*4)+2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
	  vector<double> next_anchor_pt_3 = getXY(current_ref_s + 75.0,(lane_no*4)+2, map_waypoints_s, map_waypoints_x, map_waypoints_y);

	  anchor_pts_x.push_back(next_anchor_pt_1[0]);
	  anchor_pts_x.push_back(next_anchor_pt_2[0]);
	  anchor_pts_x.push_back(next_anchor_pt_3[0]);
	  anchor_pts_y.push_back(next_anchor_pt_1[1]);
	  anchor_pts_y.push_back(next_anchor_pt_2[1]);
	  anchor_pts_y.push_back(next_anchor_pt_3[1]);
          
	  //for(int i=0; i<anchor_pts_x.size(); i++) {
	  //    std::cout<<"Anchor points in mapcoords: \n";
	  //    std::cout<<anchor_pts_x[i]<<" ";
	  //    std::cout<<anchor_pts_y[i]<<"\n";
          //}
          //convert above anchor points to ego vehicle coordinates to enable easier spline/polynomial fitting
	  for(int i=0; i<anchor_pts_x.size(); i++) {
              double x_shifted2veh = anchor_pts_x[i] - current_ref_x;
	      double y_shifted2veh = anchor_pts_y[i] - current_ref_y;
	      
	      anchor_pts_x[i] = x_shifted2veh*cos(0.0-current_ref_yaw_rad) - y_shifted2veh*sin(0.0-current_ref_yaw_rad);  
	      anchor_pts_y[i] = x_shifted2veh*sin(0.0-current_ref_yaw_rad) + y_shifted2veh*cos(0.0-current_ref_yaw_rad);  
	  }

	  // for(int i=0; i<anchor_pts_x.size(); i++) {
	  //    std::cout<<"Anchor points in vehiclecoords: \n";
	  //    std::cout<<anchor_pts_x[i]<<" ";
	  //    std::cout<<anchor_pts_y[i]<<"\n";
          //}

	  // check for sensor fusion data to avoid collisions with other vehicles
	  for(int i=0; i<sensor_fusion.size(); i++) {
	      double other_veh_d = sensor_fusion[i][6];
              if((other_veh_d > lane_no*4) && (other_veh_d < (lane_no+1)*4)) {
		  std::cout<<"Vehicle in lane detected with id: "<<sensor_fusion[i][0]<<"\n";
	          double other_veh_s = sensor_fusion[i][5];
		  double other_veh_vx = sensor_fusion[i][3];
		  double other_veh_vy = sensor_fusion[i][4];
		  double other_veh_res_vel = sqrt(other_veh_vx*other_veh_vx + other_veh_vy*other_veh_vy);

		  double fut_other_veh_s = (other_veh_res_vel*0.02*prev_path_size) + other_veh_s;
		  if (fut_other_veh_s > current_ref_s) {
		      std::cout<<"Vehicle in front detected with id: "<<sensor_fusion[i][0]<<"\n";	  
		      if (fut_other_veh_s - current_ref_s < 50) {
			  std::cout<<"Vehicle within 50m detected in front with id: "<<sensor_fusion[i][0]<<"\n";    
		          apply_brakes = true;
		      }
		  }	  
	      }	      
	  } 

	  if (apply_brakes) {
	      current_speed -= 0.1;
	      std::cout<<"Current speed set to: "<<current_speed<<"\n";
	  }
	  else if (current_speed < REF_SPEED) { 
	      current_speed += 0.1;
	      std::cout<<"Current speed set to: "<<current_speed<<"\n";
	  }
	  
	  // get new path points from a spline
          // the following spline data is in vehicle coordinates
	  tk::spline spline_trajectory;
	  spline_trajectory.set_points(anchor_pts_x, anchor_pts_y);
	  double trajectory_x_end = 25.0;
	  double trajectory_y_end = spline_trajectory(trajectory_x_end);
	  double trajectory_dist = sqrt((trajectory_x_end*trajectory_x_end)+(trajectory_y_end*trajectory_y_end));

	  double no_of_splits = trajectory_dist/(current_speed*0.02);
	  double delta_x_trajectory = trajectory_x_end/no_of_splits;
	  double current_x_veh = 0.0;

	  for(int i=0; i<(50-prev_path_size); i++) {
	      double point_x_veh = current_x_veh + delta_x_trajectory;
	      double point_y_veh = spline_trajectory(point_x_veh);

	      //convert the points in the vehicle coordinates back to map coordinates
              double point_x_map = (point_x_veh*cos(current_ref_yaw_rad) - point_y_veh*sin(current_ref_yaw_rad)) + current_ref_x;
	      double point_y_map = (point_x_veh*sin(current_ref_yaw_rad) + point_y_veh*cos(current_ref_yaw_rad)) + current_ref_y;

	      next_x_vals.push_back(point_x_map);
              next_y_vals.push_back(point_y_map);	      
	      
	      current_x_veh = point_x_veh;
	  }
	   
	  //for(int i=0; i<next_x_vals.size(); i++) {
	  //    std::cout<<next_x_vals[i]<<" ";
          //}
	  //std::cout<<"\n";

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
