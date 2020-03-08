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
  bool change_lane = false; //in initial case

  h.onMessage([&REF_SPEED, &current_speed, &lane_no, &change_lane, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
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
	  bool lane_change_over = true; //flag to indicate that previous lane change is complete
          int to_lane; //destination lane for lane change
          
	  // set reference values of s,d for ego vehicle to the current frame start values given by the simulator
	  double current_ref_s = car_s;
	  double current_ref_d = car_d; 
	  
	  // if previous path has unused points, use the last point to set reference values of s,d for ego vehicle 
	  if (prev_path_size > 0) {
              current_ref_s = end_path_s;
	      current_ref_d = end_path_d; 
	  }
          
	  // TRAJECTORY GENERATION USING SPLINE - PART I - using past data

	  // Use points from previous path as the first 2 anchor points to ensure smooth transition 
	  // if there are less than 2 unused points from previous path, reverse-estimate those 2 points
	  // from current data
	  if (prev_path_size < 2) {
	      double prev_car_x = car_x - REF_SPEED*0.02*cos(deg2rad(car_yaw));
	      double prev_car_y = car_y - REF_SPEED*0.02*sin(deg2rad(car_yaw));

	      anchor_pts_x.push_back(prev_car_x);
	      anchor_pts_x.push_back(car_x);
              anchor_pts_y.push_back(prev_car_y);
	      anchor_pts_y.push_back(car_y);
	  }
          // if there are 2 or more such unused points, use the last 2 of them as the first 2 anchor points
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
          
	  // CARRY OVER UNUSED POINTS FROM PREVIOUS PATH

	  //add unused points from the previous path to the next_vals 
	  for(int i=0;i<prev_path_size;i++) {
              next_x_vals.push_back(previous_path_x[i]);
	      next_y_vals.push_back(previous_path_y[i]); 
	  }

	  // TRAJECTORY GENERATION USING SPLINE - PART II - using predictions into future
	  // The spline will be generated based on the required behaviour in the future
	  // This behaviour will depend on the current state of the car, lane traffic speed, collision risks,etc.
	  // The following section considers all this

          //LANE CHANGE RELEVANT CHECKS
	  //before calculating the remaining points for the path, the lane must be selected
	  
	  //check if the previous lane change is over by checking if the last predicted point or the reference value
	  //is close to the destined lane centre
	  if (fabs((4*lane_no)+2 - current_ref_d) >= 1) {
	  lane_change_over = false;
	  }

	  // check for sensor fusion data to avoid collisions with other vehicles
	  for(int i=0; i<sensor_fusion.size(); i++) {
	      if (frontal_collision_risk(sensor_fusion[i], current_ref_s, current_speed, lane_no, prev_path_size)) { 
	          apply_brakes = true;
                  //change lanes only at speeds lower than 40mph to avoid excessive accelerations due to cornering
		  //change lanes only at speeds higher than 25mph to make quicker turns and avoid confusions
		  //change lanes only when previous lane change is complete
		  if((current_speed < 45.0*0.44704) && (current_speed > 25.0*0.44704) && (lane_change_over)) {  
	              change_lane = true;
		      apply_brakes = false; //to avoid combined cornering and braking
		  }
		  break;
	      }
          }		  
          
	  // if lane is required to be changed then for each choice of destination lane, investigate feasibility
	  if (change_lane) {
	      //if starting position is lane 1
	      if (lane_no == 1) {
                  //the ego vehicle will first try left lane 0
		  to_lane = 0;
	          int lane_0_status = lane_status(sensor_fusion, current_ref_s, current_speed, to_lane, prev_path_size);
		  if (lane_0_status == 2) {
		      lane_no = to_lane;
		      change_lane = false; // once lane is changed, set flag back to false
		  }
		  // if lane 0 is not safe or too slow to enter, try right lane 2
		  else {
		      to_lane = 2;
		      int lane_2_status = lane_status(sensor_fusion, current_ref_s, current_speed, to_lane, prev_path_size);
		      if (lane_2_status == 2) {
		          lane_no = to_lane;
			  change_lane = false; // once lane is changed, set flag back to false
		      }
		      // if lane 2 is also not safe or too slow to enter, abort lane change for now and apply brakes
		      else {
		          change_lane = false;
		          apply_brakes = true;
		      }	    
		 }
	      }
	      //if starting position is lane 0
	      else if (lane_no==0) {
                  //the ego vehicle will first try left lane 1
		  to_lane = 1;
	          int lane_1_status = lane_status(sensor_fusion, current_ref_s, current_speed, to_lane, prev_path_size);
		  if (lane_1_status == 2) {
		      lane_no = to_lane;
		      change_lane = false; // once lane is changed, set flag back to false
		  }
		  // if lane 1 can't be entered into since it is too slow to enter, check if it is safe to enter 
		  else if (lane_1_status == 1){
		      // if lane 1 is safe to enter, check if lane 2 can be entered using lane 1 temporarily 
		      to_lane = 2;
		      int lane_2_status = lane_status(sensor_fusion, current_ref_s, current_speed, to_lane, prev_path_size);
		      if (lane_2_status == 2) {
		          lane_no = to_lane - 1;  //to avoid direct jump 
			  change_lane = false; // once lane is changed, set flag back to false
		      }
		      // if lane 2 is also not safe or too slow to enter, abort lane change for now and apply brakes
		      else {
			  change_lane = false;
			  apply_brakes = true;
		      }
		  }
		  // if lane 1 is unsafe to enter, abort lane change for now and apply brakes 
		  else if (lane_1_status == 0){
		          change_lane = false;
		          apply_brakes = true;
		  }	    
	      }
	      //if starting position is lane 2
	      else if (lane_no == 2) {
                  //the ego vehicle will first try left lane 1
		  to_lane = 1;
	          int lane_1_status = lane_status(sensor_fusion, current_ref_s, current_speed, to_lane, prev_path_size);
		  if (lane_1_status == 2) {
		      lane_no = to_lane;
		      change_lane = false; // once lane is changed, set flag back to false
		  }
		  // if lane 1 can't be entered into since it is too slow to enter, check if it is safe to enter 
		  else if (lane_1_status == 1){
		      // if lane 1 is safe to enter, check if lane 0 can be entered using lane 1 temporarily 
		      to_lane = 0;
		      int lane_0_status = lane_status(sensor_fusion, current_ref_s, current_speed, to_lane, prev_path_size);
		      if (lane_0_status == 2) {
		          lane_no = to_lane + 1; //to avoid direct jump required
			  change_lane = false; // once lane is changed, set flag back to false
		      }
		      // if lane 0 is also not safe or too slow to enter, abort lane change for now and apply brakes
		      else {
			  change_lane = false;
			  apply_brakes = true;
		      }
		  }
		  // if lane 1 is unsafe to enter, abort lane change for now and apply brakes 
		  else if (lane_1_status == 0){
		          change_lane = false;
		          apply_brakes = true;
		  }	    
	      }   
	
  	  }

          // SPEED CHANGE CHECKS AND CONTROLS

	  // reduce the vehicle speed if the brakes are applied, but only when the previous lane change is complete
	  // braking should not happen during lane change 
	  // This to avoid higher acceleration values due to combined braking and cornering during lane change
	  if ((apply_brakes) && (lane_change_over)) {
	      current_speed -= 0.4;
	  }

	  // increase speed if current speed is lower than the set speed limit and only if no lane change is planned
	  // this is because increasing speed during lane_change will increase total acceleration
	  else if ((current_speed < REF_SPEED) && (lane_change_over)) { 
	      current_speed += 0.25;
	  }
	  
	  // TRAJECTORY GENERATION USING SPLINE - PART II contd.
          
	  //the first 2 points were taken from the previous path data or estimated by reverse predicting in time from current state
          //create 3 additional anchor points for the spline to calculate future path
          //here 3 far space points at 25m, 50m and 75m are used as the last 3 anchor points
          vector<double> next_anchor_pt_1 = getXY(current_ref_s + 25.0,(lane_no*4)+2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_anchor_pt_2 = getXY(current_ref_s + 50.0,(lane_no*4)+2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_anchor_pt_3 = getXY(current_ref_s + 75.0,(lane_no*4)+2, map_waypoints_s, map_waypoints_x, map_waypoints_y);

          anchor_pts_x.push_back(next_anchor_pt_1[0]);
          anchor_pts_x.push_back(next_anchor_pt_2[0]);
          anchor_pts_x.push_back(next_anchor_pt_3[0]);
          anchor_pts_y.push_back(next_anchor_pt_1[1]);
          anchor_pts_y.push_back(next_anchor_pt_2[1]);
          anchor_pts_y.push_back(next_anchor_pt_3[1]);

          //convert above anchor points to ego vehicle coordinates to enable easier spline/polynomial fitting
          //this will prevent multiple y values for the same x value and also avoid numerical issues
          for(int i=0; i<anchor_pts_x.size(); i++) {
              double x_shifted2veh = anchor_pts_x[i] - current_ref_x;
              double y_shifted2veh = anchor_pts_y[i] - current_ref_y;

              anchor_pts_x[i] = x_shifted2veh*cos(0.0-current_ref_yaw_rad) - y_shifted2veh*sin(0.0-current_ref_yaw_rad);
              anchor_pts_y[i] = x_shifted2veh*sin(0.0-current_ref_yaw_rad) + y_shifted2veh*cos(0.0-current_ref_yaw_rad);
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
	  double point_x_veh = 0.0;
	  double point_y_veh = 0.0;
	  double point_x_map = 0.0;
	  double point_y_map = 0.0;

	  for(int i=0; i<(50-prev_path_size); i++) {
	      point_x_veh = current_x_veh + delta_x_trajectory;
	      point_y_veh = spline_trajectory(point_x_veh);

	      //convert the points in the vehicle coordinates back to map coordinates
              point_x_map = (point_x_veh*cos(current_ref_yaw_rad) - point_y_veh*sin(current_ref_yaw_rad)) + current_ref_x;
	      point_y_map = (point_x_veh*sin(current_ref_yaw_rad) + point_y_veh*cos(current_ref_yaw_rad)) + current_ref_y;

	      next_x_vals.push_back(point_x_map);
              next_y_vals.push_back(point_y_map);	      
	      
	      current_x_veh = point_x_veh;
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
