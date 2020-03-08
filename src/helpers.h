#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>

// for convenience
using std::string;
using std::vector;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
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

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, 
                    const vector<double> &maps_y) {
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); ++i) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, 
                 const vector<double> &maps_y) {
  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = std::min(2*pi() - angle, angle);

  if (angle > pi()/2) {
    ++closestWaypoint;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, 
                         const vector<double> &maps_x, 
                         const vector<double> &maps_y) {
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if (next_wp == 0) {
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

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; ++i) {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, 
                     const vector<double> &maps_x, 
                     const vector<double> &maps_y) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1))) {
    ++prev_wp;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),
                         (maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};
}

// Check if there is a risk of collision with the vehicle in front
bool frontal_collision_risk(vector<double> other_vehicle, double current_ref_s, double current_speed, int lane_no, int prev_path_size) {
  double other_veh_d = other_vehicle[6];
  // check if the other vehicle's d indicates that it is in the same lane_no as the ego vehicle 
  if((other_veh_d > lane_no*4) && (other_veh_d < (lane_no+1)*4)) {
      double other_veh_s = other_vehicle[5];
      double other_veh_vx = other_vehicle[3];
      double other_veh_vy = other_vehicle[4];
      double other_veh_res_vel = sqrt(other_veh_vx*other_veh_vx + other_veh_vy*other_veh_vy);

      double fut_other_veh_s = (other_veh_res_vel*0.02*prev_path_size) + other_veh_s;
      // check if the other vehicle is ahead of the ego vehicle
      if (fut_other_veh_s > current_ref_s) {
	  // check if the other vehicle is within 25m distance
          if ((fut_other_veh_s - current_ref_s < 25)) { 
	      return true;
          }
      }
  }
  return false;
}

int lane_status(vector<vector<double>> sensor_fusion, double current_ref_s, double current_speed, int to_lane, int prev_path_size) {
  // returns 0 if lane is unsafe to enter
  // returns 1 if lane is safe to enter, but slow
  // returns 2 if lane is both safe to enter and also faster
  bool is_lane_slow = false;
  for(int i=0; i<sensor_fusion.size(); i++) {
      double other_veh_d = sensor_fusion[i][6];
      // check if the other vehicle's d indicates that it is in the same lane_no as the destination lane
      if((other_veh_d > to_lane*4) && (other_veh_d < (to_lane+1)*4)) {
	  int other_vehicle_id = sensor_fusion[i][0];
          double other_veh_s = sensor_fusion[i][5];
          double other_veh_vx = sensor_fusion[i][3];
          double other_veh_vy = sensor_fusion[i][4];
          double other_veh_res_vel = sqrt(other_veh_vx*other_veh_vx + other_veh_vy*other_veh_vy);

	  double fut_other_veh_s = (other_veh_res_vel*0.02*prev_path_size) + other_veh_s;
	  double delta_s_wrt_ego = fut_other_veh_s - current_ref_s;  
	  double delta_v_wrt_ego = other_veh_res_vel - current_speed;
          // check if there is a vehicle within +/- 10m which makes the lane unsafe to enter
	  // 0 can be returned immediately, since the lane is unsafe even if one vehicle blocks it
	  if (fabs(delta_s_wrt_ego) <= 10.0) {
	      return 0;
	  }
	  // if there is a vehicle ahead by min. 10m and max. 30m, and is slower than our vehicle
	  // it makes the lane unsafe to enter
	  // 0 can be returned immediately, since the lane is unsafe even if one vehicle blocks it
          else if ((delta_s_wrt_ego > 10.0) && (delta_s_wrt_ego <= 30.0) && (delta_v_wrt_ego < 0.0)) {
	      return 0;
	  }
	  // if there is a vehicle behind by min. 10m and max. 30m, and is faster than our vehicle
          // it makes the lane unsafe to enter
	  // 0 can be returned immediately, since the lane is unsafe even if one vehicle blocks it
          else if ((delta_s_wrt_ego < -10.0) && (delta_s_wrt_ego >= -30.0) && (delta_v_wrt_ego > 0.0)) {
              return 0;
          }
	  // check if the other vehicle is within a 30m to 60m distance, but slower than the ego vehicle
	  // this indicates that the destination lane is slow, but safe to enter
	  // no values to be returned here, since all vehicles need to be checked before it is confirmed as safe
	  else if ((delta_s_wrt_ego > 30.0) && (delta_s_wrt_ego <= 60.0) && (delta_v_wrt_ego < 0.0)) {
 	      is_lane_slow = true;    
	  }
      }
  }
  // if none of the vehicles affects the safety, but even if one slows down the lane
  // the lane is declared safe, but slow, meaning it can be used as a temporary lane to transition to another lane
  if (is_lane_slow) {
  return 1;
  }
  // if none of the vehicle within the vicinity of 60m blocks the lane or is slower than the ego vehicle
  // the lane is both safe and fast, and is clear to enter
  else {
  return 2;
  }
}

#endif  // HELPERS_H
