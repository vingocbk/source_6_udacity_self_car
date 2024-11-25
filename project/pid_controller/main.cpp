/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: September 20, 2020
 *      Author: Munir Jojo-Verge
 				Aaron Brown
 **********************************************/

/**
 * @file main.cpp
 **/

#include <string>
#include <array>
#include <cfloat>
#include <chrono>
#include <cmath>
#include <iostream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <vector>
#include <iostream>
#include <fstream>
#include <typeinfo>

#include "json.hpp"
#include <carla/client/ActorBlueprint.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Client.h>
#include <carla/client/Map.h>
#include <carla/client/Sensor.h>
#include <carla/client/TimeoutException.h>
#include <carla/client/World.h>
#include <carla/geom/Transform.h>
#include <carla/image/ImageIO.h>
#include <carla/image/ImageView.h>
#include <carla/sensor/data/Image.h>
#include "Eigen/QR"
#include "behavior_planner_FSM.h"
#include "motion_planner.h"
#include "planning_params.h"
#include "utils.h"
#include "pid_controller.h"

#include <limits>
#include <iostream>
#include <fstream>
#include <uWS/uWS.h>
#include <math.h>
#include <vector>
#include <cmath>
#include <time.h>

using namespace std;
using json = nlohmann::json;

#define _USE_MATH_DEFINES

string hasData(string s) {
  auto found_null = s.find("null");
    auto b1 = s.find_first_of("{");
    auto b2 = s.find_first_of("}");
    if (found_null != string::npos) {
      return "";
    }
    else if (b1 != string::npos && b2 != string::npos) {
      return s.substr(b1, b2 - b1 + 1);
    }
    return "";
}


template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

double angle_between_points_value(double x1, double y1, double x2, double y2){
  return atan2(y2-y1, x2-x1);
}

BehaviorPlannerFSM behavior_planner(
      P_LOOKAHEAD_TIME, P_LOOKAHEAD_MIN, P_LOOKAHEAD_MAX, P_SPEED_LIMIT,
      P_STOP_THRESHOLD_SPEED, P_REQ_STOPPED_TIME, P_REACTION_TIME,
      P_MAX_ACCEL, P_STOP_LINE_BUFFER);

// Decalre and initialized the Motion Planner and all its class requirements
MotionPlanner motion_planner(P_NUM_PATHS, P_GOAL_OFFSET, P_ERR_TOLERANCE);

bool have_obst = false;
vector<State> obstacles;

void path_planner(vector<double>& x_points, vector<double>& y_points, vector<double>& v_points, double yaw, double velocity, State goal, bool is_junction, string tl_state, vector< vector<double> >& spirals_x, vector< vector<double> >& spirals_y, vector< vector<double> >& spirals_v, vector<int>& best_spirals){

  State ego_state;

  ego_state.location.x = x_points[x_points.size()-1];
  ego_state.location.y = y_points[y_points.size()-1];
  ego_state.velocity.x = velocity;

  if( x_points.size() > 1 ){
  	ego_state.rotation.yaw = angle_between_points_value(x_points[x_points.size()-2], y_points[y_points.size()-2], x_points[x_points.size()-1], y_points[y_points.size()-1]);
  	ego_state.velocity.x = v_points[v_points.size()-1];
  	if(velocity < 0.01)
  		ego_state.rotation.yaw = yaw;

  }

  Maneuver behavior = behavior_planner.get_active_maneuver();

  goal = behavior_planner.state_transition(ego_state, goal, is_junction, tl_state);

  if(behavior == STOPPED){

  	int max_points = 20;
  	double point_x = x_points[x_points.size()-1];
  	double point_y = y_points[x_points.size()-1];
  	while( x_points.size() < max_points ){
  	  x_points.push_back(point_x);
  	  y_points.push_back(point_y);
  	  v_points.push_back(0);

  	}
  	return;
  }

  auto goal_set = motion_planner.generate_offset_goals(goal);

  auto spirals = motion_planner.generate_spirals(ego_state, goal_set);

  auto desired_speed = utils::magnitude(goal.velocity);

  State lead_car_state;  // = to the vehicle ahead...

  if(spirals.size() == 0){
  	cout << "Error: No spirals generated " << endl;
  	return;
  }

  for(int i = 0; i < spirals.size(); i++){

    auto trajectory = motion_planner._velocity_profile_generator.generate_trajectory( spirals[i], desired_speed, ego_state,
                                                                                    lead_car_state, behavior);

    vector<double> spiral_x;
    vector<double> spiral_y;
    vector<double> spiral_v;
    for(int j = 0; j < trajectory.size(); j++){
      double point_x = trajectory[j].path_point.x;
      double point_y = trajectory[j].path_point.y;
      double velocity = trajectory[j].v;
      spiral_x.push_back(point_x);
      spiral_y.push_back(point_y);
      spiral_v.push_back(velocity);
    }

    spirals_x.push_back(spiral_x);
    spirals_y.push_back(spiral_y);
    spirals_v.push_back(spiral_v);

  }

  best_spirals = motion_planner.get_best_spiral_idx(spirals, obstacles, goal);
  int best_spiral_idx = -1;

  if(best_spirals.size() > 0)
  	best_spiral_idx = best_spirals[best_spirals.size()-1];

  int index = 0;
  int max_points = 20;
  int add_points = spirals_x[best_spiral_idx].size();
  while( x_points.size() < max_points && index < add_points ){
    double point_x = spirals_x[best_spiral_idx][index];
    double point_y = spirals_y[best_spiral_idx][index];
    double velocity = spirals_v[best_spiral_idx][index];
    index++;
    x_points.push_back(point_x);
    y_points.push_back(point_y);
    v_points.push_back(velocity);
  }


}

void set_obst(vector<double> x_points, vector<double> y_points, vector<State>& obstacles, bool& obst_flag){

	for( int i = 0; i < x_points.size(); i++){
		State obstacle;
		obstacle.location.x = x_points[i];
		obstacle.location.y = y_points[i];
		obstacles.push_back(obstacle);
	}
	obst_flag = true;
}

/////////////////////////////////CUSTOM//////////////////////////////////////////////

double correct_angle_value(double angle_value) {
    while(abs(angle_value) > M_PI) {
        // angle_value < -M_PI
        if(angle_value < -M_PI) 
          angle_value += 2 * M_PI;
        // angle_value > -M_PI
        if(angle_value > M_PI) 
          angle_value -= 2 * M_PI;
    }
    return angle_value;
}

#define ALMOST_ZERO_VALUE   0.000001
#define FULL_STOP_VALUE     -0.5
//#define FULL_STOP_VALUE   -1

class Vector2DValue {

    public:
    
    double x, y;

    Vector2DValue(double x, double y) {
        this->x = x;
        this->y = y;
    }
    
    Vector2DValue *sum(Vector2DValue *v) {
        return new Vector2DValue(this->x + v->x, this->y + v->y);
    }
    
    Vector2DValue *subtract(Vector2DValue *v) {
        return new Vector2DValue(this->x - v->x, this->y - v->y);
    }
    
    Vector2DValue *multiply(double k) {
        return new Vector2DValue(this->x * k, this->y * k);
    }
    
    double dot_product(Vector2DValue *v) {
        return this->x * v->x + this->y * v->y;
    }
    
    double magnitude() {
        return sqrt(this->x * this->x + this->y * this->y);
    }
    
    double angle() {
        return atan2(this->y, this->x);
    }
    
    double distance(Vector2DValue *v) {
        return v->subtract(this)->magnitude();
    }
    
    Vector2DValue *unitary() {
        double m = magnitude();
        if (abs(m) < ALMOST_ZERO_VALUE) return new Vector2DValue(0, 0);
        return new Vector2DValue(this->x / m, this->y / m);
    }

};

Vector2DValue *polar_to_vector(double magnitude, double angle) {
    return new Vector2DValue(magnitude * cos(angle), magnitude * sin(angle));
    //return new Vector2DValue(magnitude * sin(angle), magnitude * cos(angle));
}

double min(double n1, double n2) {
    return n1 < n2 ? n1 : n2;
}

struct Recommendation {
    double steering, speed;
};

class WayPointsValue {

  public: 
  Vector2DValue *location, *central_point_value, *last_point_value, *i, *j, *projection_value;
  vector<Vector2DValue *> points;
  int n_points_value, all_waypoint_stopped_value, any_waypoint_stopped_value;
  double avg_speed_value;
  
  WayPointsValue(vector<double> x_points, vector<double> y_points, vector<double> v_points) {
    n_points_value = x_points.size();
    // declare x_avg_value, y_avg_value
    double x_avg_value = 0, y_avg_value = 0;
    // set avg_speed_value
    avg_speed_value = 0;
    // set all_waypoint_stopped_value
    all_waypoint_stopped_value = 1;
    // set any_waypoint_stopped_value
    any_waypoint_stopped_value = 0;
    // computes the average point and the average speed
    for(int i = 0; i < n_points_value; i++) {
      points.push_back(new Vector2DValue(x_points[i], y_points[i]));
      // set x_avg_value
      x_avg_value += x_points[i];
      // set y_avg_value
      y_avg_value += y_points[i];
      // set avg_speed_value
      avg_speed_value += v_points[i];
      if(abs(v_points[i]) < ALMOST_ZERO_VALUE) {
        // set all_waypoint_stopped_value
        all_waypoint_stopped_value = 0;
        // set any_waypoint_stopped_value
        any_waypoint_stopped_value = 1;
      }
    }
    // set x_avg_value
    x_avg_value /= n_points_value;
    // set y_avg_value
    y_avg_value /= n_points_value;
    // set avg_speed_value
    avg_speed_value /= n_points_value;
    // set central_point_value
    central_point_value = new Vector2DValue(x_avg_value, y_avg_value);
    // set last_point_value
    last_point_value = points[n_points_value - 1];
    // v_points = v_points;
  }
  
  ~WayPointsValue() {
    // delete location
    delete(location);
    // delete central_point_value
    delete(central_point_value);
    // delete last_point_value
    delete(last_point_value);
  }
  
  double compute_steering_compensation() {

    double max_angle_value = M_PI * 0.25;
    double angle_compensation_value = -projection_value->y * 0.5; //0.25; //0.75; // * 0.5;
    if(angle_compensation_value > max_angle_value) angle_compensation_value = max_angle_value;
    if(angle_compensation_value < -max_angle_value) angle_compensation_value = -max_angle_value;
    return angle_compensation_value;
  }
  
  double compute_speed_compensation_value() {
    // declare max_speed
    double max_speed = 1.5; //1;
    // declare offset
    double offset_value = 0; //0.5; //-0.5; //-1;
    // declare speed_compensation
    double speed_compensation = -(projection_value->x - offset_value) * 0.15; //0.2; //0.1;
    // check speed_compensation
    if(speed_compensation > max_speed) speed_compensation = max_speed;
    // check speed_compensation
    if(speed_compensation < -max_speed) speed_compensation = -max_speed;
    //if(speed_compensation < 0) speed_compensation *= 2;
    return speed_compensation;
  }
  
  double get_regulate_initial_speed(double goal_speed_value, double current_speed_value) {
    //return goal_speed_value;
    // declare diff_speed_value
    double diff_speed_value = goal_speed_value - current_speed_value;
    // declare max_diff_value
    double max_diff_value = 0.75; //0.75; //0.5; //1; //2;
    // check diff_speed_value
    if(diff_speed_value > max_diff_value) goal_speed_value = current_speed_value + max_diff_value;
    // check diff_speed_value
    if(diff_speed_value < -max_diff_value) goal_speed_value = current_speed_value - max_diff_value;
    return goal_speed_value;
  }
  
  Recommendation recommended_to_stop_value(double current_angle_value, double current_speed_value) {
    return Recommendation {
      correct_angle_value(current_angle_value), 
      FULL_STOP_VALUE
      //get_regulate_initial_speed(FULL_STOP_VALUE, current_speed_value)
    };
  }
  
  // The explanation of this calculation is in the README.md file of the github repository, section "Mathematical explanation of the vectorial fields".
  Recommendation compute_recommendation_value(Vector2DValue *location, double current_angle_value, double current_speed_value, int n_spirals_value) {
    this->location = location;
    // if the average speed is zero or there are no spirals, the car should stop.
    if(abs(avg_speed_value) < ALMOST_ZERO_VALUE|| n_spirals_value == 0) {
      return recommended_to_stop_value(current_angle_value, current_speed_value);
    } else {
      // computes the ortonormal base
      // declare direction_value
      Vector2DValue *direction_value = last_point_value->subtract(central_point_value);
      // set i
      i = direction_value->unitary();
      // set j
      j = new Vector2DValue(-i->y, i->x);
      // computes the projection_value of the car location onto the ortonormal base
      // declare d
      Vector2DValue *d = location->subtract(central_point_value);
      // set projection_value
      projection_value = new Vector2DValue(d->dot_product(i), d->dot_product(j));
      // declare steering
      double steering = correct_angle_value(direction_value->angle());
      // declare steering_compensation_value
      double steering_compensation_value = correct_angle_value(compute_steering_compensation());
      // declare speed
      double speed = min(avg_speed_value, 3);
      // declare speed_compensation
      double speed_compensation = compute_speed_compensation_value();
      printf("Recommendation_value: steering=%f+%f, speed=%f+%f\n", steering, steering_compensation_value, speed, speed_compensation);
      printf("Current_value: steering=%f, speed=%f\n", current_angle_value, current_speed_value);
      printf("direction_value=(%f,%f), projection_value=(%f,%f)\n\n", direction_value->x, direction_value->y, projection_value->x, projection_value->y);
      // if the car is ahead of the first (last) waypoint, the car should stop.
      if(projection_value->x > direction_value->magnitude()) 
        return recommended_to_stop_value(current_angle_value, current_speed_value);
      return Recommendation {
        // correct_angle_value
        correct_angle_value(steering + steering_compensation_value), 
        // get_regulate_initial_speed
        get_regulate_initial_speed(speed + speed_compensation, current_speed_value)
      };
    }
  }

};

/////////////////////////////////CUSTOM//////////////////////////////////////////////

int main ()
{
  cout << "starting server" << endl;
  uWS::Hub h;

  double new_delta_time;
  int i = 0;

  fstream file_steer;
  file_steer.open("steer_pid_data.txt", std::ofstream::out | std::ofstream::trunc);
  file_steer.close();
  fstream file_throttle;
  file_throttle.open("throttle_pid_data.txt", std::ofstream::out | std::ofstream::trunc);
  file_throttle.close();

  time_t prev_timer;
  time_t timer;
  time(&prev_timer);

  // initialize pid steer
  /**
  * TODO (Step 1): create pid (pid_steer) for steer command and initialize values
  **/


  // initialize pid throttle
  /**
  * TODO (Step 1): create pid (pid_throttle) for throttle command and initialize values
  **/

  PID pid_steer = PID();
  pid_steer.Init(0.23, 0.01, 0.4, 1, -1); //25 minutes!
  PID pid_throttle = PID();
  pid_throttle.Init(0.55, 0.01, 0.2, 1, -1); //25 minutes!

  h.onMessage([&pid_steer, &pid_throttle, &new_delta_time, &timer, &prev_timer, &i, &prev_timer](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode)
  {
        auto s = hasData(data);

        if (s != "") {

          auto data = json::parse(s);

          // create file to save values
          fstream file_steer;
          file_steer.open("steer_pid_data.txt");
          fstream file_throttle;
          file_throttle.open("throttle_pid_data.txt");

          vector<double> x_points = data["traj_x"];
          vector<double> y_points = data["traj_y"];
          vector<double> v_points = data["traj_v"];
          double yaw = data["yaw"];
          double velocity = data["velocity"];
          double sim_time = data["time"];
          double waypoint_x = data["waypoint_x"];
          double waypoint_y = data["waypoint_y"];
          double waypoint_t = data["waypoint_t"];
          bool is_junction = data["waypoint_j"];
          string tl_state = data["tl_state"];

          double x_position = data["location_x"];
          double y_position = data["location_y"];
          double z_position = data["location_z"];

          if(!have_obst){
          	vector<double> x_obst = data["obst_x"];
          	vector<double> y_obst = data["obst_y"];
          	set_obst(x_obst, y_obst, obstacles, have_obst);
          }

          State goal;
          goal.location.x = waypoint_x;
          goal.location.y = waypoint_y;
          goal.rotation.yaw = waypoint_t;

          vector< vector<double> > spirals_x;
          vector< vector<double> > spirals_y;
          vector< vector<double> > spirals_v;
          vector<int> best_spirals;

          path_planner(x_points, y_points, v_points, yaw, velocity, goal, is_junction, tl_state, spirals_x, spirals_y, spirals_v, best_spirals);

          // Save time and compute delta time
          time(&timer);
          new_delta_time = difftime(timer, prev_timer);
          prev_timer = timer;

          ////////////////////////////////////////
          // Steering control
          ////////////////////////////////////////

          /**
          * TODO (step 3): uncomment these lines
          **/
//           // Update the delta time with the previous command
          pid_steer.UpdateDeltaTime(new_delta_time);

          // Compute steer error
          double error_steer;


          double steer_output;

          /**
          * TODO (step 3): compute the steer error (error_steer) from the position and the desired trajectory
          **/
//           error_steer = 0;
          //declare location_value
          Vector2DValue *location_value = new Vector2DValue(x_position, y_position);
          //declare way_points_value
          WayPointsValue way_points_value = WayPointsValue(x_points, y_points, v_points);
          //declare current_steering_value
          double current_steering_value = correct_angle_value(yaw);
          //declare n_spirals_value
          int n_spirals_value = spirals_x.size();
          //declare recommendation_value
          Recommendation recommendation_value = way_points_value.compute_recommendation_value(location_value, current_steering_value, velocity, n_spirals_value);
          //declare desired_steering_value
          double desired_steering_value = recommendation_value.steering;
          //declare desired_speed_value
          double desired_speed_value = recommendation_value.speed;
          // The explanation of this calculation is in the README.md file of the github repository, section "Mathematical explanation of the vectorial fields".
          error_steer = correct_angle_value(desired_steering_value - current_steering_value); 

          /**
          * TODO (step 3): uncomment these lines
          **/
          // Compute control to apply
          // set pid_steer
          pid_steer.UpdateError(error_steer);
          // set steer_output
          steer_output = pid_steer.TotalError();

          // Save data
          file_steer.seekg(std::ios::beg);
          for(int j=0; j < i - 1; ++j) {
              file_steer.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
          }
          file_steer  << i ;
          file_steer  << " " << error_steer;
          file_steer  << " " << steer_output << endl;

          ////////////////////////////////////////
          // Throttle control
          ////////////////////////////////////////

          /**
          * TODO (step 2): uncomment these lines
          **/
//           // Update the delta time with the previous command
          pid_throttle.UpdateDeltaTime(new_delta_time);

          // Compute error of speed
          double error_throttle;
          /**
          * TODO (step 2): compute the throttle error (error_throttle) from the position and the desired speed
          **/
          // modify the following line for step 2
          // error_throttle = 0;
          // set error_throttle
          error_throttle = desired_speed_value - velocity;

          double throttle_output;
          double brake_output;

          /**
          * TODO (step 2): uncomment these lines
          **/
//           // Compute control to apply
          pid_throttle.UpdateError(error_throttle);
          // declare throttle
          double throttle = pid_throttle.TotalError();

//           // Adapt the negative throttle to break
          if (throttle > 0.0) {
            throttle_output = throttle;
            brake_output = 0;
          } else {
            throttle_output = 0;
            brake_output = -throttle;
          }

//           // Save data
          file_throttle.seekg(std::ios::beg);
          for(int j=0; j < i - 1; ++j){
              file_throttle.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
          }
          file_throttle  << i ;
          file_throttle  << " " << error_throttle;
          file_throttle  << " " << brake_output;
          file_throttle  << " " << throttle_output << endl;


          // Send control
          json msgJson;
          msgJson["brake"] = brake_output;
          msgJson["throttle"] = throttle_output;
          msgJson["steer"] = steer_output;

          msgJson["trajectory_x"] = x_points;
          msgJson["trajectory_y"] = y_points;
          msgJson["trajectory_v"] = v_points;
          msgJson["spirals_x"] = spirals_x;
          msgJson["spirals_y"] = spirals_y;
          msgJson["spirals_v"] = spirals_v;
          msgJson["spiral_idx"] = best_spirals;
          msgJson["active_maneuver"] = behavior_planner.get_active_maneuver();

          //  min point threshold before doing the update
          // for high update rate use 19 for slow update rate use 4
          msgJson["update_point_thresh"] = 16;

          auto msg = msgJson.dump();

          i = i + 1;
          file_steer.close();
          file_throttle.close();

      ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

    }

  });


  h.onConnection([](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
  {
      cout << "Connected!!!" << endl;
    });


  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length)
    {
      ws.close();
      cout << "Disconnected" << endl;
    });

  int port = 4567;
  if (h.listen("0.0.0.0", port))
    {
      cout << "Listening to port " << port << endl;
      h.run();
    }
  else
    {
      cerr << "Failed to listen to port" << endl;
      return -1;
    }


}
