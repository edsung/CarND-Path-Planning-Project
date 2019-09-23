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
#include <map>

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
  
  //setting the initial lane the self-driving car is located. 
  int lane = 1;
 
  //setting the speed limit. 
  double ref_vel = 0.0;
  
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &lane, &ref_vel, &max_s]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
     
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    struct FSM {
      int id;
      int status;
      double cost;
      double s_pos;
    };
   
    vector<FSM> states;
    //FSM state;
    
                
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
          
          double KL_Cost, CLR_Cost, CLL_Cost;
          int numCarLeftLane,numCarRightLane;  	   
          int prev_size = previous_path_x.size();
          
          if(prev_size > 0)
          {
            //this the "goal" of the path. 
            car_s = end_path_s;
          }
          
          bool too_close = false;
          bool right_lane = false;
          bool left_lane = false;
          bool KL = true, CLL = false, CLR = false;
                 
         /* for(int i = 0;i < cars_ln0.size();i++){
            std::cout<<"lane 0 s data: "<<cars_ln0[j].s<<std::endl;
          }*/
          double fusion_max_s = 0;
          //std::cout<<"Main car is in lane number: "<<lane<<std::endl;
          for(int i=0; i<sensor_fusion.size();i++)
          {
           	double d = sensor_fusion[i][6];
            if (fusion_max_s < sensor_fusion[i][5]){
              fusion_max_s = sensor_fusion[i][5];
            }
            
            double vx;
            double vy;
            double check_speed;
            double check_car_s;
            
            if(d <(2+4*lane+2) && d > (2+4*lane-2))
            {
              vx = sensor_fusion[i][3];
              vy = sensor_fusion[i][4];
              check_speed = sqrt(vx*vx+vy*vy);
              check_car_s = sensor_fusion[i][5];
              
              check_car_s +=((double)prev_size*0.02*check_speed);
              //std::cout<<"fusion id: "<<sensor_fusion[i][0]<<" d: "<<d<<" car_d: "<<car_d<<", check_car_s: "<<check_car_s<<", car_s: "<<car_s<<", check_speed: "<< check_speed<<", car_speed: "<<car_speed<<std::endl;
              if((check_car_s > car_s) && (check_car_s-car_s) < 30)// && abs(check_speed-car_speed) > 1)
              {
                
                too_close = true;
                KL_Cost = exp(-(abs(check_car_s-car_s)));
                FSM state;
                state.id = sensor_fusion[i][0];
                state.cost = KL_Cost;
                state.status = 0;
                state.s_pos = sensor_fusion[i][5];
                states.push_back(state);                
              }
              
              
              //KL_Cost = 1 - exp(-(abs(check_car_s-car_s)));
                  
                               
              //costs.insert(1,KL_Cost);
            }  
            /*else{
              FSM state;
              state.cost = -99;
              state.status = 0;
              states.push_back(state);
            }*/
               
            double d_left_lane = sensor_fusion[i][6];
            double vx1;
            double vy1 ;
            double check_speed1;
            double check_car_s1;
            double check_car_s1_1 ;
            if(d_left_lane <(2+4*(lane - 1)+2) && d_left_lane > (2+4*(lane - 1)-2) && (lane == 1 || lane == 2))
            {
                vx1 = sensor_fusion[i][3];
                vy1 = sensor_fusion[i][4];
                check_speed1 = sqrt(vx1*vx1+vy1*vy1);
                check_car_s1 = sensor_fusion[i][5];
                check_car_s1_1 = sensor_fusion[i][5];
              
                //check_car_s1 +=((double)prev_size*0.02*check_speed1);
                check_car_s1_1 +=((double)prev_size*0.02*check_speed1);
                //double delta_s1 = ((double)prev_size*0.02*check_speed1)
               // std::cout<<"fusion id: "<<sensor_fusion[i][0]<<" d_left_lane: "<<d_left_lane<<" car_d: "<<car_d<<", check_car_s1: "<<check_car_s1<<", car_s: "<<car_s<<", check_speed1: "<< check_speed1<<", car_speed: "                           <<car_speed<<std::endl;     
               if(abs(check_car_s1 - car_s) <  10 && lane == 2) //(car_s-30) > check_car_s1 || 
               {
                                  
                  CLL_Cost = exp(-abs((check_car_s1-fusion_max_s)));
                  FSM state;
                  state.id = sensor_fusion[i][0];               
                  state.cost = 1000;
                  state.status = 0;
                  state.s_pos = sensor_fusion[i][5];
                  states.push_back(state); 
                  CLL = false;
                  CLR = false;
                }
              	else if( abs(check_car_s1 - car_s) <  10 && lane == 1){
                  CLL = false;
                  left_lane = true;
                  CLL_Cost = exp(-abs((check_car_s1-fusion_max_s)));
                  FSM state;
                  state.id = sensor_fusion[i][0];               
                  state.cost = CLL_Cost;
                  state.status = 1;
                  state.s_pos = sensor_fusion[i][5];
                  states.push_back(state); 
                }
                else{
                  CLL = true;
                }
              
                //CLL_Cost = 1 - exp(-(1/abs(check_car_s1-car_s)));
                                        
               // costs.insert(pair<int, double>(2,CLR_Cost));
            }
                
                
            double d_right_lane = sensor_fusion[i][6];
            double vx2;
            double vy2;
            double check_speed2;
            double check_car_s2, check_car_s2_1;
            
            if(d_right_lane <(2+4*(lane + 1)+2) && d_right_lane > (2+4*(lane +1)-2) && (lane == 1 || lane == 0))
            {
                vx2 = sensor_fusion[i][3];
                vy2 = sensor_fusion[i][4];
                check_speed2 = sqrt(vx2*vx2+vy2*vy2);
                check_car_s2 = sensor_fusion[i][5];
                check_car_s2_1 = sensor_fusion[i][5];

                //std::cout<<"fusion id: "<<sensor_fusion[i][0]<<" d_right_lane: "<<d_right_lane<<" car_d: "<<car_d<<", check_car_s2: "<<check_car_s2<<", car_s: "<<car_s<<", check_speed2: "<< check_speed2<<", car_speed: "<<car_speed<<std::endl;
                //check_car_s2 +=((double)prev_size*0.02*check_speed2);
                check_car_s2_1 +=((double)prev_size*0.02*check_speed2);
                      //(check_car_s2 - car_s)>5
                if(abs(check_car_s2 - car_s) <  10 && lane == 1)//(car_s-30) > check_car_s2 || 
                {
                   right_lane = true;
                   CLR = false;
                   CLR_Cost = exp(-(abs(check_car_s2-car_s)));
                   FSM state;
                   state.id = sensor_fusion[i][0];
                   state.cost = CLR_Cost;
                   state.status = 0;
                   state.s_pos = sensor_fusion[i][5];
                   states.push_back(state);   
                }
                else if( abs(check_car_s2 - car_s) < 10 && lane == 0)
                {
                   right_lane = true;
                   CLR = false;
                   CLL = false;
                   CLR_Cost = exp(-(abs(check_car_s2-car_s)));
                   FSM state;
                   state.id = sensor_fusion[i][0];
                   state.cost = CLR_Cost;
                   state.status = 2;
                   state.s_pos = sensor_fusion[i][5];
                   states.push_back(state);   
                }
                else{
                  CLL = false;
                  CLR = true;
                }
                //CLR_Cost = 1 - exp(-(1/abs(check_car_s2-car_s)));                  
                //costs.insert(3,CLL_Cost);
                
             }
            
            if((check_car_s > car_s) && (check_car_s-car_s) < 30 && (check_car_s1 > car_s) && (check_car_s1-car_s) < 30 && (check_car_s2_1 > car_s) && (check_car_s2_1-car_s) < 30)
            {  
              too_close = false;
              CLL = true;
              CLR = true;
            }
            else if((check_car_s > car_s) && (check_car_s-car_s) < 30 && (check_car_s1_1 > car_s) && (check_car_s1_1-car_s) < 30 && (lane == 1 || lane == 2))
            {
              too_close = true;
              CLL = true;
              CLR = false;
            }
            else if((check_car_s > car_s) && (check_car_s-car_s) < 30 && (check_car_s2_1 > car_s) && (check_car_s2_1-car_s) < 30 && (lane == 1 || lane == 0))
            {
              too_close = true;
              CLL = false;
              CLR = true;
            }
          }
         /* double lowest_cost = 100.0;
          int final_state;
          
          std::cout<<"States size: "<<states.size()<<std::endl;
          
          if(states.size()>0){
            for(int i =0;i<states.size();i++){
              std::cout<<"Fusion ID: "<<states[i].id<<", Cost: "<<states[i].cost<<", S Position: "<<states[i].s_pos<<std::endl;
              if(states[i].cost < lowest_cost){
                final_state = states[i].status;
              } 
            }
          }
          else
            final_state = 0;*/
          
          //std::cout<<"Final State is "<<final_state<<std::endl;
          
          std::cout<<"KL is: "<<KL<<" CLL is: "<<CLL<<" CLR is: "<<CLR<<std::endl;
          
         // states.clear();                  
                             
          if(too_close)
          {      
              KL = false;
             
              if ((KL == true && CLL == true && CLR == true) || (KL == false && CLL == false && CLR == false) ){ //|| (KL == true && CLL == true && CLR == true)
                lane = lane;
                ref_vel -= 0.224;
              }
              else if (KL == false && CLL == true && CLR == false && lane == 1){
                std::cout<<"Changing to left lane."<<std::endl;
                lane = 0;
              }
              else if (KL == false && CLL == true && CLR == false && lane == 2){
                std::cout<<"Changing to left lane."<<std::endl;
                lane = 1;
              }
              else if(KL == false && CLL == false && CLR == true && lane == 0){
                std::cout<<"Changing to right lane."<<std::endl; 
                lane = 1;
              }         
              else if(KL == false && CLL == false && CLR == true && lane == 1){
                std::cout<<"Changing to right lane."<<std::endl; 
                lane = 2;
              }
              else if(KL == false && CLL == true && CLR == true && lane == 1){
                std::cout<<"Changing to right lane."<<std::endl; 
                lane = 0;
              }                
             
              
          }            
          else if (ref_vel < 49.5)
          {
            std::cout<<"Speeding up."<<std::endl; 
            ref_vel += 0.224;
          }
          
          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          vector<double> ptsx;
          vector<double> ptsy;
          
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          
          if(prev_size < 2)
          {
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);
            
            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);
            
            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          }
          else
          {
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];
            
            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);
            
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);
            
            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }
          
          vector<double> next_wp0 = getXY(car_s+30,(2+4*lane),map_waypoints_s, map_waypoints_x,map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+60,(2+4*lane),map_waypoints_s, map_waypoints_x,map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+90,(2+4*lane),map_waypoints_s, map_waypoints_x,map_waypoints_y);
          
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);
          
          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);
          
          for(int i = 0;i < ptsx.size();i++)
          {
            double shift_x = ptsx[i]-ref_x;
            double shift_y = ptsy[i]-ref_y;
            
            ptsx[i] = (shift_x *cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
            ptsy[i] = (shift_x *sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
            
          }
          
          tk::spline s;
          
          s.set_points(ptsx, ptsy);
          
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          
          for (int i = 0; i<previous_path_x.size(); i++)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          double target_x = 30;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x*target_x+target_y*target_y);
          
          double x_add_on = 0;
          
          for(int i=0; i <= 50-previous_path_x.size(); i++)
          {
            double N = (target_dist/(0.02*ref_vel/2.24));
            double x_point = x_add_on+(target_x)/N;
            double y_point = s(x_point);
            
            x_add_on = x_point;
            
            double x_ref = x_point;
            double y_ref = y_point;
            
            x_point = (x_ref * cos(ref_yaw)-y_ref*sin(ref_yaw));
            y_point = (x_ref * sin(ref_yaw)+y_ref*cos(ref_yaw));
            
            x_point += ref_x;
            y_point += ref_y;
            
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
            
          }
          
		  json msgJson;
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