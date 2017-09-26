#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include <math.h>
#include "ukf.h"
#include "tools.h"

using namespace std;

// for convenience
using json = nlohmann::json;

/**
 * Checks if the SocketIO event has JSON data.
 * If there is data the JSON object in string format will be returned,
 * else the empty string "" will be returned.
 */
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

/**
 * Main loop to process measurements received from Udacity simulator via
 * uWebSocket messages.  After measurement is received, process it using
 * ukf.ProcessMeasurement(meas_package) and send resulting state
 * estimates and RMSE values back to the simulator for visualization.
 */
int main() {
  uWS::Hub h;

  // Create a Kalman Filter instance
  UKF ukf;

  // Used to compute the RMSE later
  Tools tools;
  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;
  
  // Create output file to store data values
  ofstream out_file_;
  out_file_.open("log_data_output.txt");
  out_file_ << "timestamp,Laser/Radar,NIS,px,py,vx,vy,yaw,yawd,"
            << "px_act,py_act,vx_act,vy_act,yaw_act,yawd_act\n";

  h.onMessage([&ukf,&tools,&estimations,&ground_truth,&out_file_]
              (uWS::WebSocket<uWS::SERVER> ws, char *data,
               size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event

    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(std::string(data));
      if (s != "") {
      	
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          string sensor_measurment = j[1]["sensor_measurement"];
          
          MeasurementPackage meas_package;
          istringstream iss(sensor_measurment);
          long long timestamp;

          // Reads first element from the current line
          string sensor_type;
          iss >> sensor_type;

          if (sensor_type.compare("L") == 0) {
            meas_package.sensor_type_ = MeasurementPackage::LASER;
            meas_package.raw_measurements_ = VectorXd(2);
            float px;
            float py;
            iss >> px;
            iss >> py;
            meas_package.raw_measurements_ << px, py;
            iss >> timestamp;
            meas_package.timestamp_ = timestamp;
          }
          else if (sensor_type.compare("R") == 0) {
            meas_package.sensor_type_ = MeasurementPackage::RADAR;
            meas_package.raw_measurements_ = VectorXd(3);
            float ro;
            float theta;
            float ro_dot;
            iss >> ro;
            iss >> theta;
            iss >> ro_dot;
            meas_package.raw_measurements_ << ro,theta, ro_dot;
            iss >> timestamp;
            meas_package.timestamp_ = timestamp;
          }
          else {
            timestamp = 0; // invalid measurement received, should not occur
          }
          
          // Reset RMSE if timestamp resets (restart dataset) or
          // has a gap > 1000 sec (switch dataset) to allow repeated simulations
          if ((timestamp < ukf.prev_time_us_) ||
              (abs(ukf.prev_time_us_ - timestamp) > 1000000000.0)) {
            ground_truth.clear();
            estimations.clear();
          }
          
          float x_gt;
          float y_gt;
          float vx_gt;
          float vy_gt;
          float yaw_gt;
          float yawd_gt;
          iss >> x_gt;
          iss >> y_gt;
          iss >> vx_gt;
          iss >> vy_gt;
          iss >> yaw_gt;
          iss >> yawd_gt;
          
          VectorXd gt_values(4);
          gt_values(0) = x_gt;
          gt_values(1) = y_gt; 
          gt_values(2) = vx_gt;
          gt_values(3) = vy_gt;
          
          // Log output for each measurement with ground truth reference
          std::cout << "\nMeasurement received: " << sensor_type <<
                        timestamp << "\n" << std::endl;
          std::cout << "Ground truth: \n" << gt_values << "\n" << std::endl;
          
          // Call ProcessMeasurment(meas_package) for Kalman filter
          ukf.ProcessMeasurement(meas_package);

          // Push the current estimated x,y positon from the Kalman
          // filter's state vector
          double px = ukf.x_(0);
          double py = ukf.x_(1);
          double v  = ukf.x_(2);
          double yaw = ukf.x_(3);
          double yawd = ukf.x_(4);
          double vx = cos(yaw)*v;
          double vy = sin(yaw)*v;
          VectorXd estimate(4);
          estimate(0) = px;
          estimate(1) = py;
          estimate(2) = vx;
          estimate(3) = vy;
          
          // Only accumulate estimates if state x started calculating values.
          // In case of using only one measurement type and the first
          // measurement is the other sensor type, the state x will be
          // initialized as all zero.
          if (!ukf.x_.isZero()) {
            ground_truth.push_back(gt_values);
            estimations.push_back(estimate);
          }
          
          // Calculate RMSE of accumulated state estimations vs ground truth
          VectorXd RMSE = tools.CalculateRMSE(estimations, ground_truth);

          // Log output for Kalman filter state and covariance result
          std::cout << "x:\n" << ukf.x_ << "\n" << std::endl;
          std::cout << "P:\n" << ukf.P_ << "\n" << std::endl;
          std::cout << "RMSE:\n" << RMSE << "\n" << std::endl;
          
          // Send resulting state estimates and RMSE back to simulator
          json msgJson;
          msgJson["estimate_x"] = px;
          msgJson["estimate_y"] = py;
          msgJson["rmse_x"] =  RMSE(0);
          msgJson["rmse_y"] =  RMSE(1);
          msgJson["rmse_vx"] = RMSE(2);
          msgJson["rmse_vy"] = RMSE(3);
          auto msg = "42[\"estimate_marker\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
          // Output the data values to log file
          if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
            out_file_ << timestamp <<",L,"<< ukf.NIS_laser_ <<","<<
            px <<","<< py <<","<< vx <<","<< vy <<","<< yaw <<","<< yawd <<
            ","<< x_gt <<","<< y_gt <<","<< vx_gt <<","<< vy_gt <<","<<
            yaw_gt <<","<< yawd_gt <<"\n";
          }
          else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
            out_file_ << timestamp << ",R," << ukf.NIS_radar_ <<","<<
            px <<","<< py <<","<< vx <<","<< vy <<","<< yaw <<","<< yawd <<
            ","<< x_gt <<","<< y_gt <<","<< vx_gt <<","<< vy_gt <<","<<
            yaw_gt <<","<< yawd_gt <<"\n";
          }
	  
        } // (event == "telemetry")
      } // (s != "")
      else { // (s == "")
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    } // websocket message 42
  }); // h.onMessage()

  // We don't need this since we're not using HTTP but if it's removed the
  // program doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req,
                     char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h, &out_file_](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    out_file_.close(); // Close data log file
    std::cout << "Data log file written: log_data_output.txt" << std::endl;
    //ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
