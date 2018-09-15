#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

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
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}


// global2car converts global coordinates to car coordinates
// returns a touple with x and y in car coordinates
Eigen::VectorXd global2car(const double global_x, const double global_y,
                           const Eigen::VectorXd &global_car_state) {
  double car_gx = global_car_state[0];
  double car_gy = global_car_state[1];
  double car_theta = -global_car_state[2];

  double dx = global_x - car_gx;
  double dy = global_y - car_gy;


  Eigen::Matrix2d trans_matrix;
  trans_matrix << cos(car_theta),  -sin(car_theta),
                  sin(car_theta),   cos(car_theta);

  Eigen::Vector2d gc = {dx, dy};

  return trans_matrix * gc;
}


// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {

    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];

          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */

          // choose only some of the waypoints to prevent "overfitting"
          int poly_ind[] = {0, 1, 2, 5};

          // required by global2car function
          Eigen::VectorXd car_state(3);
          car_state << px, py, psi;


          Eigen::VectorXd path_x(4);
          Eigen::VectorXd path_y(4);

          // convert global waypoints to car coords waypoints
          for(int i=0; i<4; ++i) {
            Eigen::VectorXd result;
            result = global2car(ptsx[poly_ind[i]], ptsy[poly_ind[i]], car_state);
            path_x[i] = result[0];
            path_y[i] = result[1];
          }

          // fit a polynomial of the 3rd degree to the waypoints
          size_t poly_order = 3;
          Eigen::VectorXd pcoeffs;
          pcoeffs = polyfit(path_x, path_y, poly_order);

          // cte = distance between a car and the desired path
          // at time t0, where x=0 is car's x positon
          double cte = polyeval(pcoeffs, 0);

          // difference between car's angle (=0)
          // and a vector tangent to the Fitted
          // polynomial at x=0:
          // y'= pcoeffs[1] + pcoeffs[2]*0 + pcoeffs[3]*0 = pcoeffs[1]
          double epsi = 0 - atan(pcoeffs[1]);

          // car's state which we pass to the IPOPT
          // at time t0 car's position, and yaw are equal to 0.
          Eigen::VectorXd state(6);
          state << 0, 0, 0, v, cte, epsi;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          // show waypoints up to 80m ahead
          for (double i=0.0; i<80; i += 5) {
            next_x_vals.push_back(i);
            next_y_vals.push_back(polyeval(pcoeffs, i));
          }


          // pass x and y values of the waypoints up to the 30th meter,
          // to the mpc.Solve function.
          // (Required to calculate integral for the loss function)

          double *x_ptr = &next_x_vals[0];
          double *y_ptr = &next_y_vals[0];

          Eigen::Map<Eigen::VectorXd> eigen_x_vals(x_ptr, 30);
          Eigen::Map<Eigen::VectorXd> eigen_y_vals(y_ptr, 30);

          vector<double> result = mpc.Solve(state, pcoeffs, eigen_x_vals, eigen_y_vals);

          double steer_value = result[0];
          double throttle_value = result[1];

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value/deg2rad(25);
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          // plot the planned path
          // start from the second element of the result vector.
          // result=<delta, a, x0, y0, x1, y1, ... , xN-1, yN-1>
          for (int i=2; i<result.size(); ++i) {
            if (i % 2 == 0) {
              mpc_x_vals.push_back(result[i]);
            } else {
              mpc_y_vals.push_back(result[i]);
            }
          }

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
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
