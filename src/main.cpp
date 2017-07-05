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

void CreateWayPoints(Eigen::VectorXd& x_wp, Eigen::VectorXd& y_wp, const vector<double>& x_mp, const vector<double>& y_mp, double x, double y, double psi)
{
	auto cos_theta = cos(-psi);
	auto sin_theta = sin(-psi);

	for (auto i = 0 ; i < x_mp.size(); i++)
	{
		auto dx = x_mp[i] - x;
		auto dy = y_mp[i] - y;

		x_wp(i) = dx * cos_theta - dy * sin_theta;
		y_wp(i) = dx * sin_theta + dy * cos_theta;
	}
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
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
		  double delta = j[1]["steering_angle"];
		  double a = j[1]["throttle"];
		  v *= (1609.344 / 3600);

			// Transform Waypoints to Local
		  Eigen::VectorXd local_wp_x(ptsx.size());
		  Eigen::VectorXd local_wp_y(ptsy.size());
		  CreateWayPoints(local_wp_x, local_wp_y, ptsx, ptsy, px, py, psi);

			// Poly fit a polynomial to these WPs:
		  auto poly = polyfit(local_wp_x, local_wp_y, 3);
			
			// Create the state vector.
		  double cte = polyeval(poly, 0);
		  double epsi = -atan(poly[1]);

		  std::cout << endl;
		  std::cout << endl;

		  std::cout << "cte: " << cte << endl;
		  std::cout << "epsi: " << epsi << endl;

		  std::cout << endl;
		  std::cout << endl;

		  // Latency for predicting time at actuation
		  const double dt = 0.1;

		  const double Lf = 2.67;

		  // Predict future state (take latency into account)
		  // x, y and psi are all zero in the new reference system
		  double pred_px = 0.0 + v * dt;               // psi is zero, cos(0) = 1, can leave out
		  const double pred_py = 0.0;                        // sin(0) = 0, y stays as 0 (y + v * 0 * dt)
		  double pred_psi = 0.0 + v * -delta / Lf * dt;
		  double pred_v = v + a * dt;
		  double pred_cte = cte + v * sin(epsi) * dt;
		  double pred_epsi = epsi + v * -delta / Lf * dt;

		  Eigen::VectorXd state_vector(6);
		  state_vector << pred_px, pred_py, pred_psi, pred_v, pred_cte, pred_epsi;

		  auto actuator_output = mpc.Solve(state_vector, poly);


          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */

		  // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
		  // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].

		  const double angle_norm_factor = deg2rad(25);
		  double steer_value = actuator_output[0] / angle_norm_factor;
          double throttle_value = actuator_output[1];

          json msgJson;
          

          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

		  for (size_t i = 2; i < actuator_output.size(); ++i) {
			  if (i % 2 == 0) mpc_x_vals.push_back(actuator_output[i]);
			  else            mpc_y_vals.push_back(actuator_output[i]);
		  }

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
		 /* vector<double> next_x_vals(local_wp_x.data(), local_wp_x.data() + local_wp_x.rows() * local_wp_x.cols());
		  vector<double> next_y_vals(local_wp_y.data(), local_wp_y.data() + local_wp_y.rows() * local_wp_y.cols());*/

		  vector<double> next_x_vals;
		  vector<double> next_y_vals;
		  double poly_inc = 2.5; // step on x
		  int num_points = 25;    // how many point "in the future" to be plotted
		  for (int i = 1; i < num_points; ++i) {
			  double future_x = poly_inc * i;
			  double future_y = polyeval(poly, future_x);
			  next_x_vals.push_back(future_x);
			  next_y_vals.push_back(future_y);
		  }

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
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
