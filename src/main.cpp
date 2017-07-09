#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "MPC.h"
#include "json.hpp"

using json = nlohmann::json;

// Conversion constants.
const double kSteeringAngleToActuator = 2.294;
const double kMphToMps = 0.447;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s)
{
	auto found_null = s.find("null");
	auto b1 = s.find_first_of("[");
	auto b2 = s.rfind("}]");

	if (found_null != string::npos)
		return "";

	if (b1 != string::npos && b2 != string::npos)
		return s.substr(b1, b2 - b1 + 2);

	return "";
}

int main()
{
	uWS::Hub h;

	// Create MPC Settings
	MPC_Settings fast_and_safe_mpc;
	fast_and_safe_mpc.N						= 8;
	fast_and_safe_mpc.dt					= 0.15;
	fast_and_safe_mpc.Lf					= 2.67;
	fast_and_safe_mpc.target_v				= 50;
	fast_and_safe_mpc.cte_cost_w			= 200;
	fast_and_safe_mpc.v_cost_w				= 1;
	fast_and_safe_mpc.epsi_cost_w			= 15000;
	fast_and_safe_mpc.steer_cost_w			= 80000;
	fast_and_safe_mpc.steer_delta_cost_w	= 80000;

	MPC_Settings racecar_mpc;
	racecar_mpc.N							= 6;
	racecar_mpc.dt							= 0.15;
	racecar_mpc.Lf							= 2.67;
	racecar_mpc.target_v					= 60;
	racecar_mpc.cte_cost_w					= 100;
	racecar_mpc.v_cost_w					= 1;
	racecar_mpc.epsi_cost_w					= 5000;
	racecar_mpc.steer_cost_w				= 1000;
	racecar_mpc.steer_delta_cost_w			= 1000;

	MPC mpc(fast_and_safe_mpc);

	h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char* data, size_t length,
			uWS::OpCode opCode)
		{
			// "42" at the start of the message means there's a websocket message event.
			// The 4 signifies a websocket message
			// The 2 signifies a websocket event
			auto sdata = string(data).substr(0, length);
			cout << sdata << endl;
			if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2')
			{
				auto s = hasData(sdata);
				if (s != "")
				{
					auto j = json::parse(s);
					auto event = j[0].get<string>();
					if (event == "telemetry")
					{
						// Waypoints.
						vector<double> ptsx = j[1]["ptsx"];
						vector<double> ptsy = j[1]["ptsy"];

						// Get the current basic state.
						BasicState new_state;
						new_state.x			= j[1]["x"];
						new_state.y			= j[1]["y"];
						new_state.psi		= j[1]["psi"];
						new_state.v			= double(j[1]["speed"]) * kMphToMps;
						new_state.delta		= j[1]["steering_angle"];
						new_state.a			= j[1]["throttle"];

						// Solve with the MPC.
						auto output			= mpc.Solve(new_state, ptsx, ptsy);
						auto steer_value	= output[0] * kSteeringAngleToActuator;
						auto throttle_value = output[1];

						json msgJson;

						// Send actuator input.
						msgJson["steering_angle"] = steer_value;
						msgJson["throttle"] = throttle_value;

						// Display green MPC Waypoints.
						msgJson["mpc_x"] = mpc.ai_waypoints_x_;
						msgJson["mpc_y"] = mpc.ai_waypoints_y_;

						// Display yellow map waypoints.
						msgJson["next_x"] = mpc.map_waypoints_x_;
						msgJson["next_y"] = mpc.map_waypoints_y_;

						auto msg = "42[\"steer\"," + msgJson.dump() + "]";
						// std::cout << msg << std::endl;

						// Latency
						this_thread::sleep_for(chrono::milliseconds(100));
						ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
					}
				}
				else
				{
					// Manual driving
					std::string msg = "42[\"manual\",{}]";
					ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
				}
			}
		});

	// We don't need this since we're not using HTTP but if it's removed the
	// program
	// doesn't compile :-(
	h.onHttpRequest([](uWS::HttpResponse* res, uWS::HttpRequest req, char* data,
			size_t, size_t)
		{
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

	h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
		{
			std::cout << "Connected!!!" << std::endl;
		});

	h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
			char* message, size_t length)
		{
			ws.close();
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
