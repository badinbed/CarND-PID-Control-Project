#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include "twiddle.h"
#include <math.h>
//
//
// pid.Init(.1, 0.0, 0.0); -> 0.02 const
//
//
//
//PID: 0.075, 0, 0.35
// 50:   16  16
//100:   95  79
//150:  141  46
//200:  227  86
//250:  372 145
//300:  497 125
//Debugging has finished
//
//
// for convenience
using json = nlohmann::json;
using namespace std;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main()
{
  uWS::Hub h;

  PID pidSteering;
  pidSteering.Init(.15, 0, 1.5);

  PID pidSpeed;
  pidSpeed.Init(0.04, 0.00005, 0.0);
  const double targetSpeed = 30.0;

  Twiddle twiddle({pidSteering.KP(), pidSteering.KD()}, {0.01, 0.01}, 0.004);
  bool doTwiddle = true;
  int speedCheck = 500;
  int maxFrame = 5800;
  h.onMessage([&pidSteering, &pidSpeed, targetSpeed, &twiddle, doTwiddle, speedCheck, maxFrame](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {

      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {

            // j[1] is the data JSON object
            double cte = std::stod(j[1]["cte"].get<std::string>());
            double speed = std::stod(j[1]["speed"].get<std::string>());
            double angle = std::stod(j[1]["steering_angle"].get<std::string>());

            static int n_frame = 0;
            ++n_frame;

            if(doTwiddle) {
                if(n_frame > maxFrame || (pidSteering.TotalError() > twiddle.MinimumError()) || (n_frame%speedCheck == 0 && speed < 0.7*targetSpeed)) {
                    cout << (n_frame > maxFrame) << "  " << (pidSteering.TotalError() > twiddle.MinimumError()) << "  " << (n_frame%speedCheck == 0 && speed < 0.7*targetSpeed) << endl;

                    double error = (n_frame%speedCheck == 0 && speed < 0.7*targetSpeed) ? numeric_limits<double>::max() : pidSteering.TotalError();
                    vector<double> p;
                    if(!twiddle.Next(&p, error))
                        exit(0);

                    pidSteering.Init(p[0],pidSpeed.KI(), p[2]);
                    n_frame = 0;

                    std::string msg = "42[\"reset\",{}]";
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                    return;
                }
            }



          double steer_value = std::min(1.0, std::max(-1.0, pidSteering.Control(cte)));

          double speed_value = std::min(1.0, std::max(-1.0, pidSpeed.Control(speed - targetSpeed)));

          // DEBUG
          auto ce = pidSteering.CurrentErrors();
          auto cwe = pidSteering.CurrentWeightedErrors();

//          std::cout << "----------------------------------------------" << std::endl << std::endl;

//          std::cout << "StCE:   " << ce[0] <<  ", " << ce[1] <<  ", " << ce[2] << std::endl;
//          std::cout << "StCWE:  " << cwe[0] <<  ", " << cwe[1] <<  ", " << cwe[2] << std::endl;
//          std::cout << "St:     " << steer_value << std::endl << std::endl;

//          ce = pidSpeed.CurrentErrors();
//          cwe = pidSpeed.CurrentWeightedErrors();

//          std::cout << "SpCE:  " << ce[0] <<  ", " << ce[1] <<  ", " << ce[2] << std::endl;
//          std::cout << "SpCWE: " << cwe[0] <<  ", " << cwe[1] <<  ", " << cwe[2] << std::endl;
//          std::cout << "A:     " << avgAngle << std::endl;
//          std::cout << "T:     " << target_speed << std::endl;
//          std::cout << "Sp:    " << speed_value << std::endl << std::endl;

//          static int pre_te = 0;
//          if(n_frame == 1) {

//              std::cout << endl << "PID: " << pidSteering.KP() << ", " << pidSteering.KI() << ", " << pidSteering.KD() << endl;
//          }
//          else if(n_frame%500 == 0) {
//              int te = pidSteering.TotalError() + 0.5;
//              int delta_te = te - pre_te;
//              std::cout << setw(3) << n_frame << ": " <<  setw(4) << te << setw(4) << delta_te << std::endl;
//              pre_te = te;
//          } else if(n_frame > 2500) {
//              std::string msg = "42[\"reset\",{}]";
//              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
//              n_frame = 0;
//              return;
//          }

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = speed_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        } else {
         std::cout << s << std::endl;
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
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
    //std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
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
