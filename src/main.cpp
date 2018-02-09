#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "Map.h"

using namespace std;

// for convenience
using json = nlohmann::json;

static const int WEBSOCKECT_OK_DISCONNECT_CODE = 1000;
static const string RESET_SIMULATOR_WS_MESSAGE = "42[\"reset\", {}]";
static const string MANUAL_WS_MESSAGE = "42[\"manual\",{}]";

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s);

void sendMessage(uWS::WebSocket<uWS::SERVER> ws, string msg) { ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT); }

json process_telemetry_data(Map map, json data);

int main() {
    uWS::Hub h;
    bool firstTimeConnecting = true;

    Map map;

    // Waypoint map to read from
    string map_file = "../data/highway_map.csv";

    map.load_map(map_file);

    h.onMessage( [&map] (
            uWS::WebSocket<uWS::SERVER> ws,
            char *data,
            size_t length,
            uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        //auto sdata = string(data).substr(0, length);
        //cout << sdata << endl;
        if (length && length > 2 && data[0] == '4' && data[1] == '2') {

            auto s = hasData(data);

            if (s != "") {
                auto j = json::parse(s);

                string event = j[0].get<string>();

                if (event == "telemetry") {
                    json msgJson = process_telemetry_data(map, j[1]); // j[1] is the data JSON object

                    auto msg = "42[\"control\"," + msgJson.dump() + "]";

                    //this_thread::sleep_for(chrono::milliseconds(1000));
                    sendMessage(ws, msg);
                }
            } else {
                // Manual driving
                sendMessage(ws, MANUAL_WS_MESSAGE);
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

    h.onConnection([&h, &firstTimeConnecting](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        if (firstTimeConnecting) {
            cout << "Connected for first time!!!  Restarting simulator!" << endl;
            firstTimeConnecting = false;
            sendMessage(ws, RESET_SIMULATOR_WS_MESSAGE);
        } else {
            cout << "Reconnected to simulator, success!" << endl;
        }
    });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                           char *message, size_t length) {
        if (code == WEBSOCKECT_OK_DISCONNECT_CODE) {
            cout << "Disconnected normally." << endl;
        } else {
            cout << "Unexpected Disconnect with code: " << code << "!" << endl;
        }

        cout << "WARN: Not closing WS because we get bad access exception! (Which one cannot catch in C++!?)" << endl;
        // StackOverflow https://stackoverflow.com/q/19304157 suggested that error code 1006 means to check onError
        // But, but, but, adding onError here doesn't get called at all, everything seems alright
        // (other than this ws.close() exc_bad_access)
        // ws.close(code, message, length);
        // Besides ^^, if we're getting disconnection method, then the connection is already closed??
    });

    h.onError([](void *user) {
        // Code copied from: https://github.com/uNetworking/uWebSockets/blob/master/tests/main.cpp
        switch ((long) user) {
            case 1:
                cout << "Client emitted error on invalid URI" << endl;
                break;
            case 2:
                cout << "Client emitted error on resolve failure" << endl;
                break;
            case 3:
                cout << "Client emitted error on connection timeout (non-SSL)" << endl;
                break;
            case 5:
                cout << "Client emitted error on connection timeout (SSL)" << endl;
                break;
            case 6:
                cout << "Client emitted error on HTTP response without upgrade (non-SSL)" << endl;
                break;
            case 7:
                cout << "Client emitted error on HTTP response without upgrade (SSL)" << endl;
                break;
            case 10:
                cout << "Client emitted error on poll error" << endl;
                break;
            case 11:
                static int protocolErrorCount = 0;
                protocolErrorCount++;
                cout << "Client emitted error on invalid protocol" << endl;
                if (protocolErrorCount > 1) {
                    cout << "FAILURE:  " << protocolErrorCount << " errors emitted for one connection!"
                         << endl;
                }
                break;
            default:
                cout << "FAILURE: " << user << " should not emit error!" << endl;
        }
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

json process_telemetry_data(Map map, json data) {
    json msgJson;

    // Main car's localization Data
    double car_x = data["x"];
    double car_y = data["y"];
    double car_s = data["s"];
    double car_d = data["d"];
    double car_yaw = data["yaw"];
    double car_speed = data["speed"];

    // Previous path data given to the Planner
    auto previous_path_x = data["previous_path_x"];
    auto previous_path_y = data["previous_path_y"];
    // Previous path's end s and d values
    double end_path_s = data["end_path_s"];
    double end_path_d = data["end_path_d"];

    // Sensor Fusion Data, a list of all other cars on the same side of the road.
    auto sensor_fusion = data["sensor_fusion"]; // format is [ id, x, y, vx, vy, s, d]

    vector<double> next_x_vals;
    vector<double> next_y_vals;

    double pos_x;
    double pos_y;
    double angle;
    int path_size = previous_path_x.size();

    for (int i = 0; i < path_size; i++) {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }

    if (path_size == 0) {
        pos_x = car_x;
        pos_y = car_y;
        angle = deg2rad(car_yaw);
    } else {
        pos_x = previous_path_x[path_size - 1];
        pos_y = previous_path_y[path_size - 1];

        double pos_x2 = previous_path_x[path_size - 2];
        double pos_y2 = previous_path_y[path_size - 2];
        angle = atan2(pos_y - pos_y2, pos_x - pos_x2);
    }

    double dist_inc = 0.5;
    for (int i = 0; i < 50 - path_size; i++) {
        next_x_vals.push_back(pos_x + (dist_inc) * cos(angle + (i + 1) * (pi() / 100)));
        next_y_vals.push_back(pos_y + (dist_inc) * sin(angle + (i + 1) * (pi() / 100)));
        pos_x += (dist_inc) * cos(angle + (i + 1) * (pi() / 100));
        pos_y += (dist_inc) * sin(angle + (i + 1) * (pi() / 100));
    }

    // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
    msgJson["next_x"] = next_x_vals;
    msgJson["next_y"] = next_y_vals;

    return msgJson;
}
