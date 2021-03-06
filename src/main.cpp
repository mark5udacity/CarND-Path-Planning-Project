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
#include "spline.h"

using namespace std;

using json = nlohmann::json;

static const double MAX_SPEED = 49.5;
static const double MAX_SPEED_CHANGE = .224; // About 5 m/s^2 accelleration
static const double MPH_TO_METERS = 2.24;

static const int NUM_LANES = 3; // FYI: Lanes are indexed at 0.
static const double LANE_WIDTH = 4.; // in meters, useful for d part of Frenet coordinates
static const double HALF_LANE_WIDTH = LANE_WIDTH / 2.; // to avoid having to compute /2 everytime.

static const int NUM_POINTS = 50; // Number of points to use in path
static const double TARGET_DISTANCE = 30.; // How far to look ahead with path calc.
static const double SIMULATOR_TIME_STEP = .02; // Num seconds between each point that the simulator
static const int WEBSOCKECT_OK_DISCONNECT_CODE = 1000;
static const string MANUAL_WS_MESSAGE = "42[\"manual\",{}]";

struct SF_CONSTANTS {
    explicit SF_CONSTANTS(){};

    static const int v_x = 3;
    static const int v_y = 4;
    static const int s = 5;
    const int d = 6;
};

static const struct SF_CONSTANTS SENSOR_FUSION_IDX;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s);

void sendMessage(uWS::WebSocket<uWS::SERVER> ws, string msg) { ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT); }

json process_telemetry_data(Map map, json data, int &lane, double &ref_velocity);

pair<vector<double>, vector<double>> generate_trajectory_for_lane(json data,
                                                                  Map map,
                                                                  const int lane,
                                                                  const double ref_velocity);

void determine_lane_and_velocity(json data, int &lane, double &ref_velocity);

int main() {
    uWS::Hub h;
    bool firstTimeConnecting = true;

    Map map;

    // Waypoint map to read from
    string map_file = "../data/highway_map.csv";

    map.load_map(map_file);

    // start in lane 1
    int lane = 1;
    double ref_velocity = 0; //mph

    h.onMessage( [&lane, &map, &ref_velocity] (
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
                    json msgJson = process_telemetry_data(map, j[1], lane, ref_velocity); // j[1] is the data JSON object

                    auto msg = "42[\"control\"," + msgJson.dump() + "]";

                    //this_thread::sleep_for(chrono::milliseconds(1000));
                    sendMessage(ws, msg);
                } else {
                    cout << "Unknown event type (" << event << ") received!!" << "\n";
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
            cout << "Connected for first time!!!" << endl;
            firstTimeConnecting = false;
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

json process_telemetry_data(Map map, json data, int &lane, double &ref_velocity) {
    determine_lane_and_velocity(data, lane, ref_velocity);

    pair<vector<double>, vector<double>> trajectory = generate_trajectory_for_lane(data, map, lane, ref_velocity);

    json msgJson;
    msgJson["next_x"] = trajectory.first;
    msgJson["next_y"] = trajectory.second;

    return msgJson;
}

void determine_lane_and_velocity(json data, int &lane, double &ref_velocity) {
    // Main car's localization Data
    double car_s = data["s"];

    // Previous path data given to the Planner
    auto previous_path_x = data["previous_path_x"];
    auto previous_path_y = data["previous_path_y"];
    // Previous path's end s and d values
    double end_path_s = data["end_path_s"];

    // Sensor Fusion Data, a list of all other cars on the same side of the road.
    auto sensor_fusion = data["sensor_fusion"]; // format is [ id, x, y, vx, vy, s, d]

    int prev_size = previous_path_x.size();

    double last_s = prev_size > 0 ? end_path_s : car_s;

    bool same_lane_clear = true;
    bool left_lane_clear = lane != 0;
    bool right_lane_clear = lane != NUM_LANES - 1;

    for (auto cur_sense : sensor_fusion) {
        float d = cur_sense[SENSOR_FUSION_IDX.d];
        double v_x = cur_sense[SENSOR_FUSION_IDX.v_x];
        double v_y = cur_sense[SENSOR_FUSION_IDX.v_y];
        double check_speed = sqrt(v_x * v_x + v_y * v_y);
        double check_car_s = cur_sense[SENSOR_FUSION_IDX.s];
        check_car_s += (double) prev_size * SIMULATOR_TIME_STEP * check_speed;

        bool in_same_lane = d < (LANE_WIDTH + LANE_WIDTH * lane) && d > LANE_WIDTH * lane;
        if (in_same_lane) {
            bool getting_close = check_car_s > last_s && (check_car_s - last_s) < TARGET_DISTANCE;
            if (getting_close) {
                same_lane_clear = false;
            }
        } else {
            // Possible improvement -- these checks will return true if the car is in lane 0,
            // but the sensed obstacle is in lane 2, thereby preventing the car from going to 1.
            // I would rather implement FSM than fix this issue as the car performs fairly well otherwise.
            bool in_left_lane = d < LANE_WIDTH * lane;
            bool in_right_lane = d > LANE_WIDTH + LANE_WIDTH * lane;

            bool getting_close = check_car_s > last_s - TARGET_DISTANCE / 3
                                   && check_car_s - last_s < TARGET_DISTANCE;
            if (in_left_lane) {
                if (getting_close) {
                    left_lane_clear = false;
                }
            } else {
                if (!in_right_lane) { // I may need to reconsider this logic? But again, would prefer to use FSM
                    cout << "in_right_lane must be true if we are here! For car's lane: " << lane << " and sensor's d: " << d << "\n";
                }

                if (getting_close) {
                    right_lane_clear = false;
                }
            }
        }
    }

    if (same_lane_clear) {
        if (ref_velocity < MAX_SPEED) {
            ref_velocity += MAX_SPEED_CHANGE;
        }
    } else if (left_lane_clear) {
        lane--;
    } else if (right_lane_clear) {
        lane++;
    } else {
        ref_velocity -= MAX_SPEED_CHANGE;
    }
}

pair<vector<double>, vector<double>> generate_trajectory_for_lane(json data,
                                                                  Map map,
                                                                  const int lane,
                                                                  const double ref_velocity) {
    // Main car's localization Data
    double car_x = data["x"];
    double car_y = data["y"];
    double car_s = data["s"];
    double car_yaw = data["yaw"];

    // Previous path data given to the Planner
    auto previous_path_x = data["previous_path_x"];
    auto previous_path_y = data["previous_path_y"];
    // Previous path's end s and d values
    double end_path_s = data["end_path_s"];

    int prev_size = previous_path_x.size();

    double last_s = prev_size > 0 ? end_path_s : car_s;

    vector<double> pts_x;
    vector<double> pts_y;

    // ref x,y,yaw states either we will reference the starting point where car is or the previous path end point
    double ref_x;
    double ref_y;
    double ref_yaw;

    // If we're almost empty on paths, use the car as starting reference
    if (prev_size < 2) {
        ref_x = car_x;
        ref_y = car_y;
        ref_yaw = map.deg2rad(car_yaw);
        double prev_car_x = car_x - cos(car_yaw);
        double prev_car_y = car_y - sin(car_yaw);

        pts_x.push_back(prev_car_x);
        pts_x.push_back(car_x);

        pts_y.push_back(prev_car_y);
        pts_y.push_back(car_y);
    } else {
        ref_x = previous_path_x[prev_size - 1];
        ref_y = previous_path_y[prev_size - 1];

        double ref_x_prev = previous_path_x[prev_size - 2];
        double ref_y_prev = previous_path_y[prev_size - 2];
        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

        pts_x.push_back(ref_x_prev);
        pts_x.push_back(ref_x);

        pts_y.push_back(ref_y_prev);
        pts_y.push_back(ref_y);
    }

    // Add some some extra space for starting reference
    vector<pair<double, double>> wps;
    wps.push_back(map.getXY(last_s + TARGET_DISTANCE    , (HALF_LANE_WIDTH + LANE_WIDTH * lane)));
    wps.push_back(map.getXY(last_s + TARGET_DISTANCE * 2, (HALF_LANE_WIDTH + LANE_WIDTH * lane)));
    wps.push_back(map.getXY(last_s + TARGET_DISTANCE * 3, (HALF_LANE_WIDTH + LANE_WIDTH * lane)));
    for (pair<double, double> wp : wps) {
        pts_x.push_back(wp.first);
        pts_y.push_back(wp.second);
    }

    // Transform to local car coordinates
    for (int i = 0; i < pts_x.size(); ++i) {
        double shift_x = pts_x[i] - ref_x;
        double shift_y = pts_y[i] - ref_y;

        pts_x[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
        pts_y[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
    }

    tk::spline spline;
    spline.set_points(pts_x, pts_y);

    vector<double> next_x_vals;
    vector<double> next_y_vals;

    // Add all previous paths to next
    next_x_vals.insert(end(next_x_vals), begin(previous_path_x), end(previous_path_x));
    next_y_vals.insert(end(next_y_vals), begin(previous_path_y), end(previous_path_y));

    double target_x = TARGET_DISTANCE;
    double target_y = spline(target_x);
    double target_dist = sqrt(target_x * target_x + target_y * target_y);

    double x_add_on = 0;

    int points_to_add = NUM_POINTS - prev_size;
    for (int i = 1; i <= points_to_add; i++) {
        double N = target_dist / (SIMULATOR_TIME_STEP * ref_velocity / MPH_TO_METERS); // converting back to meters/s, not MPH
        double x_point = x_add_on + target_x / N;
        double y_point = spline(x_point);

        x_add_on = x_point;

        double local_x_ref = x_point;
        double local_y_ref = y_point;

        // rotate back to normal after rotating it earlier
        x_point = local_x_ref * cos(ref_yaw) - local_y_ref * sin(ref_yaw);
        y_point = local_x_ref * sin(ref_yaw) + local_y_ref * cos(ref_yaw);


        // Very poor naming from Q&A, x_ref looks a lot like ref_x, was stuck on that for a little!
        x_point += ref_x;
        y_point += ref_y;

        next_x_vals.push_back(x_point);
        next_y_vals.push_back(y_point);
    }

    return make_pair(next_x_vals, next_y_vals);
}
