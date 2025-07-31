#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <cstdlib>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <signal.h>

#include <kuka_resources/kuka_common.h>

int sock = -1;
bool isConnected = false;

void shutdownHandler(int sig) {
    ROS_INFO("Shutting down KUKA RSI Simulator");
    if (sock >= 0) {
        close(sock);
    }
    ros::shutdown();
}

std::string createRSIXMLRob(const std::vector<double>& act_joint_pos,
                            const std::vector<double>& setpoint_joint_pos,
                            int timeout_count, long ipoc,
                            int n_dof, kuka_rsi_common::RSIConfigType config_type) {
    std::ostringstream oss;
    oss << "<Rob TYPE=\"KUKA\">";
    oss << "<RIst X=\"0.0\" Y=\"0.0\" Z=\"0.0\" A=\"0.0\" B=\"0.0\" C=\"0.0\"/>";
    oss << "<RSol X=\"0.0\" Y=\"0.0\" Z=\"0.0\" A=\"0.0\" B=\"0.0\" C=\"0.0\"/>";

    oss << "<AIPos";
    // TODO: Once changed to use the kinematic types in kuka_common.h, update this section
    if (n_dof == 6) {
        for (size_t i = 0; i < act_joint_pos.size(); ++i) {
            oss << " A" << (i + 1) << "=\"" << act_joint_pos[i] << "\"";
        }
    } else if (n_dof == 7) {
        for (size_t i = 0; i < (act_joint_pos.size() - 1); ++i) {
            oss << " A" << (i + 1) << "=\"" << act_joint_pos[i] << "\"";
        }
    }
    oss << "/>";

    oss << "<ASPos";
    if (n_dof == 6) {
        for (size_t i = 0; i < setpoint_joint_pos.size(); ++i) {
            oss << " A" << (i + 1) << "=\"" << setpoint_joint_pos[i] << "\"";
        }
    } else if (n_dof == 7) {
        for (size_t i = 0; i < (setpoint_joint_pos.size() - 1); ++i) {
            oss << " A" << (i + 1) << "=\"" << setpoint_joint_pos[i] << "\"";
        }
    }
    oss << "/>";

    // TODO: Handle this with direct kinematic type in the future since we could be using a 7DOF robot instead of 6+1 track
    if (n_dof == 7) {
        oss << "<EIPos";
        oss << " E1=\"" << act_joint_pos[6] << "\"/>";
        oss << "<ESPos";
        oss << " E1=\"" << setpoint_joint_pos[6] << "\"/>";
    }

    // TODO: DUAL_MOTOR_EXTRUDER to be added
    if (config_type == kuka_rsi_common::RSIConfigType::SINGLE_MOTOR_EXTRUDER) {
        oss << "<CurCmdID>-1</CurCmdID>";
        oss << "<CurMotSpd>0</CurMotSpd>";
    } else if (config_type == kuka_rsi_common::RSIConfigType::FIBERGUN) {
        oss << "<MainServoSpeed>0</MainServoSpeed>";
        oss << "<BladeCount>-1</BladeCount>";
        oss << "<ResinSprayState>-1</ResinSprayState>";
        oss << "<ChuteAirState>-1</ChuteAirState>";
    }

    oss << "<Delay D=\"" << timeout_count << "\"/>";
    oss << "<IPOC>" << ipoc << "</IPOC>";
    oss << "</Rob>";

    return oss.str();
}

std::pair<std::vector<double>, long> parseRSIXMLSen(const std::string& data, int n_dof) {
    std::vector<double> joint_corrections(n_dof, 0.0);
    long ipoc = 0;

    auto start = data.find("<AK");
    if (start != std::string::npos) {
        auto end = data.find(">", start);
        if (end != std::string::npos) {
            std::string ak_data = data.substr(start, end - start);
            for (size_t i = 0; i < 6; ++i) {
                auto attr_start = ak_data.find("A" + std::to_string(i + 1) + "=\"");
                if (attr_start != std::string::npos) {
                    attr_start += 4; // Skip attribute prefix
                    auto attr_end = ak_data.find("\"", attr_start);
                    if (attr_end != std::string::npos) {
                        joint_corrections[i] = std::stod(ak_data.substr(attr_start, attr_end - attr_start));
                    }
                }
            }
        }
    }

    if (n_dof == 7) {
        auto external_start = data.find("<EK");
        if (external_start != std::string::npos) {
            auto end = data.find(">", external_start);
            if (end != std::string::npos) {
                std::string ek_data = data.substr(external_start, end - external_start);
                auto attr_start = ek_data.find("E1=\"");
                if (attr_start != std::string::npos) {
                    attr_start += 4; // Skip attribute prefix
                    auto attr_end = ek_data.find("\"", attr_start);
                    if (attr_end != std::string::npos) {
                        joint_corrections[6] = std::stod(ek_data.substr(attr_start, attr_end - attr_start));
                    }
                }
            }
        }
    }

    auto ipoc_start = data.find("<IPOC>");
    if (ipoc_start != std::string::npos) {
        ipoc_start += 6; // Skip <IPOC>
        auto ipoc_end = data.find("</IPOC>", ipoc_start);
        if (ipoc_end != std::string::npos) {
            ipoc = std::stol(data.substr(ipoc_start, ipoc_end - ipoc_start));
        }
    }

    return {joint_corrections, ipoc};
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "kuka_rsi_simulation");
    ros::NodeHandle nh("~");

    // Get external track presence from the parameter server which will determine DOF
    bool has_linear_track;
    int n_dof;
    nh.getParam("has_linear_track", has_linear_track);
    if (has_linear_track) {
        n_dof = 7;
    } else {
        n_dof = 6;
    }
    // Get the RSI configuration type (which depends on the end-effector type) from the parameter server
    std::string feedback_type_str;
    nh.getParam("rsi_feedback_type", feedback_type_str);
    // TODO: Change this to a shared enum that both the rsi_hw_interface and the simulator can use
    kuka_rsi_common::RSIConfigType config_type;
    if (feedback_type_str == "single_motor_extruder") {
        config_type = kuka_rsi_common::RSIConfigType::SINGLE_MOTOR_EXTRUDER;
    } else if (feedback_type_str == "dual_motor_extruder") {
        config_type = kuka_rsi_common::RSIConfigType::DUAL_MOTOR_EXTRUDER;
    } else if (feedback_type_str == "fibergun") {
        config_type = kuka_rsi_common::RSIConfigType::FIBERGUN;
    } else {
        // Default or error handling
        config_type = kuka_rsi_common::RSIConfigType::SINGLE_MOTOR_EXTRUDER;
        ROS_WARN_STREAM("Unknown rsi_feedback_type: " << feedback_type_str << ", defaulting to SINGLE_MOTOR_EXTRUDER");
    }

    ros::Publisher rsi_act_pub = nh.advertise<std_msgs::String>("rsi/state", 1);
    ros::Publisher rsi_cmd_pub = nh.advertise<std_msgs::String>("rsi/command", 1);

    std::vector<double> act_joint_pos, cmd_joint_pos, des_joint_correction_absolute;
    if (n_dof == 6) {
        act_joint_pos = {0, -90, 0, 0, 0, 0};
        cmd_joint_pos = act_joint_pos;
        des_joint_correction_absolute.assign(6, 0.0);
    } else if (n_dof == 7) {
        act_joint_pos = {0, -90, 0, 0, 0, 0, 50};     // Example starting position, change as needed
        cmd_joint_pos = act_joint_pos;
        des_joint_correction_absolute.assign(7, 0.0);
    }

    double cycle_time = 0.004;
    int timeout_count = 0;
    long ipoc = 0;

    std::string host;
    int port;
    // TODO: Does this work when multiple simulators are running in one network? How does it resolve the name?
    std::string host_param = nh.resolveName("rsi_hw_iface_ip");
    std::string port_param = nh.resolveName("rsi_hw_iface_port");

    if (!nh.getParam(host_param, host)) {
        ROS_FATAL("Failed to get parameter: %s", host_param.c_str());
        return -1;
    }

    if (!nh.getParam(port_param, port)) {
        ROS_FATAL("Failed to get parameter: %s", port_param.c_str());
        return -1;
    }

    ROS_INFO("Host: %s, Port: %i", host.c_str(), port);

    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        ROS_FATAL("Could not create socket");
        return -1;
    }

    // Set receive timeout
    struct timeval tv;
    tv.tv_sec = 1;  // 1 second timeout
    tv.tv_usec = 0;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(tv));

    sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);
    inet_pton(AF_INET, host.c_str(), &server_addr.sin_addr);

    ros::Rate loop_rate(1.0 / cycle_time);

    signal(SIGINT, shutdownHandler);

    ROS_INFO("Starting KUKA RSI Simulator");

    while (ros::ok()) {
        try {
            std::string msg = createRSIXMLRob(act_joint_pos, cmd_joint_pos, timeout_count, ipoc, n_dof, config_type);
            std_msgs::String act_msg;
            act_msg.data = msg;
            rsi_act_pub.publish(act_msg);

            sendto(sock, msg.c_str(), msg.size(), 0, (struct sockaddr*)&server_addr, sizeof(server_addr));

            char buffer[1024];
            socklen_t addr_len = sizeof(server_addr);
            int recv_len = recvfrom(sock, buffer, sizeof(buffer) - 1, 0, (struct sockaddr*)&server_addr, &addr_len);

            if (recv_len > 0) {
                buffer[recv_len] = '\0';
                std::string recv_msg(buffer);

                std_msgs::String cmd_msg;
                cmd_msg.data = recv_msg;
                rsi_cmd_pub.publish(cmd_msg);

                auto [corrections, ipoc_recv] = parseRSIXMLSen(recv_msg, n_dof);
                des_joint_correction_absolute = corrections;
                act_joint_pos = cmd_joint_pos;
                for (size_t i = 0; i < act_joint_pos.size(); ++i) {
                    act_joint_pos[i] += des_joint_correction_absolute[i];
                }

                ipoc = ipoc_recv + 1;
            } else if (recv_len < 0) {
                if (errno == EWOULDBLOCK || errno == EAGAIN) {
                    throw std::runtime_error("Waiting for connection from robot controller node: socket receive timeout");
                } else {
                    throw std::runtime_error("Waiting for connection from robot controller node: socket receive error");
                }
            }
            if (!isConnected) {
                ROS_INFO("\033[1;32mConnected to robot controller node\033[0m");
                isConnected = true;
            }
        } catch (const std::exception& e) {
            if (isConnected) {
                ROS_WARN("Lost connection to robot controller node");
                isConnected = false;
            }
            ROS_WARN(e.what());
            timeout_count++;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    close(sock);
    return 0;
}