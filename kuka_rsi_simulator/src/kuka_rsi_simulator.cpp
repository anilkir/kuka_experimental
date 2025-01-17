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
                            int timeout_count, long ipoc) {
    std::ostringstream oss;
    oss << "<Rob TYPE=\"KUKA\">";
    oss << "<RIst X=\"0.0\" Y=\"0.0\" Z=\"0.0\" A=\"0.0\" B=\"0.0\" C=\"0.0\"/>";
    oss << "<RSol X=\"0.0\" Y=\"0.0\" Z=\"0.0\" A=\"0.0\" B=\"0.0\" C=\"0.0\"/>";

    oss << "<AIPos";
    for (size_t i = 0; i < act_joint_pos.size(); ++i) {
        oss << " A" << (i + 1) << "=\"" << act_joint_pos[i] << "\"";
    }
    oss << "/>";

    oss << "<ASPos";
    for (size_t i = 0; i < setpoint_joint_pos.size(); ++i) {
        oss << " A" << (i + 1) << "=\"" << setpoint_joint_pos[i] << "\"";
    }
    oss << "/>";

    oss << "<Delay D=\"" << timeout_count << "\"/>";
    oss << "<IPOC>" << ipoc << "</IPOC>";
    oss << "</Rob>";

    return oss.str();
}

std::pair<std::vector<double>, long> parseRSIXMLSen(const std::string& data) {
    std::vector<double> joint_corrections(6, 0.0);
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

    ros::Publisher rsi_act_pub = nh.advertise<std_msgs::String>("kuka_rsi_simulation/rsi/state", 1);
    ros::Publisher rsi_cmd_pub = nh.advertise<std_msgs::String>("kuka_rsi_simulation/rsi/command", 1);

    double cycle_time = 0.004;
    std::vector<double> act_joint_pos = {0, -90, 0, 0, 0, 0};
    std::vector<double> cmd_joint_pos = act_joint_pos;
    std::vector<double> des_joint_correction_absolute(6, 0.0);
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
            std::string msg = createRSIXMLRob(act_joint_pos, cmd_joint_pos, timeout_count, ipoc);
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

                auto [corrections, ipoc_recv] = parseRSIXMLSen(recv_msg);
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