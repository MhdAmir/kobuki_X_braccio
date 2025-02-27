#include "braccio/main.h"
#include <iostream>
#include <chrono>
#include <thread>

#define L0 71.5
#define L1 125
#define L2 125
#define L3 192 

// 45

std::vector<int> InverseKinematics::computeAngles(double x, double y, double z) {
    double r_compensation = 1.02;
    z += 15;
    double r_hor = std::sqrt(x * x + y * y);
    double r = std::sqrt(r_hor * r_hor + (z - L0) * (z - L0)) * r_compensation;

    double theta_base = (y == 0) ? (x <= 0 ? 180 : 0) : 90 - std::atan(x / y) * 180.0 / M_PI;
    double theta_shoulder, theta_elbow, theta_wrist;
    
    try {
        double alpha1 = std::acos((r - L2) / (L1 + L3));
        theta_shoulder = alpha1 * 180.0 / M_PI;
        double alpha3 = std::asin((std::sin(alpha1) * L3 - std::sin(alpha1) * L1) / L2);
        theta_elbow = (90 - alpha1 * 180.0 / M_PI) + alpha3 * 180.0 / M_PI;
        theta_wrist = (90 - alpha1 * 180.0 / M_PI) - alpha3 * 180.0 / M_PI;
    } catch (...) {
        std::cerr << "Error: Position out of range!" << std::endl;
        return {};
    }

    theta_elbow += 5;
    theta_wrist += 5;

    if (y < 0) {
        theta_shoulder = 180 - theta_shoulder;
        theta_elbow = 180 - theta_elbow;
        theta_wrist = 180 - theta_wrist;
    }

    return {static_cast<int>(std::round(theta_base)), static_cast<int>(std::round(theta_shoulder)), static_cast<int>(std::round(theta_elbow)), static_cast<int>(std::round(theta_wrist))};
}

ArduinoCommunicator::ArduinoCommunicator(const std::string &port, int baudrate) {
    try {
        serialPort.setPort(port);
        serialPort.setBaudrate(baudrate);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
        serialPort.setTimeout(timeout);

        serialPort.open();
        std::cout << "Connected to Arduino at " << port << std::endl;
    } catch (serial::IOException &e) {
        std::cerr << "Error connecting: " << e.what() << std::endl;
    }
}

void ArduinoCommunicator::sendCommand(const std::string &command, double delay) {
    std::this_thread::sleep_for(std::chrono::seconds(static_cast<int>(delay)));
    if (serialPort.isOpen()) {
        serialPort.write(command);
        std::cout << "Sent: " << command << std::endl;
    } else {
        std::cerr << "Arduino not connected!" << std::endl;
    }
}

GetObject::GetObject(int argc, char **argv) : arduino("/dev/ttyACM0", 115200) {
    nh_ = ros::NodeHandle(); // Inisialisasi setelah ros::init
    get_target_position_srv_ = nh_.advertiseService("/get_target_object", &GetObject::get_target_position_, this);
    // init_position_srv_   = nh_.advertiseService("/init_position", &GetObject::init_position_, this);  
}

void GetObject::run() {
    ros::Rate rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
}

bool GetObject::get_target_position_(brac_msgs::getTarget::Request & req, brac_msgs::getTarget::Response & res) {
    std::vector<int> angles = InverseKinematics::computeAngles(req.x, req.z, 0);
    if (angles[0] != -2147483648 && angles[1] != -2147483648 && angles[2] != -2147483648 && angles[3] != -2147483648 ) {
        std::string command = "P " + std::to_string(angles[0]) + " 90 90 90 90 30 50\n";
        arduino.sendCommand(command, 2);    
        command = "P " + std::to_string(angles[0]) + " " + std::to_string(angles[1]) + " " + std::to_string(angles[2]) + " " + std::to_string(angles[3]) + " 90 30 50\n";
        arduino.sendCommand(command, 2);    
        command = "P " + std::to_string(angles[0]) + " " + std::to_string(angles[1]) + " " + std::to_string(angles[2]) + " " + std::to_string(angles[3]) + " 90 78 50\n";
        arduino.sendCommand(command, 2);    
        command = "P 90 87 161 161 90 80 50\n";
        arduino.sendCommand(command, 2);   
        command = "P 90 87 161 161 90 30 50\n";
        arduino.sendCommand(command, 2);
        res.success = true;
        res.error_code = 0;
    }else{
        res.success = false;
        res.error_code = 2;
    }
    
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "object_click_3d");

    GetObject node(argc, argv);
    node.run();
    return 0;
}
