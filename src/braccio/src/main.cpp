#include "braccio/main.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <mutex>
#include <string>

#define L0 71.5
#define L1 125
#define L2 125
#define L3 192

#define kHalfFrameWidth 960
#define kHalfFrameHeight 540

// Command rate limiting
const int MINIMUM_COMMAND_INTERVAL_MS = 10; // Minimum time between commands
std::chrono::steady_clock::time_point lastCommandTime;
std::mutex commandMutex;

ArduinoCommunicator::ArduinoCommunicator(const std::string &port, int baudrate)
{
    try
    {
        serialPort.setPort(port);
        serialPort.setBaudrate(baudrate);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
        serialPort.setTimeout(timeout);

        serialPort.open();
        std::cout << "Connected to Arduino at " << port << std::endl;

        // Clear any pending data
        if (serialPort.available())
            serialPort.read(serialPort.available());

        // Wait for Arduino to initialize
        std::this_thread::sleep_for(std::chrono::seconds(2));

        // Send initial "Power On" command
        sendCommand("1\n");
    }
    catch (serial::IOException &e)
    {
        std::cerr << "Error connecting: " << e.what() << std::endl;
    }

    lastCommandTime = std::chrono::steady_clock::now();
}

std::string ArduinoCommunicator::readResponse(int timeout_ms)
{
    std::string response;
    auto start_time = std::chrono::steady_clock::now();

    while (std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now() - start_time)
               .count() < timeout_ms)
    {
        if (serialPort.available())
        {
            std::string data = serialPort.readline();
            response += data;

            if (data == "OK\r\n" || data.find("E") == 0)
            {
                break; // Complete response received
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    return response;
}

bool ArduinoCommunicator::isIdle()
{
    std::lock_guard<std::mutex> lock(commandMutex);

    if (!serialPort.isOpen())
    {
        return false;
    }

    serialPort.write("S\n");
    std::string response = readResponse(500);

    return response.find("STATUS:IDLE") != std::string::npos;
}

bool ArduinoCommunicator::sendCommand(const std::string &command)
{
    std::lock_guard<std::mutex> lock(commandMutex);

    if (!serialPort.isOpen())
    {
        std::cerr << "Error: Arduino not connected!" << std::endl;
        return false;
    }

    // Rate limiting
    // auto now = std::chrono::steady_clock::now();
    // auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastCommandTime).count();

    // if (elapsed < MINIMUM_COMMAND_INTERVAL_MS)
    // {
    //     std::this_thread::sleep_for(std::chrono::milliseconds(MINIMUM_COMMAND_INTERVAL_MS - elapsed));
    // }

    // Send command
    serialPort.write(command);
    std::cout << "Sent: " << command;

    // Wait for response with timeout
    std::string response = readResponse(1000);
    std::cout << "Response: " << response << std::endl;

    lastCommandTime = std::chrono::steady_clock::now();
    return response.find("OK") != std::string::npos;
}

GetObject::GetObject(int argc, char **argv) : arduino("/dev/ttyACM0", 115200)
{
    nh_ = ros::NodeHandle();

    // Initialize PID controllers
    initTrackingParameters();

    // Setup ROS communication
    // get_target_position_srv_ = nh_.advertiseService("/get_target_object", &GetObject::get_target_position_, this);
    sub_ = nh_.subscribe("/realsense", 0, &GetObject::realsenseCallback, this);

    // Send home position command
    std::this_thread::sleep_for(std::chrono::seconds(2)); // Give Arduino time to initialize
    std::string command = "H\n";                          // Use the home command instead of explicit position
    arduino.sendCommand(command);
}

void GetObject::initTrackingParameters()
{
    // Initialize flags
    flag_.tracking_object = false;

    // Initialize PID for pan control
    // arm_pan_.pid.Init();
    // arm_pan_.pid.KP = 0.005;  // Reduced from 0.0287
    // arm_pan_.pid.KI = 0.006;  // Reduced from 0.0166
    // arm_pan_.pid.KD = 0.007; // Reduced from 0.0066
    arm_pan_.degree = 90;    // Initial position
    arm_pan_.min_degree = 0;
    arm_pan_.max_degree = 180;

    // Initialize PID for tilt control
    // arm_tilt_.pid.Init();
    // arm_tilt_.pid.KP = 0.005;  // Reduced from 0.0287
    // arm_tilt_.pid.KI = 0.006;  // Reduced from 0.0166
    // arm_tilt_.pid.KD = 0.007; // Reduced from 0.0066
    arm_tilt_.degree = 90;    // Initial position
    arm_tilt_.min_degree = 0;
    arm_tilt_.max_degree = 100;
}

void GetObject::run()
{
    ros::Rate rate(100); // 10Hz control loop

    // Tracking update variables
    ros::Time lastTrackingUpdate = ros::Time::now();
    const double trackingInterval = 0.002; // Update tracking at 5Hz (200ms)

    while (ros::ok())
    {
        ros::spinOnce();

        // Only update tracking at the specified interval
        if (flag_.tracking_object
            &&
            (ros::Time::now() - lastTrackingUpdate).toSec() >= trackingInterval)
        {
            updateTracking();
            // Only send command if Arduino is ready (not moving)
            if (arduino.isIdle())
            {
                // Apply tracking corrections
                lastTrackingUpdate = ros::Time::now();
            }
        }

        rate.sleep();
    }
}

void GetObject::updateTracking()
{
    // Use object_[0] for tracking (assuming it's the most recently updated)
    if (!flag_.tracking_object)
        return;

    // int pan_correction = arm_pan_.pid.Compute(object_[0].error.x, kHalfFrameWidth);
    // int tilt_correction = arm_tilt_.pid.Compute(object_[0].error.y, kHalfFrameHeight);

    // Apply corrections with dampening
    // arm_pan_.Set(arm_pan_.degree + pan_correction);
    // arm_tilt_.Set(arm_tilt_.degree + tilt_correction);

    // Enforce limits
    if (arm_pan_.degree < arm_pan_.min_degree)
        arm_pan_.Set(arm_pan_.min_degree);
    else if (arm_pan_.degree > arm_pan_.max_degree)
        arm_pan_.Set(arm_pan_.max_degree);

    if (arm_tilt_.degree < arm_tilt_.min_degree)
        arm_tilt_.Set(arm_tilt_.min_degree);
    else if (arm_tilt_.degree > arm_tilt_.max_degree)
        arm_tilt_.Set(arm_tilt_.max_degree);

    // printf("pan_corr >> %d\n", pan_correction);
    // printf("tilt_corr >> %d\n\n", tilt_correction);

    // Only send command if correction is significant
    // if (abs(pan_correction) > 1 || abs(tilt_correction) > 1)
    // {
    //     std::string command = "P " + std::to_string(arm_pan_.degree) +
    //                           " 140 120 "+std::to_string(arm_tilt_.degree)+" 90 78 250\n";
    //     arduino.sendCommand(command);
    // }
}

void GetObject::realsenseCallback(const custom_msgs::Realsense::ConstPtr &msg)
{
    // Check if there are valid objects in the message
    bool exist = msg->yolo.size() > 0 &&
                 !(msg->yolo[0].x == 0 && msg->yolo[0].y == 0);

    if (exist)
    {
        const auto &obj0 = msg->yolo[0];
        object_[0].name = obj0.name.c_str();
        object_[0].frame.x = obj0.x - kHalfFrameWidth;
        object_[0].frame.y = obj0.y - kHalfFrameHeight;
        object_[0].distance = obj0.distance;

        // Store error for PID controller
        object_[0].error.x = object_[0].frame.x;
        object_[0].error.y = object_[0].frame.y;

        // Only set tracking flag if not already tracking
        if (!flag_.tracking_object)
        {
            flag_.tracking_object = true;
            arm_pan_.Unlock();
            arm_tilt_.Unlock();
        }
    }
    else if (flag_.tracking_object)
    {
        // If no object detected for several frames, stop tracking
        static int noObjectCount = 0;
        if (++noObjectCount > 15)
        { // About 1.5 seconds at 10Hz
            flag_.tracking_object = false;
            noObjectCount = 0;
        }
    }

    // Handle second object if present
    if (msg->yolo.size() > 41)
    {
        const auto &obj41 = msg->yolo[41];
        object_[1].name = obj41.name.c_str();
        object_[1].frame.x = obj41.x;
        object_[1].frame.y = obj41.y;
        object_[1].distance = obj41.distance;
    }
}

void GetObject::TrackObject(int i)
{
    // Set tracking flag and unlock servos
    flag_.tracking_object = true;
    arm_pan_.Unlock();
    arm_tilt_.Unlock();

    // Update error values for tracking
    object_[i].error.x = object_[i].frame.x;
    object_[i].error.y = object_[i].frame.y;
}

// bool GetObject::get_target_position_(custom_msgs::getTarget::Request &req, custom_msgs::getTarget::Response &res)
// {
//     // Disable tracking while moving to target position
//     flag_.tracking_object = false;

//     // Wait a moment to ensure tracking stops
//     std::this_thread::sleep_for(std::chrono::milliseconds(300));

//     // Calculate inverse kinematics
//     std::vector<int> angles = InverseKinematics::computeAngles(req.x, req.z, 0);

//     // Check if angles are valid
//     if (angles.size() == 4 && angles[0] != -29 && angles[1] != -29 &&
//         angles[2] != -29 && angles[3] != -29)
//     {
//         // Move in stages with checks for success
//         bool success = true;

//         // Stage 1: Move base to target position
//         std::string command = "P " + std::to_string(angles[0]) + " 90 90 90 90 30 50\n";
//         success &= arduino.sendCommand(command);

//         // Wait for movement to complete
//         std::this_thread::sleep_for(std::chrono::milliseconds(500));

//         // Stage 2: Move to target with arm extended
//         if (success)
//         {
//             command = "P " + std::to_string(angles[0]) + " " +
//                       std::to_string(angles[1]) + " " +
//                       std::to_string(angles[2]) + " " +
//                       std::to_string(angles[3]) + " 90 30 50\n";
//             success &= arduino.sendCommand(command);
//             std::this_thread::sleep_for(std::chrono::milliseconds(500));
//         }

//         // Stage 3: Close gripper
//         if (success)
//         {
//             command = "P " + std::to_string(angles[0]) + " " +
//                       std::to_string(angles[1]) + " " +
//                       std::to_string(angles[2]) + " " +
//                       std::to_string(angles[3]) + " 90 78 50\n";
//             success &= arduino.sendCommand(command);
//             std::this_thread::sleep_for(std::chrono::milliseconds(500));
//         }

//         // Stage 4: Move to safe position
//         if (success)
//         {
//             command = "P 90 87 161 161 90 80 50\n";
//             success &= arduino.sendCommand(command);
//             std::this_thread::sleep_for(std::chrono::milliseconds(500));
//         }

//         // Stage 5: Open gripper
//         if (success)
//         {
//             command = "P 90 87 161 161 90 30 50\n";
//             success &= arduino.sendCommand(command);
//         }

//         res.success = success;
//         res.error_code = success ? 0 : 1;
//     }
//     else
//     {
//         // Invalid position
//         res.success = false;
//         res.error_code = 2;
//     }

//     return true;
// }

std::vector<int> InverseKinematics::computeAngles(double x, double y, double z)
{
    // Added boundary check before calculations
    double maxReach = L1 + L2 + L3 - 20; // Subtract safety margin
    double r_hor = std::sqrt(x * x + y * y);
    double r_tot = std::sqrt(r_hor * r_hor + (z - L0) * (z - L0));

    // Check if position is within reach
    if (r_tot > maxReach)
    {
        std::cerr << "Position out of range: " << r_tot << " > " << maxReach << std::endl;
        return {-29, -29, -29, -29}; // Error code
    }

    double r_compensation = 1.02;
    z += 15;
    r_hor = std::sqrt(x * x + y * y);
    double r = std::sqrt(r_hor * r_hor + (z - L0) * (z - L0)) * r_compensation;

    // Calculate base angle (prevent division by zero)
    double theta_base;
    if (std::abs(y) < 0.001)
    {
        theta_base = (x <= 0) ? 180.0 : 0.0;
    }
    else
    {
        theta_base = 90 - std::atan(x / y) * 180.0 / M_PI;
    }

    double theta_shoulder, theta_elbow, theta_wrist;

    try
    {
        // Using law of cosines for angle calculations
        double cos_alpha1 = (r * r + L1 * L1 - (L2 + L3) * (L2 + L3)) / (2 * r * L1);

        // Boundary check for acos
        if (cos_alpha1 < -1.0 || cos_alpha1 > 1.0)
        {
            std::cerr << "Cosine value out of range: " << cos_alpha1 << std::endl;
            return {-29, -29, -29, -29};
        }

        double alpha1 = std::acos(cos_alpha1);
        theta_shoulder = alpha1 * 180.0 / M_PI;

        double cos_alpha3 = ((L1 + L3) * (L1 + L3) + L2 * L2 - r * r) / (2 * (L1 + L3) * L2);

        // Boundary check for acos
        if (cos_alpha3 < -1.0 || cos_alpha3 > 1.0)
        {
            std::cerr << "Cosine value out of range: " << cos_alpha3 << std::endl;
            return {-29, -29, -29, -29};
        }

        double alpha3 = std::acos(cos_alpha3);
        theta_elbow = (90 - alpha1 * 180.0 / M_PI) + alpha3 * 180.0 / M_PI;
        theta_wrist = (90 - alpha1 * 180.0 / M_PI) - alpha3 * 180.0 / M_PI;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Exception in IK: " << e.what() << std::endl;
        return {-29, -29, -29, -29};
    }

    theta_elbow += 5;
    theta_wrist += 5;

    // Handle negative Y values
    if (y < 0)
    {
        theta_shoulder = 180 - theta_shoulder;
        theta_elbow = 180 - theta_elbow;
        theta_wrist = 180 - theta_wrist;
    }

    // Boundary checks for servo limits
    if (theta_base < 0 || theta_base > 180 ||
        theta_shoulder < 0 || theta_shoulder > 180 ||
        theta_elbow < 0 || theta_elbow > 180 ||
        theta_wrist < 0 || theta_wrist > 180)
    {
        return {-29, -29, -29, -29};
    }

    return {
        static_cast<int>(std::round(theta_base)),
        static_cast<int>(std::round(theta_shoulder)),
        static_cast<int>(std::round(theta_elbow)),
        static_cast<int>(std::round(theta_wrist))};
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_click_3d");

    try
    {
        GetObject node(argc, argv);
        node.run();
    }
    catch (const std::exception &e)
    {
        ROS_ERROR("Exception in main: %s", e.what());
        return 1;
    }

    return 0;
}