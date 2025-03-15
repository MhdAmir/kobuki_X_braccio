#include "kobuki_intelligent/intelligent.h"
double degrees(double radians) {
    return radians * 180.0 / M_PI;
}

std::vector<int> compute_inverse_kinematics(double x, double y, double z) {
    double r_compensation = 1.02;
    z = z + 15; 
    double r_hor = sqrt(x * x + y * y);
    double r = sqrt(r_hor * r_hor + (z - L0) * (z - L0)) * r_compensation;

    double theta_base;
    if (y == 0) {
        theta_base = (x <= 0) ? 180 : 0;
    } else {
        theta_base = 90 - degrees(atan(x / y));
    }

    double alpha1, theta_shoulder, alpha3, theta_elbow, theta_wrist;
    try {
        alpha1 = acos((r - L2) / (L1 + L3));
        theta_shoulder = degrees(alpha1);
        alpha3 = asin((sin(alpha1) * L3 - sin(alpha1) * L1) / L2);
        theta_elbow = (90 - degrees(alpha1)) + degrees(alpha3);
        theta_wrist = (90 - degrees(alpha1)) - degrees(alpha3);
    } catch (const std::exception& e) {
        throw std::runtime_error("❌ Error: Position out of reach!");
    }

    if (theta_wrist <= 0) {
        alpha1 = acos((r - L2) / (L1 + L3));
        theta_shoulder = degrees(alpha1 + asin((L3 - L1) / r));
        theta_elbow = (90 - degrees(alpha1));
        theta_wrist = (90 - degrees(alpha1));
    }

    if (z != L0) {
        theta_shoulder += degrees(atan((z - L0) / r));
    }

    theta_elbow += 5;
    theta_wrist += 5;

    if (y < 0) {
        theta_shoulder = 180 - theta_shoulder;  // Exceed 90°
        theta_elbow = 180 - theta_elbow;        // Exceed 90°
        theta_wrist = 180 - theta_wrist;        // Exceed 90°
    }

    std::vector<int> theta_array = {
        static_cast<int>(round(theta_base)),
        static_cast<int>(round(theta_shoulder)),
        static_cast<int>(round(theta_elbow)),
        static_cast<int>(round(theta_wrist))
    };

    return theta_array;
}