#ifndef KUKA_COMMON_H
#define KUKA_COMMON_H

namespace kuka_rsi_common {

    // Configuration type of the system depending on the end-effector for the feedback 
    enum RSIConfigType {
        SINGLE_MOTOR_EXTRUDER,
        DUAL_MOTOR_EXTRUDER,
        FIBERGUN
    };

    // Kinematic type of the robotic system as defined in the KUKA controller
    // 6DOF: 6-axis robot only
    // 7DOF: 7-axis robot only
    // 6DOF_LINEAR_TRACK: 6-axis robot with a linear track (external axis) for a total of 7 DOF
    // 6DOF_SINGLE_ROTARY_TABLE: 6-axis robot with a single-axis rotary table (external axis) for a total of 7 DOF
    // 7DOF_LINEAR_TRACK: 7-axis robot with a linear track (external axis) for a total of 8 DOF
    enum KinematicType {
        KINEMATIC_6DOF,
        KINEMATIC_7DOF,
        KINEMATIC_6DOF_LINEAR_TRACK,
        KINEMATIC_6DOF_SINGLE_ROTARY_TABLE,
        KINEMATIC_7DOF_LINEAR_TRACK
    };
}

#endif // KUKA_COMMON_H