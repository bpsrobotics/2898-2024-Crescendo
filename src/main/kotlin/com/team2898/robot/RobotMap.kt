package com.team2898.robot

object RobotMap {
    // SPARK MAX CAN IDs
    const val FrontLeftDrivingCanId = 8
    const val RearLeftDrivingCanId = 6
    const val FrontRightDrivingCanId = 2
    const val RearRightDrivingCanId = 4

    const val FrontLeftTurningCanId = 7
    const val RearLeftTurningCanId = 5
    const val FrontRightTurningCanId = 1
    const val RearRightTurningCanId = 3

    const val Arm_left = 10
    const val Arm_right = 9
    const val IntakeId = 12
    const val ShooterTopId = 13
    const val ShooterBottomId = 11
    const val ClimbPrimaryId = 14
    const val ClimbSecondaryId = 15

    // CANcoder IDs
    const val FrontLeftCANCoderID = 44
    const val FrontRightCANCoderID = 21
    const val RearRightCANCoderID = 22
    const val RearLeftCANCoderID = 33

    //DIO IDs
    const val ArmDigitalInput = 1
    const val IntakeBeamBreak = 2

}