// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
@file:Suppress("unused")

package com.team2898.robot

import com.revrobotics.CANSparkBase
import com.team2898.robot.subsystems.Arm
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj2.command.Command


/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 *
 *
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
@Suppress("MemberVisibilityCanBePrivate")
class Constants {
    object DriveConstants {
        // Driving Parameters - Note that these are not the maximum capable speeds of
        // the robot, rather the allowed maximum speeds
        const val kMaxSpeedMetersPerSecond = 4.5
        const val kMaxAngularSpeed = 2 * Math.PI // radians per second (2*PI)
        const val kDirectionSlewRate = 1.2 // radians per second
        const val kMagnitudeSlewRate = 1.8 // percent per second (1 = 100%)
        const val kRotationalSlewRate = 2.0 // percent per second (1 = 100%)

        const val kFrontLeftCANCoderID = 2
        const val kFrontRightCANCoderID = 3
        const val kRearRightCANCoderID = 0
        const val kRearLeftCANCoderID = 1




        // Chassis configuration (left to right dist of center of the wheels)
        val kTrackWidth = Units.inchesToMeters(22.75)

        // Distance between centers of right and left wheels on robot (front to back dist)
        val kWheelBase = Units.inchesToMeters(22.75)

        // Distance between front and back wheels on robot
        val kDriveKinematics = SwerveDriveKinematics(
                Translation2d(kWheelBase / 2, kTrackWidth / 2),
                Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                Translation2d(-kWheelBase / 2, -kTrackWidth / 2))

        // Angular offsets of the modules relative to the chassis in radians
        const val kFrontLeftChassisAngularOffset  = 4.940901
        const val kFrontRightChassisAngularOffset = 0.100158
        const val kBackLeftChassisAngularOffset   = 4.068921
        const val kBackRightChassisAngularOffset  = 2.557114

        //angular

        // SPARK MAX CAN IDs
        const val kFrontLeftDrivingCanId = 3
        const val kRearLeftDrivingCanId = 1
        const val kFrontRightDrivingCanId = 4
        const val kRearRightDrivingCanId = 2
        const val kFrontLeftTurningCanId = 5
        const val kRearLeftTurningCanId = 7
        const val kFrontRightTurningCanId = 8
        const val kRearRightTurningCanId = 6
        const val kGyroReversed = false
        const val kArm_left = 9
        const val kArm_right = 10 //TODO()
        const val kIntakeId = 11
        const val kShooterId = 12
        const val kClimberId = 13
    }

    object ModuleConstants {
        // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
        // This changes the drive speed of the module (a pinion gear with more teeth will result in a
        // robot that drives faster).
//        const val kDrivingMotorPinionTeeth = 14

        // Invert the turning encoder, since the output shaft rotates in the opposite direction of
        // the steering motor in the MAXSwerve Module.
        const val kTurningEncoderInverted = true

        // Calculations required for driving motor conversion factors and feed forward
        const val kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60
        const val kWheelDiameterMeters = 0.10033
        const val kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI

        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
//        const val kDrivingMotorReduction = 45.0 * 22 / (kDrivingMotorPinionTeeth * 15)
        const val kDrivingMotorReduction = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0) //Gear Ratio of MK4I L2
        const val kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters
                / kDrivingMotorReduction)
        const val kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI
                / kDrivingMotorReduction // meters
                )
        const val kDrivingEncoderVelocityFactor = (kWheelDiameterMeters * Math.PI
                / kDrivingMotorReduction) / 60.0 // meters per second
        const val kTurningEncoderPositionFactor = 2 * Math.PI // radians
        const val kTurningEncoderVelocityFactor = 2 * Math.PI / 60.0 // radians per second
        const val kTurningEncoderPositionPIDMinInput = 0.0 // radians
        const val kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor // radians
        const val kDrivingP = 0.04
        const val kDrivingI = 0.0
        const val kDrivingD = 0.0
        const val kDrivingFF = 1 / kDriveWheelFreeSpeedRps
        const val kDrivingMinOutput = -1.0
        const val kDrivingMaxOutput = 1.0
        var kTurningP = 2.0
        var kTurningI = 0.0
        var kTurningD = 0.01
        var Ks = 0.06 //0.085
        const val kTurningFF = 0.0
        const val kTurningMinOutput = -1.0
        const val kTurningMaxOutput = 1.0
        val kDrivingMotorIdleMode = CANSparkBase.IdleMode.kCoast
        val kTurningMotorIdleMode = CANSparkBase.IdleMode.kCoast
        const val kDrivingMotorCurrentLimit = 40 // amps
        const val kTurningMotorCurrentLimit = 20 // amps
    }

    object OIConstants {
        const val kDriverControllerPort = 0
        @Suppress("SpellCheckingInspection")
        const val kDriveDeadband = 0.05
        const val kSpeedMultiplierMin = 0.4
        const val kSpeedMultiplierMax = 1.0
    }

    object AutoConstants {
        var commandMap: HashMap<String, Command> = HashMap<String, Command>()

        const val kMaxAccelerationMetersPerSecondSquared = 3.0
        const val kMaxAngularSpeedRadiansPerSecond = Math.PI
        const val kMaxAngularSpeedRadiansPerSecondSquared = Math.PI
        const val kPXController = 1.0
        const val kPYController = 1.0
        const val kPThetaController = 1.0

        // Constraint for the motion profiled robot angle controller
        val kThetaControllerConstraints = TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared)
    }

    object NeoMotorConstants {
        const val kFreeSpeedRpm = 5676.0
    }

    object ArmConstants {
        const val kArmDigitalInput = 1 //TODO() get number 4 dis
        const val kArmMaxSpeed = 1.0
        const val kArm_MaxAccel = 1.5
        enum class ArmHeights(val position: Double) {
            GROUND(Arm.LOWER_SOFT_STOP),
            STOWED(0.0), //TODO Set these to real values
            AMP(0.1),
            SHOOTER1(0.2),
            SHOOTER2(0.3)

        }
    }

    object IntakeConstants{
    }
    object ShooterConstants{

    }

    object ClimberConstants{
        enum class ClimbHeights(val position: Double) {
            STOWED(0.0), //TODO guess what more real values needed
            REACH(1.0),
            LIFTOFF(0.6)
        }
    }



}