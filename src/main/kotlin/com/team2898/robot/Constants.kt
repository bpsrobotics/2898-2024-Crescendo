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
import com.team2898.engine.utils.units.*
import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.XboxController
import java.io.File

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
    private val driverController = XboxController(0)
    private val operatorController = Joystick(1)

    object DriveConstants {
        // Driving Parameters - Note that these are not the maximum capable speeds of
        // the robot, rather the allowed maximum speeds
        const val MaxSpeedMetersPerSecond = 4.5
        const val MaxAngularSpeed = 2 * Math.PI // radians per second (2*PI)
        const val DirectionSlewRate = 2.0 // radians per second
        const val MagnitudeSlewRate = 1.8 // percent per second (1 = 100%)
        const val RotationalSlewRate = 10.0 // percent per second (1 = 100%)

        // Chassis configuration (left to right dist of center of the wheels)
        val TrackWidth = Units.inchesToMeters(22.75)

        // Distance between centers of right and left wheels on robot (front to back dist)
        val WheelBase = Units.inchesToMeters(22.75)

        // Distance between front and back wheels on robot: CHANGE TO MATCH WITH ROBOT
        val DriveKinematics = SwerveDriveKinematics(
                Translation2d(-WheelBase / 2, TrackWidth / 2), // Front Left (-,+) 4:4 :1 :1
                Translation2d(-WheelBase / 2, -TrackWidth / 2), // Front Right (+,+) 1:3 :4 :2
                Translation2d(WheelBase / 2, TrackWidth / 2), // Back Left (-,-) 3:1 :2 :4
                Translation2d(WheelBase / 2, -TrackWidth / 2)) //Back Right (+,-) 2:2 :3 :3

        // Angular offsets of the modules relative to the chassis in radians
        const val FrontLeftChassisAngularOffset  = 0.0
        const val FrontRightChassisAngularOffset = 0.0
        const val BackLeftChassisAngularOffset   = 0.0
        const val BackRightChassisAngularOffset  = 0.0

        // YAGSL File Configs
        val DRIVE_CONFIG: File = File(Filesystem.getDeployDirectory(), "/swerve/")

    }

    object ModuleConstants {
        // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
        // This changes the drive speed of the module (a pinion gear with more teeth will result in a
        // robot that drives faster).
//        const val kDrivingMotorPinionTeeth = 14

        // Invert the turning encoder, since the output shaft rotates in the opposite direction of
        // the steering motor in the MAXSwerve Module.
        const val TurningEncoderInverted = false

        // Calculations required for driving motor conversion factors and feed forward
        const val DrivingMotorFreeSpeedRps = NeoMotorConstants.FreeSpeedRpm / 60
        const val WheelDiameterMeters = 0.0979840867
        const val WheelCircumferenceMeters = WheelDiameterMeters * Math.PI

        const val DrivingEncoderPositionFactor = (WheelDiameterMeters * Math.PI / 6.75) // Wheel Diameter * PI * Gear Ratio

        const val DrivingEncoderVelocityFactor = DrivingEncoderPositionFactor / 60.0 // meters per second velocity
        const val TurningEncoderPositionFactor = 2 * Math.PI // radians
        const val TurningEncoderVelocityFactor = 2 * Math.PI / 60.0 // radians per second
        const val TurningEncoderPositionPIDMinInput = 0.0 // radians
        const val TurningEncoderPositionPIDMaxInput = TurningEncoderPositionFactor // radians
//        var DrivingP = 0.1
//        var DrivingI = 0.0
//        var DrivingD = 0.0
////        const val DrivingKs = 0.11937
//        const val DrivingKs = 0.01937
////        const val DrivingKv = 2.6335
//        const val DrivingKv = 0.1335
//        const val DrivingKa = 0.06035
////        const val DrivingKa = 0.46035
        var DrivingP = 0.05
        var DrivingI = 0.0
        var DrivingD = 0.01
        const val DrivingKs = 0.0
        const val DrivingKv = 0.55
        const val DrivingKa = 0.0
        const val DrivingMinOutput = -1.0
        const val DrivingMaxOutput = 1.0
        var TurningP = 0.9
//        var TurningP = 0.75
        var TurningI = 0.0
        var TurningD = 0.0
//        var Ks = 0.06 //0.085
        const val TurningFF = 0.0
        const val TurningMinOutput = 0.5
        const val TurningMaxOutput = 1.0
        val DrivingMotorIdleMode = CANSparkBase.IdleMode.kBrake
        val TurningMotorIdleMode = CANSparkBase.IdleMode.kBrake
        const val DrivingMotorCurrentLimit = 30 // amps
        const val TurningMotorCurrentLimit = 10 // amps
    }

    object OIConstants {
        const val DriverControllerPort = 0
        @Suppress("SpellCheckingInspection")
        const val DriveDeadband = 0.05
        const val SpeedMultiplierMin = 0.4
        const val SpeedMultiplierMax = 1.0
    }

    object AutoConstants {

        const val MaxAccelerationMetersPerSecondSquared = 3.0
        const val MaxAngularSpeedRadiansPerSecond = Math.PI
        const val MaxAngularSpeedRadiansPerSecondSquared = Math.PI
        const val PXController = 1.0
        const val PYController = 1.0
        const val PThetaController = 1.0

        const val TranslationP = 5.0
        const val TranslationI = 0.0
        const val TranslationD = 0.0

        const val RotationP = 0.01
        const val RotationI = 0.0
        const val RotationD = 0.0

        // Constraint for the motion profiled robot angle controller
        val ThetaControllerConstraints = TrapezoidProfile.Constraints(
                MaxAngularSpeedRadiansPerSecond, MaxAngularSpeedRadiansPerSecondSquared)
    }

    object NeoMotorConstants {
        const val FreeSpeedRpm = 5676.0
    }

    object ArmConstants {
        const val CurrentLimit = 40
        const val ArmMaxSpeed = 1.5
        const val Arm_MaxAccel = 1.5
        enum class ArmHeights(val position: Double) {
            GROUND(Arm.LOWER_SOFT_STOP),
            STOWED(0.183),
            AMP(-0.1),
            SHOOTER1(1.0),
            SHOOTER2(1.3),   //base
            SIXPIECE1(1.15),
            SIXPIECE2(1.05)

        }
    }

    object IntakeConstants{
        const val CURRENT_LIMIT = 15
        const val INTAKE_SPEED = 0.75
        const val STOP_BUFFER = 1.0
    }
    object ShooterConstants{
        val FLYWHEEL_CIRCUMFERENCE = 4.inches
        const val INTAKE_SPEED = 1.0
        const val INTAKE_DURATION = 0.5
    }

    object ClimberConstants{
        const val CURRENT_LIMIT = 15
        const val STALL_CURRENT = 20

        const val ClimberMaxSpeed = 1.0
        const val ClimberAcceleration = 1.0
        enum class ClimbHeights(val position: Double) {
            STOWED(0.0), //TODO guess what more real values needed
            REACH(1.0),
            LIFTOFF(0.6);

            fun advance() = when (this) {
                STOWED -> REACH
                REACH -> LIFTOFF
                LIFTOFF -> LIFTOFF
            }
            fun retract() = when (this) {
                STOWED -> STOWED
                REACH -> STOWED
                LIFTOFF -> REACH
            }
        }
    }

    // set to operator/driver's preferences
    object ButtonConstants {
        const val CLIMBER_UP = 2 // very hard to press accidentally
        const val CLIMBER_WAIT_DURATION = 0.5

        const val SHOOT = 4
        const val ARM_UP = 5
        const val ARM_DOWN = 3

        const val ARM_DIRECT_GROUND = 11
        const val ARM_DIRECT_STOWED = 8
        const val ARM_DIRECT_AMP = 7
        const val ARM_DIRECT_SHOOTER1 = 9
        const val ARM_DIRECT_SHOOTER2 = 10
        const val ARM_DIRECT_WAIT_DURATION = 0.05

        const val PRESS_ACTIVATE_DURATION = 0.1
        const val INPUT_BUFFER_DURATION = 0.2
    }
    object VisionConstants{
        const val RED_ALLIANCE_SPEAKER_TAG_ID = 4
        const val BLUE_ALLIANCE_SPEAKER_TAG_ID = 7
    }

}