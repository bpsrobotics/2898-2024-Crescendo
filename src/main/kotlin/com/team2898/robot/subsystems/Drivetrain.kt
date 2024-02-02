// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package com.team2898.robot.subsystems



import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.util.HolonomicPathFollowerConfig
import com.pathplanner.lib.util.PIDConstants
import com.pathplanner.lib.util.ReplanningConfig
import com.team2898.engine.utils.SwerveUtils
import com.team2898.robot.Constants
import com.team2898.robot.Constants.DriveConstants
import com.team2898.robot.Constants.DriveConstants.DriveKinematics
import com.team2898.robot.Constants.DriveConstants.MaxAngularSpeed
import com.team2898.robot.Constants.DriveConstants.MaxSpeedMetersPerSecond
import com.team2898.robot.RobotMap.FrontLeftCANCoderID
import com.team2898.robot.RobotMap.FrontLeftDrivingCanId
import com.team2898.robot.RobotMap.FrontLeftTurningCanId
import com.team2898.robot.RobotMap.FrontRightCANCoderID
import com.team2898.robot.RobotMap.FrontRightDrivingCanId
import com.team2898.robot.RobotMap.FrontRightTurningCanId
import com.team2898.robot.RobotMap.RearLeftCANCoderID
import com.team2898.robot.RobotMap.RearLeftDrivingCanId
import com.team2898.robot.RobotMap.RearLeftTurningCanId
import com.team2898.robot.RobotMap.RearRightCANCoderID
import com.team2898.robot.RobotMap.RearRightDrivingCanId
import com.team2898.robot.RobotMap.RearRightTurningCanId
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveDriveOdometry
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.util.WPIUtilJNI
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import java.util.function.BooleanSupplier


object Drivetrain
    : SubsystemBase() {

    // Create MAXSwerveModules
    val frontLeft: SwerveModule = SwerveModule(
            FrontLeftDrivingCanId,
            FrontLeftTurningCanId,
            DriveConstants.FrontLeftChassisAngularOffset,
            FrontLeftCANCoderID,
        "FrontLeft",
        true
    )
    val frontRight: SwerveModule = SwerveModule(
            FrontRightDrivingCanId,
            FrontRightTurningCanId,
            DriveConstants.FrontRightChassisAngularOffset,
            FrontRightCANCoderID,
        "FrontRight",
        true
    )
    val rearLeft: SwerveModule = SwerveModule(
            RearLeftDrivingCanId,
            RearLeftTurningCanId,
            DriveConstants.BackLeftChassisAngularOffset,
            RearLeftCANCoderID,
        "RearLeft",
            true
    )
    val rearRight: SwerveModule = SwerveModule(
            RearRightDrivingCanId,
            RearRightTurningCanId,
            DriveConstants.BackRightChassisAngularOffset,
            RearRightCANCoderID,
        "RearRight",
        true
    )

    val states = arrayOf(frontLeft.state, frontRight.state, rearLeft.state, rearRight.state)
    var publisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("MyStates", SwerveModuleState.struct).publish()

    val modules = arrayOf(frontLeft, frontRight, rearLeft, rearRight)
    init {
//        SmartDashboard.putNumber("TurningKs", Constants.ModuleConstants.Ks)
        SmartDashboard.putNumber("TurningKP", Constants.ModuleConstants.TurningP)
        SmartDashboard.putNumber("TurningKI", Constants.ModuleConstants.TurningI)
        SmartDashboard.putNumber("TurningKD", Constants.ModuleConstants.TurningD)

        SmartDashboard.putNumber("DrivingKP", Constants.ModuleConstants.DrivingP)
        SmartDashboard.putNumber("DrivingKI", Constants.ModuleConstants.DrivingI)
        SmartDashboard.putNumber("DrivingKD", Constants.ModuleConstants.DrivingD)

        configureAuto()
    }


    // Slew rate filter variables for controlling lateral acceleration
    private var currentRotation = 0.0
    private var currentTranslationDir = 0.0
    private var currentTranslationMag = 0.0
    private val magLimiter = SlewRateLimiter(DriveConstants.MagnitudeSlewRate)
    private val rotLimiter = SlewRateLimiter(DriveConstants.RotationalSlewRate)
    private var prevTime = WPIUtilJNI.now() * 1e-6

    // Odometry class for tracking robot pose
    var m_odometry = SwerveDriveOdometry(
            DriveConstants.DriveKinematics,
            Rotation2d.fromDegrees(NavX.getAngle()), arrayOf(
            frontLeft.position,
            frontRight.position,
            rearLeft.position,
            rearRight.position
    ))

    override fun periodic() {
        publisher.set(states)
        // Update the odometry in the periodic block

//        Constants.ModuleConstants.Ks = SmartDashboard.getNumber("TurningKs", Constants.ModuleConstants.Ks)
        Constants.ModuleConstants.TurningP = SmartDashboard.getNumber("TurningKP", Constants.ModuleConstants.TurningP)
        Constants.ModuleConstants.TurningI = SmartDashboard.getNumber("TurningKI", Constants.ModuleConstants.TurningI)
        Constants.ModuleConstants.TurningD = SmartDashboard.getNumber("TurningKD", Constants.ModuleConstants.TurningD)

        Constants.ModuleConstants.DrivingP = SmartDashboard.getNumber("DrivingKP", Constants.ModuleConstants.DrivingP)
        Constants.ModuleConstants.DrivingI = SmartDashboard.getNumber("DrivingKI", Constants.ModuleConstants.DrivingI)
        Constants.ModuleConstants.DrivingD = SmartDashboard.getNumber("DrivingKD", Constants.ModuleConstants.DrivingD)



        for (module in modules){
            module.update()
        }


    }

    /** Current estimated pose of the robot.*/
    val pose: Pose2d
        get() = Odometry.SwerveOdometry.poseMeters




    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    fun resetOdometry(pose: Pose2d?) {
        Odometry.resetOdometry(pose)
    }



    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     * field.
     * @param rateLimit     Whether to enable rate limiting for smoother control.
     * @param secondOrder Whether to apply second order kinematics
     */
    fun drive(xSpeed: Double, ySpeed: Double, rot: Double, fieldRelative: Boolean, rateLimit: Boolean, secondOrder: Boolean) {
        val xSpeedCommanded: Double
        val ySpeedCommanded: Double
        if (rateLimit) {
            // Convert XY to polar for rate limiting
            val inputTranslationDir = Math.atan2(ySpeed, xSpeed)
            val inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2.0) + Math.pow(ySpeed, 2.0))

            // Calculate the direction slew rate based on an estimate of the lateral acceleration
            val directionSlewRate: Double
            if (currentTranslationMag != 0.0) {
                directionSlewRate = Math.abs(DriveConstants.DirectionSlewRate / currentTranslationMag)
            } else {
                directionSlewRate = 500.0 //some high number that means the slew rate is effectively instantaneous
            }
            val currentTime = WPIUtilJNI.now() * 1e-6
            val elapsedTime = currentTime - prevTime
            val angleDif: Double = SwerveUtils.AngleDifference(inputTranslationDir, currentTranslationDir)
            if (angleDif < 0.45 * Math.PI) {
                currentTranslationDir = SwerveUtils.StepTowardsCircular(currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime)
                currentTranslationMag = magLimiter.calculate(inputTranslationMag)
            } else if (angleDif > 0.85 * Math.PI) {
                if (currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
                    // keep currentTranslationDir unchanged
                    currentTranslationMag = magLimiter.calculate(0.0)
                } else {
                    currentTranslationDir = SwerveUtils.WrapAngle(currentTranslationDir + Math.PI)
                    currentTranslationMag = magLimiter.calculate(inputTranslationMag)
                }
            } else {
                currentTranslationDir = SwerveUtils.StepTowardsCircular(currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime)
                currentTranslationMag = magLimiter.calculate(0.0)
            }
            prevTime = currentTime
            xSpeedCommanded = currentTranslationMag * Math.cos(currentTranslationDir)
            ySpeedCommanded = currentTranslationMag * Math.sin(currentTranslationDir)
            currentRotation = rotLimiter.calculate(rot)
        } else {
            xSpeedCommanded = xSpeed
            ySpeedCommanded = ySpeed
            currentRotation = rot
        }

        // Convert the commanded speeds into the correct units for the drivetrain

        val xSpeedDelivered: Double = xSpeedCommanded * DriveConstants.MaxSpeedMetersPerSecond
        val ySpeedDelivered: Double = ySpeedCommanded * DriveConstants.MaxSpeedMetersPerSecond
        val rotDelivered: Double = currentRotation * DriveConstants.MaxAngularSpeed

        val speed = if (fieldRelative) ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(NavX.getAngle())) else ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered)
        val secondOrderSpeeds = ChassisSpeeds.discretize(speed, 0.01)
        val swerveModuleStates = DriveConstants.DriveKinematics.toSwerveModuleStates(if (secondOrder) secondOrderSpeeds else speed)
        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, DriveConstants.MaxSpeedMetersPerSecond)
        frontLeft.setDesiredState(swerveModuleStates.get(0))
        frontRight.setDesiredState(swerveModuleStates.get(1))
        rearLeft.setDesiredState(swerveModuleStates.get(2))
        rearRight.setDesiredState(swerveModuleStates.get(3))
//        chassisDrive(if (secondOrder) secondOrderSpeeds else speed)

    }

    val chassisDrive = {
            speeds: ChassisSpeeds ->
        val wantedStates = DriveKinematics.toSwerveModuleStates(speeds)
        setModuleStates(wantedStates)
    }



    /**
     * Sets the wheels into an X formation to prevent movement.
     */
    fun lock() {
        frontLeft.setDesiredState(SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)))
        frontRight.setDesiredState(SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)))
        rearLeft.setDesiredState(SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)))
        rearRight.setDesiredState(SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)))
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    fun setModuleStates(desiredStates: Array<SwerveModuleState>) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates, DriveConstants.MaxSpeedMetersPerSecond)
        frontLeft.setDesiredState(desiredStates[0])
        frontRight.setDesiredState(desiredStates[1])
        rearLeft.setDesiredState(desiredStates[2])
        rearRight.setDesiredState(desiredStates[3])
    }

    /** Resets the drive encoders to currently read a position of 0.  */
    fun resetEncoders() {
        frontLeft.resetEncoders()
        rearLeft.resetEncoders()
        frontRight.resetEncoders()
        rearRight.resetEncoders()
    }


    /** Zeroes the heading of the robot.  */
    fun zeroHeading() {
        NavX.reset()
    }
    /** The robot's heading in degrees, from -180 to 180 */
    val heading: Double
        get() = Rotation2d.fromDegrees(NavX.getAngle()).degrees
    /** The turn rate of the robot, in degrees per second */
    val turnRate: Double
        get() = NavX.navx.rate * if (DriveConstants.GyroReversed) -1.0 else 1.0

    val stateConsumer = { x: Array<SwerveModuleState> -> arrayOf(frontLeft.state, frontRight.state, rearLeft.state, rearRight.state) }

    fun configureAuto() {
        AutoBuilder.configureHolonomic(
            Odometry::supplyPose,  // Robot pose supplier
            Odometry::resetOdometry,  // Method to reset odometry (will be called if your auto has a starting pose)
            Odometry::chassisSpeeds,  // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            chassisDrive,  // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                PIDConstants(Constants.ModuleConstants.DrivingP, Constants.ModuleConstants.DrivingI, Constants.ModuleConstants.DrivingD),  // Translation PID constants
                PIDConstants(Constants.ModuleConstants.TurningP, Constants.ModuleConstants.TurningI, Constants.ModuleConstants.TurningD),  // Rotation PID constants
                MaxSpeedMetersPerSecond,  // Max module speed, in m/s
                0.7112,  // Drive base radius in meters. Distance from robot center to furthest module.
                ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            BooleanSupplier {

                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                val alliance = DriverStation.getAlliance()
                if (alliance.isPresent) {
                    alliance.get() == Alliance.Red
                }
                false
            },
            this // Reference to this subsystem to set requirements
        )

    }

}