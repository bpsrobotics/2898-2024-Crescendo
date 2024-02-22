package com.team2898.robot.commands

//The commands for both the driver and operator
//import com.com.engine.utils.`M/s`
//import com.team2898.robot.Constants.ArmHeights.*
//import com.team2898.robot.Field
//import com.team2898.robot.OI
//import com.team2898.robot.subsystems.Arm

import com.team2898.engine.utils.Sugar.degreesToRadians
import com.team2898.engine.utils.Sugar.eqEpsilon
import com.team2898.engine.utils.TurningPID
import com.team2898.engine.utils.odometry.Vision
import com.team2898.robot.Constants
import com.team2898.robot.OI
import com.team2898.robot.subsystems.*
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import org.photonvision.PhotonUtils
import kotlin.math.*


enum class DriveMode {
    Normal,
    Defense
}
/**
    Called when the Tele-Operated stage of the game begins.
 */
class TeleOp : Command() {
    init {
        addRequirements(Drivetrain)
    }
    // Called when the command is started.
    override fun initialize() {
        //SmartDashboard.putNumber("goal", PI)
        Drivetrain.zeroHeading()
        breakTimer.start()


    }

    var angle = 0.0

    val PID = TurningPID(3.5,0.05)
    val kS = 0.1
    val breakTimer = Timer()
    var breakTimerGoal = 0.0
    var drivemode = DriveMode.Normal
    var resetGyroTimer = Timer()
    private val vision = Vision("Camera_Module_v1")
    var alignMode = false
    var xDist = 0.0
    var yDist = 0.0
    // Called every time the scheduler runs while the command is scheduled.
    fun turnSpeedNormal():Double {
        return -OI.turnX
    }
    fun turnSpeedDefense():Double {
        angle += OI.turnX.pow(2).degreesToRadians() * -5 * OI.turnX.sign
        SmartDashboard.putNumber("angle", angle)

        PID.setPoint = angle
        var turnSpeed = PID.turnspeedOutputNoNormalize(-NavX.totalRotation.degreesToRadians())
        if (!turnSpeed.eqEpsilon(0, 0.04)) turnSpeed += kS * turnSpeed.sign
        return turnSpeed
    }
    fun turnSpeedFieldOriented(): Double {
        if((abs(OI.turnY)+abs(OI.turnX)).eqEpsilon(0,0.2)) return 0.0
        angle = atan2(OI.turnY,OI.turnX)
        SmartDashboard.putNumber("angle", angle)

        PID.setPoint = angle
        var turnSpeed = PID.turnspeedOutputNoNormalize(NavX.getInvertedAngle().degreesToRadians())
        if (!turnSpeed.eqEpsilon(0, 0.04)) turnSpeed += kS * turnSpeed.sign
        return turnSpeed
    }
    fun getTurnSpeed():Double {
        return when {
            OI.leftTrigger > 0.2 -> turnSpeedFieldOriented()
            (drivemode == DriveMode.Defense) -> turnSpeedDefense()
            else -> turnSpeedNormal()
        }
    }
    fun handleResetGyro(){
        if(OI.resetGyroStart){
            resetGyroTimer.reset()
            resetGyroTimer.start()
        }
        if(OI.resetGyro){
            if(resetGyroTimer.hasElapsed(0.5)){
                NavX.reset()
                OI.Rumble.set(0.25,1.0, GenericHID.RumbleType.kRightRumble)
            }
        }
        if(OI.resetGyroEnd){
            resetGyroTimer.stop()
        }
    }
    val climbReachInputBuffer = Timer()
    val climbLiftInputBuffer = Timer()
    fun peripheralControls() {
        if (OI.climbReach.asBoolean || !climbReachInputBuffer.hasElapsed(Constants.ButtonConstants.INPUT_BUFFER_DURATION)) {
            when (Climber.currentState) {
                Constants.ClimberConstants.ClimbHeights.STOWED -> {
                    Climber.setState(Constants.ClimberConstants.ClimbHeights.REACH)
                }
                Constants.ClimberConstants.ClimbHeights.REACH -> {
                    Climber.setState(Constants.ClimberConstants.ClimbHeights.STOWED)
                }
                else -> {
                    climbReachInputBuffer.reset()
                    climbReachInputBuffer.start()
                }
            }
        }
        if (OI.climbLift.asBoolean || !climbLiftInputBuffer.hasElapsed(Constants.ButtonConstants.INPUT_BUFFER_DURATION)) {
            when (Climber.currentState) {
                Constants.ClimberConstants.ClimbHeights.REACH -> {
                    Climber.setState(Constants.ClimberConstants.ClimbHeights.LIFTOFF)
                }
                Constants.ClimberConstants.ClimbHeights.LIFTOFF -> {
                    Climber.setState(Constants.ClimberConstants.ClimbHeights.REACH)
                }
                else -> {
                    climbLiftInputBuffer.reset()
                    climbLiftInputBuffer.start()
                }
            }
        }
        Shooter.setFlywheelSpeed(OI.shooterFlywheel)
    }

    var targetRotation = Odometry.supplyPose().rotation.degrees
    val ANGULAR_P = 0.2
    val ANGULAR_D = 0.0
    val turnController = PIDController(ANGULAR_P, 0.0, ANGULAR_D)
    fun alignRobot() {
        var currentPose = Odometry.supplyPose()
        var poseYaw = 0.0
        var rotationSpeed = 0.0
        if (OI.alignButtonPressed && !alignMode) {
                alignMode = true
        }
        if (OI.alignButtonRelease) {
            alignMode = false
        }
        if (alignMode) {
            if (!vision.hasSpecificTarget(4)) {
                targetRotation = (1*PI)
                rotationSpeed = -turnController.calculate(currentPose.rotation.radians, targetRotation)
            }
            if (vision.hasSpecificTarget(4)) {
                val target = vision.getCameraData(4)
                poseYaw = target.yaw
                println("target rotation" + targetRotation)
                println("Turning")
//                println(xDist)
//                println(yDist)
                println("current rotation" + currentPose.rotation.degrees)
                println("alignMode" + alignMode)

                targetRotation = currentPose.rotation.degrees - poseYaw
//                val targetRotationNegativeError = targetRotation - 3.0
//                val targetRotationPositiveError = targetRotation + 3.0
//                if (currentPose.rotation.degrees >= targetRotationNegativeError && currentPose.rotation.degrees <= targetRotationPositiveError) {
//                    alignMode = false
//                }
                rotationSpeed = -turnController.calculate(currentPose.rotation.radians, targetRotation + (1 * PI))
                println("Pose.yaw:" + poseYaw.degreesToRadians())
                // Use our forward/turn speeds to control the drivetrain
            }
            Drivetrain.drive(0.0, 0.0, rotationSpeed, true, true, true)// Use our forward/turn speeds to control the drivetrain
        }
    }
    override fun execute() {
        alignRobot()
        handleResetGyro()
////        peripheralControls()
        Drivetrain.drive(
            -OI.translationX,
            -OI.translationY,
            getTurnSpeed(),
            fieldRelative = true,
            rateLimit = true,
            secondOrder = true
        )


    }

    // Called once the command ends or is interrupted.
    override fun end(interrupted: Boolean) {
    }


    // Returns true when the command should end.
    override fun isFinished(): Boolean {
        return false
    }
}
