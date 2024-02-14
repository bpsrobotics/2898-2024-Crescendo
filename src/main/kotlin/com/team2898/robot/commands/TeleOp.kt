package com.team2898.robot.commands

//The commands for both the driver and operator
//import com.com.engine.utils.`M/s`
//import com.team2898.robot.Constants.ArmHeights.*
//import com.team2898.robot.Field
//import com.team2898.robot.OI
//import com.team2898.robot.subsystems.Arm

import com.team2898.engine.utils.Sugar.degreesToRadians
import com.team2898.engine.utils.Sugar.eqEpsilon
import com.team2898.engine.utils.Sugar.radiansToDegrees
import com.team2898.engine.utils.TurningPID
import com.team2898.engine.utils.odometry.Vision
import com.team2898.robot.Constants
import com.team2898.robot.OI
import com.team2898.robot.subsystems.Drivetrain
import com.team2898.robot.subsystems.NavX
import com.team2898.robot.subsystems.Odometry
import edu.wpi.first.math.MathUtil.angleModulus
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
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
    var targetRotation = Odometry.supplyPose().rotation.degrees
    val ANGULAR_P = 0.2
    val ANGULAR_D = 0.0
    val turnController = PIDController(ANGULAR_P, 0.0, ANGULAR_D)
    fun alignRobot() {
        if (vision.hasTargets()) {
            var currentPose = Odometry.supplyPose()
            val pose = vision.getCameraData()
            if (OI.alignButtonPressed && !alignMode) {
                    xDist = pose.bestCameraToTarget.x
                    yDist = pose.bestCameraToTarget.y

                alignMode = true
            }
            if (OI.alignButtonRelease) {
                alignMode = false
            }
            if (alignMode) {
                println("target rotation" + targetRotation)
                println("Turning")
                println(xDist)
                println(yDist)
                println("current rotation" + currentPose.rotation.degrees)
                println("alignMode" + alignMode)
                targetRotation = atan2(xDist, yDist).radiansToDegrees() + (0.5 * PI).radiansToDegrees()
                val targetRotationNegativeError = targetRotation - 30.0
                val targetRotationPositiveError = targetRotation + 30.0
                println("targetRotationNegativeError"+targetRotationNegativeError)
                println("targetRotationPositiveError"+targetRotationPositiveError)
                if (abs(currentPose.rotation.degrees) >= abs(targetRotationNegativeError) && abs(currentPose.rotation.degrees) <= abs(targetRotationPositiveError)) {
                    alignMode = false
                }

                val rotationSpeed = if (yDist < 0) {
                    0.1
                } else{
                    -0.1
                }
//                val rotationSpeed = -turnController.calculate(pose.yaw.degreesToRadians() , targetRotation + (1 * PI))
                Drivetrain.drive(0.0, 0.0, rotationSpeed, true, true)
                println("targetRotation:"+targetRotation)
                println("Pose.yaw:"+pose.yaw.degreesToRadians())
                println("rotationSpeed:"+rotationSpeed)

                // Use our forward/turn speeds to control the drivetrain

                // Use our forward/turn speeds to control the drivetrain
            }
        }
    }
    override fun execute() {
        alignRobot()
        var speedMultiplier = Constants.OIConstants.kSpeedMultiplierMin
        handleResetGyro()
        val SpeedDif = Constants.OIConstants.kSpeedMultiplierMax - Constants.OIConstants.kSpeedMultiplierMin
        if(OI.rightTrigger > 0.2) speedMultiplier += SpeedDif* OI.rightTrigger
        val turnSpeed = getTurnSpeed() * speedMultiplier

        /*if(OI.defenseModeButton) drivemode = DriveMode.Defense
        if(OI.normalModeButton) drivemode = DriveMode.Normal

        val turnSpeed = getTurnSpeed() * speedMultiplier

        if(drivemode == DriveMode.Defense) {
            if (OI.translationX == 0.0 && OI.translationY == 0.0 && turnSpeed.eqEpsilon(0, 0.04)) {
                if (breakTimer.hasElapsed(breakTimerGoal)) Drivetrain.lock()
                else Drivetrain.drive(0.0, 0.0, 0.0, fieldRelative = true, rateLimit = true)
                return
            } else {
                breakTimer.reset()
                breakTimerGoal = (Odometry.velocity.norm + turnSpeed / 2) / 3
            }
        }*/
        Drivetrain.drive(
            -(OI.translationY)*speedMultiplier, //* OI.translationY.sign,
            -(OI.translationX)*speedMultiplier, //* OI.translationX.sign,
            turnSpeed,
            fieldRelative = true,
            rateLimit = true

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
