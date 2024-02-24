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
import com.team2898.robot.Constants
import com.team2898.robot.OI
import com.team2898.robot.subsystems.*
import edu.wpi.first.math.controller.PIDController
import com.team2898.engine.utils.units.*
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import org.photonvision.PhotonUtils
import kotlin.math.*
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import com.team2898.robot.Constants.*
import edu.wpi.first.wpilibj2.command.InstantCommand
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.pow
import kotlin.math.sign

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

    val pid = TurningPID(3.5,0.05)
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

        pid.setPoint = angle
        var turnSpeed = pid.turnspeedOutputNoNormalize(-NavX.totalRotation.degreesToRadians())
        if (!turnSpeed.eqEpsilon(0, 0.04)) turnSpeed += kS * turnSpeed.sign
        return turnSpeed
    }
    fun turnSpeedFieldOriented(): Double {
        if((abs(OI.turnY)+abs(OI.turnX)).eqEpsilon(0,0.2)) return 0.0
        angle = atan2(OI.turnY,OI.turnX)
        SmartDashboard.putNumber("angle", angle)

        pid.setPoint = angle
        var turnSpeed = pid.turnspeedOutputNoNormalize(NavX.getInvertedAngle().degreesToRadians())
        if (!turnSpeed.eqEpsilon(0, 0.04)) turnSpeed += kS * turnSpeed.sign
        return turnSpeed
    }
//    fun getTurnSpeed():Double {
//        return when {
//            OI.leftTrigger > 0.2 -> turnSpeedFieldOriented()
//            (drivemode == DriveMode.Defense) -> turnSpeedDefense()
//            else -> turnSpeedNormal()
//        }
//    }
    fun handleResetGyro(){
//        if(OI.resetGyroStart){
//            resetGyroTimer.reset()
//            resetGyroTimer.start()
//        }
        if(OI.resetGyro.asBoolean) {
//            if(resetGyroTimer.hasElapsed(0.5)){
                NavX.reset()
                OI.Rumble.set(0.25,1.0, GenericHID.RumbleType.kRightRumble)
//            }
        }
//        if(OI.resetGyroEnd){
//            resetGyroTimer.stop()
//        }
    }
    val climbReachInputBuffer = Timer()
    val climbLiftInputBuffer = Timer()
    fun peripheralControls() {

        if (OI.armSelectUp.asBoolean) {
            println("arm up")
            Arm.setGoal(Arm.pos() - 0.05)
//            Arm.setGoal(Arm.targetState.up())
        }
        if (OI.armSelectDown.asBoolean) {
            println("arm down")
            Arm.setGoal(Arm.pos() + 0.05)

//            Arm.setGoal(Arm.targetState.down())
        }
        when {
            OI.armDirectGround.asBoolean -> {
                Arm.setGoal(ArmConstants.ArmHeights.GROUND.position)
                println("arm ground")
            }

            OI.armDirectStowed.asBoolean -> {
                Arm.setGoal(ArmConstants.ArmHeights.STOWED.position)
                println("arm stowed")
            }

            OI.armDirectAmp.asBoolean -> {
                Arm.setGoal(ArmConstants.ArmHeights.AMP.position)
                println("arm amp")
            }

            OI.armDirectShooter1.asBoolean -> {
                Arm.setGoal(ArmConstants.ArmHeights.SHOOTER1.position)
                println("arm shooter1")
            }

            OI.armDirectShooter2.asBoolean -> {
                Arm.setGoal(ArmConstants.ArmHeights.SHOOTER2.position)
                println("arm shooter2")
            }
        }
        if (OI.climb.asBoolean) {
            println("climbing!!!")
        }
//        Climber.setState(OI.climb.asBoolean)
//        if (Arm.currentPosition != null) {
        if (OI.operatorTrigger.asBoolean) {
            Shooter.setVoltage(7.0)
//            SmartDashboard.putNumber("shooter speed", 60.0)
//            val speed = SmartDashboard.getNumber("shooter speed", 610.0)
//            Shooter.setWheelSpeed(Shooter.speed)
        }
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
        OI.loop.poll()
        handleResetGyro()
        peripheralControls()
        Drivetrain.drive(
            -OI.translationX,
            -OI.translationY,
            turnSpeedNormal(),
            fieldRelative = true,
            rateLimit = true,
            secondOrder = true
        )
//        Arm.voltMove(Arm.voltageApplied)


    }

    // Called once the command ends or is interrupted.
    override fun end(interrupted: Boolean) {
    }


    // Returns true when the command should end.
    override fun isFinished(): Boolean {
        return false
    }

    companion object {
        enum class DriveMode {
            Normal,
            Defense
        }
    }
}
