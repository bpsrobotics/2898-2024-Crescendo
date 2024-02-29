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
    var angleSpeaker = 0.0
    fun peripheralControls() {
        if (vision.hasSpecificTarget(4)) {
            val target = vision.getCameraData(4)
            val distToSpeaker = atan2(2.08, sqrt((target.bestCameraToTarget.x).pow(2)-(1.41 - 0.675).pow(2)))
            angleSpeaker =(0.5 * PI) - (distToSpeaker - 32.0.degreesToRadians() - ((0.5* PI) - ArmConstants.ArmHeights.SHOOTER1.position))
            SmartDashboard.putNumber("AngleToSpeaker", distToSpeaker)
            SmartDashboard.putNumber("arm angle b4", angleSpeaker)
        } else {
            angleSpeaker = 0.0
        }
        if (OI.armSelectUp.asBoolean) {
            Arm.setGoal(Arm.pos() - 0.075)
//            Arm.setGoal(Arm.targetState.up())
        }
        if (OI.armSelectDown.asBoolean) {
            Arm.setGoal(Arm.pos() + 0.075)

//            Arm.setGoal(Arm.targetState.down())
        }
        when {
            OI.armDirectGround.asBoolean -> {
                Arm.setGoal(ArmConstants.ArmHeights.GROUND.position)
            }

            OI.armDirectStowed.asBoolean -> {
                Arm.setGoal(ArmConstants.ArmHeights.STOWED.position)
            }

            OI.armDirectAmp.asBoolean -> {
                Arm.setGoal(ArmConstants.ArmHeights.AMP.position)
            }

            OI.armDirectShooter1.asBoolean -> {
                Arm.setGoal(ArmConstants.ArmHeights.SHOOTER1.position)
            }

            OI.armDirectShooter2.asBoolean -> {
                Arm.setGoal(ArmConstants.ArmHeights.SHOOTER2.position)
            }
        }
        if (OI.operatorTrigger.asBoolean) {
            Shooter.setVoltage(6.0)
        } else if (OI.shooterOutake.asBoolean) {
            Shooter.setVoltage(-0.75)
        } else {
            Shooter.setVoltage(0.0)
        }
        if (OI.runIntake.asBoolean) {
            Intake.intake(0.55)
        } else if (OI.shooterOutake.asBoolean) {
            Intake.outtake()
        } else if (OI.shootNote.asBoolean) {
            InAndOut()
        } else {
            Intake.intake(0.0)
        }



    }

    val ANGULAR_P = 0.1
    val ANGULAR_D = 0.0
    val turnController = PIDController(ANGULAR_P, 0.0, ANGULAR_D)
    fun alignRobot() {
        var targetRotation = Odometry.supplyPose().rotation.degrees
        var currentPose = Odometry.supplyPose()
        var poseYaw = 0.0
        var rotationSpeed = 0.0

        if (vision.hasSpecificTarget(4)) {
            println("see")
        }
        if (vision.hasSpecificTarget(4)) {
            val target = vision.getCameraData(4)
//            if (!vision.hasSpecificTarget(4)) {
//                targetRotation = (1*PI)
//                rotationSpeed = -turnController.calculate(currentPose.rotation.radians, targetRotation)
//            }
            if (OI.alignButtonPressed && !alignMode) {
                alignMode = true
                poseYaw = currentPose.rotation.degrees - target.yaw
            }
            if (OI.alignButtonRelease) {
                alignMode = false
            }
            if (alignMode) {
                println("target rotation" + targetRotation)
                println("Turning")
//                println(xDist)
//                println(yDist)
                println("current rotation" + currentPose.rotation.degrees)
                println("alignMode" + alignMode)

                targetRotation = poseYaw
//                val targetRotationNegativeError = targetRotation - 3.0
//                val targetRotationPositiveError = targetRotation + 3.0
//                if (currentPose.rotation.degrees >= targetRotationNegativeError && currentPose.rotation.degrees <= targetRotationPositiveError) {
//                    alignMode = false
//                }
                rotationSpeed = -turnController.calculate(currentPose.rotation.radians, targetRotation.degreesToRadians() + (1*PI))
                println("Pose.yaw:" + poseYaw.degreesToRadians())
                Drivetrain.drive(0.0, 0.0, rotationSpeed, true, true, true)
            // Use our forward/turn speeds to control the drivetrain
            }


            // Use our forward/turn speeds to control the drivetrain
        }
    }
    override fun execute() {
        OI.loop.poll()
        handleResetGyro()
        alignRobot()
        peripheralControls()
        Drivetrain.drive(
            -OI.translationX,
            -OI.translationY,
            turnSpeedNormal(),
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

    companion object {
        enum class DriveMode {
            Normal,
            Defense
        }
    }
}
