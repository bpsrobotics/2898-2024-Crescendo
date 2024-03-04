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
import edu.wpi.first.units.Angle
import edu.wpi.first.wpilibj.DriverStation
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
//    val targetID = if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
//        Constants.VisionConstants.RED_ALLIANCE_SPEAKER_TAG_ID
//    } else{
//        Constants.VisionConstants.BLUE_ALLIANCE_SPEAKER_TAG_ID
//    }
    val targetID = 4
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
        var x1 = 0.0
        var y1 = 0.0
        var h = 1.41 - 0.23375
        if (vision.hasSpecificTarget(targetID)) {
            var d = vision.getCameraData(targetID).x
            var distToSpeaker = sqrt(d.pow(2)-h.pow(2))
            var angleToSpeaker = 0.0
            for(i in 1..5) {
                angleToSpeaker = (180.0 - atan2(2.08 - y1, distToSpeaker + x1).radiansToDegrees() - (26.42+90+10.88)).degreesToRadians()
                x1 = 0.10125 + 0.6*cos(angleToSpeaker)
                y1 = 0.255 + 0.6*sin(angleToSpeaker)
            }
            angleSpeaker =(0.5 * PI) - angleToSpeaker
            SmartDashboard.putNumber("AngleToSpeaker", distToSpeaker)
            SmartDashboard.putNumber("arm angle b4", angleSpeaker)
        } else {
            angleSpeaker = ArmConstants.ArmHeights.SHOOTER1.position
        }
        if (OI.armSelectUp.asBoolean) {
            Arm.setGoal(Arm.pos() - 0.1)
//            Arm.setGoal(Arm.targetState.up())
        }
        if (OI.armSelectDown.asBoolean) {
            Arm.setGoal(Arm.pos() + 0.1)

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
                Arm.setGoal(angleSpeaker)
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
    var target = 0.0
    fun alignRobot() {
        var currentPose = Odometry.supplyPose()
        var poseYaw = 0.0
        var rotationSpeed = 0.0
        var targetRotation = currentPose.rotation.degrees

        if (vision.hasSpecificTarget(targetID)) {
            println("see")
        }
        if (vision.hasSpecificTarget(targetID)) {
            target = vision.getCameraYaw(targetID)
//            if (!vision.hasSpecificTarget(4)) {
//                targetRotation = (1*PI)
//                rotationSpeed = -turnController.calculate(currentPose.rotation.radians, targetRotation)
//            }
            if (OI.alignButtonPressed && !alignMode) {
                alignMode = true
            }
            if (OI.alignButtonRelease) {
                alignMode = false
            }
            if (alignMode) {
                println("target rotation" + targetRotation)
                println("Turning")
                println("alignMode" + alignMode)


                targetRotation = currentPose.rotation.degrees + target
//                val targetRotationNegativeError = targetRotation - 3.0
//                val targetRotationPositiveError = targetRotation + 3.0
//                if (currentPose.rotation.degrees >= targetRotationNegativeError && currentPose.rotation.degrees <= targetRotationPositiveError) {
//                    alignMode = false
//                }
                rotationSpeed = turnController.calculate(currentPose.rotation.radians, targetRotation.degreesToRadians())
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
            OI.translationX,
            OI.translationY,
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
