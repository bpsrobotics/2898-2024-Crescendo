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
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.pow
import kotlin.math.sign

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
//    var resetGyroTimer = Timer()
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
    fun getTurnSpeed():Double {
        return when {
            OI.leftTrigger > 0.2 -> turnSpeedFieldOriented()
            (drivemode == DriveMode.Defense) -> turnSpeedDefense()
            else -> turnSpeedNormal()
        }
    }
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
    var armDirectChoosing = false
    val armDirectChoose = Timer()
    fun peripheralControls() {
        if (OI.armSelectUp.asBoolean) {
            println("arm up")
            Arm.setGoal(Arm.targetState.up())
        }
        if (OI.armSelectDown.asBoolean) {
            println("arm down")
            Arm.setGoal(Arm.targetState.down())
        }
        if (armDirectChoosing && !armDirectChoose.hasElapsed(Constants.ButtonConstants.ARM_DIRECT_CHOOSE_DURATION)) {
            when {
                OI.armDirectGround.asBoolean -> {
                    Arm.setGoal(Constants.ArmConstants.ArmHeights.GROUND)
                    println("arm ground")
                    armDirectChoosing = false
                }
                OI.armDirectStowed.asBoolean -> {
                    Arm.setGoal(Constants.ArmConstants.ArmHeights.STOWED)
                    println("arm stowed")
                    armDirectChoosing = false
                }
                OI.armDirectAmp.asBoolean -> {
                    Arm.setGoal(Constants.ArmConstants.ArmHeights.AMP)
                    println("arm amp")
                    armDirectChoosing = false
                }
                OI.armDirectShooter1.asBoolean -> {
                    Arm.setGoal(Constants.ArmConstants.ArmHeights.SHOOTER1)
                    println("arm shooter1")
                    armDirectChoosing = false
                }
                OI.armDirectShooter2.asBoolean -> {
                    Arm.setGoal(Constants.ArmConstants.ArmHeights.SHOOTER2)
                    println("arm shooter2")
                    armDirectChoosing = false
                }
                OI.armDirectSelect.asBoolean -> {
                    println("arm choose cancel")
                    armDirectChoosing = false
                }
            }
        } else if (OI.armDirectSelect.asBoolean) {
            println("arm choose")
            armDirectChoosing = true
            armDirectChoose.reset()
            armDirectChoose.start()
        }
        if (OI.climbAdvance.asBoolean) {
            println("climber advance")
            Climber.setState(Climber.targetState.advance())
        }
        if (OI.climbRetract.asBoolean) {
            println("climber retract")
            Climber.setState(Climber.targetState.retract())
        }
        if (Arm.currentPosition != null) {
            if (OI.operatorTrigger.asBoolean) {
                println("shooter velocity ${Arm.currentPosition!!.velocity}")
                Shooter.setFlywheelSpeed(Arm.currentPosition!!.velocity)
            } else if (OI.operatorTriggerReleased.asBoolean) {
                println("shooter shoot")
                Shooter.shoot()
            } else {
                Shooter.setFlywheelSpeed(0.0)
            }
        }
        if (Arm.currentPosition == Constants.ArmConstants.ArmHeights.GROUND && OI.runIntake.asBoolean) {
            println("run intake")
            Intake.runIntake(Constants.IntakeConstants.INTAKE_SPEED)
        } else {
            Intake.stopIntake()
        }
    }
    override fun execute() {
        OI.loop.poll()
        handleResetGyro()
        peripheralControls()
        Drivetrain.drive(
            -OI.translationX,
            -OI.translationY,
            getTurnSpeed(),
            fieldRelative = true,
            rateLimit = true,
            secondOrder = true
        )
        Arm.voltMove(Arm.voltageApplied)

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
