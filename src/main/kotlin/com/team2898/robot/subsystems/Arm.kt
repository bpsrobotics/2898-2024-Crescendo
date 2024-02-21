package com.team2898.robot.subsystems


import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import com.team2898.engine.utils.MovingAverage
import com.team2898.engine.utils.Sugar.radiansToDegrees
import com.team2898.robot.Constants
import com.team2898.robot.Constants.ArmConstants.ArmMaxSpeed
import com.team2898.robot.Constants.ArmConstants.Arm_MaxAccel
import com.team2898.robot.RobotMap.ArmDigitalInput

import com.team2898.robot.RobotMap.Arm_left
import com.team2898.robot.RobotMap.Arm_right
import edu.wpi.first.math.MathUtil.angleModulus
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.DutyCycleEncoder
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard

import edu.wpi.first.wpilibj2.command.SubsystemBase
import kotlin.math.PI
import kotlin.math.absoluteValue
import kotlin.math.sin


object Arm : SubsystemBase() {

    private val armMotor = CANSparkMax(Arm_left, CANSparkLowLevel.MotorType.kBrushless)
    private val armMotorSecondary = CANSparkMax(Arm_right, CANSparkLowLevel.MotorType.kBrushless)
    private val encoder = DutyCycleEncoder(ArmDigitalInput)

    var setpoint = pos()
    private const val UPPER_SOFT_STOP = -0.3
    val LOWER_SOFT_STOP = 1.7
    private var stopped = false
    var ksin = 0.614274
    var ks = -0.114699
//    var kv = -3.85556
    var kv = -5.0
    var voltageApplied = 0.0
    var currentPosition: Constants.ArmConstants.ArmHeights? = null

    fun pos(): Double {
        val p = if (encoder.absolutePosition < 0.3) {
            ((encoder.absolutePosition + 1.07) * 2 * PI)
        } else {
            (encoder.absolutePosition + 0.07) * 2 * PI
        }
        return angleModulus(p)
    }

    val movingAverage = MovingAverage(3)
    val movingAverage2 = MovingAverage(25)

    val profileTimer = Timer()

    val constraints = TrapezoidProfile.Constraints(
        ArmMaxSpeed,
        Arm_MaxAccel
    )
    val pid = PIDController(0.0, 0.0, 0.0)
    var profile: TrapezoidProfile? = null
    private val integral = MovingAverage(50)

    init {
        armMotor.restoreFactoryDefaults()
        armMotor.setSmartCurrentLimit(40)
        armMotor.idleMode = CANSparkBase.IdleMode.kCoast
        armMotor.inverted = true
        armMotor.burnFlash()

        armMotorSecondary.restoreFactoryDefaults()
        armMotorSecondary.setSmartCurrentLimit(40)
        armMotorSecondary.idleMode = CANSparkBase.IdleMode.kCoast
        armMotor.burnFlash()

        SmartDashboard.putNumber("arm kp", 0.0)
        SmartDashboard.putNumber("arm kd", 0.0)
        SmartDashboard.putNumber("arm ks", ks)
        SmartDashboard.putNumber("arm ksin", ksin)
        SmartDashboard.putNumber("arm kv", kv)
        SmartDashboard.putNumber("position", pos())
        SmartDashboard.putNumber("voltage applied", voltageApplied)
        SmartDashboard.putNumber("angle deg", encoder.absolutePosition * 360)
        SmartDashboard.putNumber("angle from lower deg", LOWER_SOFT_STOP.radiansToDegrees() - pos().radiansToDegrees())
    }

    var last = pos()
    val timer = Timer()

    override fun periodic() {
        SmartDashboard.putNumber("arm position ", pos())
        SmartDashboard.putNumber("quotient", armMotor.encoder.velocity / movingAverage.average)
        SmartDashboard.putNumber("arm current", armMotor.outputCurrent)
        SmartDashboard.putNumber("arm duty cycle", armMotor.appliedOutput)
        SmartDashboard.putNumber("rate", movingAverage.average)
        SmartDashboard.putNumber("raw pos", encoder.absolutePosition)
        ks = SmartDashboard.getNumber("arm ks", ks)
        ksin = SmartDashboard.getNumber("arm ksin", ksin)
        kv = SmartDashboard.getNumber("arm kv", kv)
        voltageApplied = SmartDashboard.getNumber("voltage applied", voltageApplied)
//        println(voltageApplied)
        val currentTick = false

        val p = pos()
        val dp = p - last
        val dt = timer.get()
        timer.start()
        movingAverage.add(vel)
        movingAverage2.add(dp / dt)
        val rate = movingAverage.average
//        val averagedRate = movingAverage2.average

        integral.add((rate - armMotor.encoder.velocity).absoluteValue)

        if (stopped) {
            println("STOPPED")
            armMotor.set(0.0)
            return
        }

        pid.p = SmartDashboard.getNumber("arm kp", pid.p)
        pid.d = SmartDashboard.getNumber("arm kd", pid.d)

//        val targetSpeed = profile?.calculate(profileTimer.get())?.velocity ?: 0.0
//        val targetSpeed = profile.calculate(profileTimer.get(), )
        targetSpeed = profile.calculate(profileTimer.get(),
            initState,
            goalState
        ).velocity
        println("timer" + profileTimer.get())
        SmartDashboard.putNumber("arm target speed", targetSpeed)
//        if (setpoint !in LOWER_SOFT_STOP..UPPER_SOFT_STOP) {
////            profile = null
//            targetSpeed = 0.0
//            currentPosition = Constants.ArmConstants.ArmHeights.entries.toTypedArray().find { (it.position - p).absoluteValue < 0.05 }
//        }
        var output = pid.calculate(vel, targetSpeed)
        println("pid output $output")
        output += kv * targetSpeed
        output += ks + sin(p) * ksin
//
//        if (p < UPPER_SOFT_STOP) {
//            output = output.coerceAtLeast(0.0)
//            println("UPPER SOFT STOP")
//        } else if (p > LOWER_SOFT_STOP) {
//            output = output.coerceAtMost(0.0)
//            println("LOWER SOFT STOP")
//        } else {
//            println("ur good bro")
//        }
        SmartDashboard.putNumber("output", output)
        voltMove(output)
//        armMotor.setVoltage(output)
//        armMotorSecondary.set(output)
        last = p
    }

    fun setGoal(newPos: Double) {
        if (newPos !in UPPER_SOFT_STOP..LOWER_SOFT_STOP) return
        setpoint = newPos

        profile = TrapezoidProfile(constraints)
        profile?.calculate(profileTimer.get(),
            TrapezoidProfile.State(pos(), movingAverage.average),
            TrapezoidProfile.State(newPos, 0.0)
        )
        profileTimer.reset()
        profileTimer.start()

    }

    fun release() {
        armMotor.stopMotor()
        armMotorSecondary.stopMotor()
        armMotor.idleMode = CANSparkBase.IdleMode.kCoast
        armMotorSecondary.idleMode = CANSparkBase.IdleMode.kCoast
//        profile = null
    }

    fun isMoving(): Boolean {
        return targetSpeed != 0.0
    }
    fun voltMove(volts: Double){
        armMotor.setVoltage(volts)
        armMotorSecondary.setVoltage(volts)
    }
    override fun initSendable(builder: SendableBuilder) {
        builder.addDoubleProperty("position", { pos() }) {}
        builder.addDoubleProperty("raw position", { encoder.absolutePosition }) {}
        builder.addDoubleProperty("arm motor rate", { armMotor.encoder.velocity }) {}
        builder.addDoubleProperty("rate", { movingAverage.average }) {}
        builder.addDoubleProperty("rate2", { movingAverage2.average }) {}
        builder.addDoubleProperty("motor output", { armMotor.appliedOutput }) {}
    }

}