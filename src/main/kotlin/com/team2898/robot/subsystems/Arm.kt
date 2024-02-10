package com.team2898.robot.subsystems


import com.ctre.phoenix6.hardware.CANcoder
import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import com.team2898.engine.utils.MovingAverage
import com.team2898.robot.Constants.ArmConstants
import com.team2898.robot.Constants.ArmConstants.ArmMaxSpeed
import com.team2898.robot.Constants.ArmConstants.Arm_MaxAccel

import com.team2898.robot.RobotMap
import com.team2898.robot.RobotMap.Arm_left
import com.team2898.robot.RobotMap.Arm_right
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard

import edu.wpi.first.wpilibj2.command.SubsystemBase
import kotlin.math.PI
import kotlin.math.absoluteValue
import kotlin.math.sin


object Arm : SubsystemBase() {

    private val armMotor = CANSparkMax(Arm_left, CANSparkLowLevel.MotorType.kBrushless)
    private val armMotorSecondary = CANSparkMax(Arm_right, CANSparkLowLevel.MotorType.kBrushless)
    private val encoder = CANcoder(RobotMap.ArmCANCoder)

    var setpoint = pos()
    private const val UPPER_SOFT_STOP = 0.0
    val LOWER_SOFT_STOP = 0.0 //TODO() Set Stop Positions
    var ksin = 0.0
    var ks = 0.0
    var kv = 0.0
    var currentPosition: ArmConstants.ArmHeights? = null

    fun pos(): Double {
        val p = encoder.absolutePosition.valueAsDouble * 2 * PI
        return p
    //TODO() DO the offset stuff for these positions
    }

    val movingAverage = MovingAverage(15)
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
        armMotor.idleMode = CANSparkBase.IdleMode.kBrake

        armMotorSecondary.restoreFactoryDefaults()
        armMotorSecondary.setSmartCurrentLimit(40)
        armMotorSecondary.idleMode = CANSparkBase.IdleMode.kBrake
        armMotorSecondary.follow(armMotor)

        SmartDashboard.putNumber("arm kp", 0.0)
        SmartDashboard.putNumber("arm kd", 0.0)
        SmartDashboard.putNumber("arm ks", ks)
        SmartDashboard.putNumber("arm ksin", ksin)
        SmartDashboard.putNumber("arm kv", kv)

    }

    var last = pos()
    val timer = Timer()

    override fun periodic() {
        SmartDashboard.putNumber("quotient", armMotor.encoder.velocity / movingAverage.average)
        SmartDashboard.putNumber("arm current", armMotor.outputCurrent)
        SmartDashboard.putNumber("arm duty cycle", armMotor.appliedOutput)
        ks = SmartDashboard.getNumber("arm ks", ks)
        ksin = SmartDashboard.getNumber("arm ksin", ksin)
        kv = SmartDashboard.getNumber("arm kv", kv)

        val currentTick = false

        val p = pos()
        val dp = p - last
        last = p
        val dt = timer.get()
        timer.start()
        timer.reset()
        movingAverage.add(dp / dt)
        movingAverage2.add(dp / dt)
        val rate = movingAverage.average
//        val averagedRate = movingAverage2.average

        integral.add((rate - armMotor.encoder.velocity).absoluteValue)

        pid.p = SmartDashboard.getNumber("arm kp", 0.0)
        pid.d = SmartDashboard.getNumber("arm kd", 0.0)
        if (setpoint == 0.0 || setpoint !in LOWER_SOFT_STOP..UPPER_SOFT_STOP || ((p - setpoint).absoluteValue < 0.05 && rate.absoluteValue < 0.1) || profileTimer.get() > (profile?.totalTime() ?: 0.0)) {
            profile = null
            currentPosition = ArmConstants.ArmHeights.entries.toTypedArray().find { (it.position - p).absoluteValue < 0.05 }
        }
//        val targetSpeed = profile?.calculate(profileTimer.get())?.velocity ?: 0.0
//        val targetSpeed = profile.calculate(profileTimer.get(), )
        val targetSpeed = profile?.calculate(profileTimer.get(),
            TrapezoidProfile.State(pos(), movingAverage.average),
            TrapezoidProfile.State(pos(), 0.0)
        )?.velocity ?: 0.0
        SmartDashboard.putNumber("arm target speed", targetSpeed)

        var output = pid.calculate(rate, targetSpeed)
        output += kv * targetSpeed
        output += ks + sin(p) * ksin

        if (p > UPPER_SOFT_STOP) {
            output = output.coerceAtMost(0.0)
            println("UPPER SOFT STOP")
        } else if (p < LOWER_SOFT_STOP || currentTick) {
            output = output.coerceAtLeast(0.0)
            println("LOWER SOFT STOP")
        }
        armMotor.set(output)



    }

    fun setGoal(newPos: Double) {
        if (newPos !in LOWER_SOFT_STOP..UPPER_SOFT_STOP) return
//        currentGoal = newPos
        setpoint = newPos
//        profile = TrapezoidProfile(constraints,
//            TrapezoidProfile.State(newPos, 0.0),
//            TrapezoidProfile.State(pos(), movingAverage.average)
//        )
        profile?.calculate(profileTimer.get(),
            TrapezoidProfile.State(pos(), movingAverage.average),
            TrapezoidProfile.State(newPos, 0.0)
        )
        profileTimer.reset()
        profileTimer.start()

    }
    fun setGoal(newPos: ArmConstants.ArmHeights) {
        setGoal(newPos.position)
    }

    fun stop() {
        profile = null
    }

    fun isMoving(): Boolean {
        return profile != null
    }
    override fun initSendable(builder: SendableBuilder) {
        builder.addDoubleProperty("position", { pos() }) {}
        builder.addDoubleProperty("raw position", { encoder.absolutePosition.valueAsDouble }) {}
        builder.addDoubleProperty("arm motor rate", { armMotor.encoder.velocity }) {}
        builder.addDoubleProperty("rate", { movingAverage.average }) {}
        builder.addDoubleProperty("rate2", { movingAverage2.average }) {}
        builder.addDoubleProperty("motor output", { armMotor.appliedOutput }) {}
    }

}