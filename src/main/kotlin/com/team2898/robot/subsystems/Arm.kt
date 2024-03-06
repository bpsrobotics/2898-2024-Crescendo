package com.team2898.robot.subsystems


import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import com.team2898.engine.utils.MovingAverage
import com.team2898.engine.utils.Sugar.radiansToDegrees
import com.team2898.engine.utils.async.EventTarget
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

    val onStop = EventTarget<Constants.ArmConstants.ArmHeights?, Arm>(this)
    var setpoint = pos()
    private const val UPPER_SOFT_STOP = -0.275
    val LOWER_SOFT_STOP = 1.76
    private var stopped = false
//    var ksin = 0.877281
    var ksin = 0.86
    var ks = -0.078825
//    var kv = -2.42291
    var kv = -2.9
//    var kv = -3.42291
    var voltageApplied = 0.0
    var currentPosition: Constants.ArmConstants.ArmHeights? = null

    var vel = 0.0

    fun pos(): Double {
        val p = if (encoder.absolutePosition < 0.3) {
            ((encoder.absolutePosition + 1.07) * 2 * PI)
        } else {
            (encoder.absolutePosition + 0.07) * 2 * PI
        }
        return angleModulus(p)
    }

    val movingAverage = MovingAverage(15)
    val movingAverage2 = MovingAverage(25)

    val profileTimer = Timer()

    val constraints = TrapezoidProfile.Constraints(
        ArmMaxSpeed,
        Arm_MaxAccel
    )
    // 0.5, 0.0, 0.15 undershoots
//    val pid = PIDController(0.01, 0.0, 0.5)
    val pid = PIDController(0.01, 0.0, 0.1)
//    var profile: TrapezoidProfile? = null
    var profile = TrapezoidProfile(constraints)
    var initState = TrapezoidProfile.State(pos(), 0.0)
    var goalState = TrapezoidProfile.State(pos(),0.0)
    var targetSpeed = 0.0

    private val integral = MovingAverage(50)

    init {
        armMotor.restoreFactoryDefaults()
        armMotor.setSmartCurrentLimit(40)
        armMotor.idleMode = CANSparkBase.IdleMode.kBrake
        armMotor.inverted = true
        armMotor.burnFlash()

        armMotorSecondary.restoreFactoryDefaults()
        armMotorSecondary.setSmartCurrentLimit(40)
        armMotorSecondary.idleMode = CANSparkBase.IdleMode.kBrake
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
        onStop {
            if (it.data == null) println("Arm has stopped at a position not within tolerance of an ArmHeight")
        }
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

        val p = pos()
        val dp = p - last
        val dt = timer.get()
        vel = dp / dt
        timer.reset()
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


        targetSpeed = profile.calculate(profileTimer.get(),
            initState,
            goalState
        ).velocity
        if (pos() in (setpoint-0.01)..(setpoint+0.01)) {
            targetSpeed = 0.0
        }
        SmartDashboard.putNumber("arm target speed", targetSpeed)
        if ((p - setpoint).absoluteValue < 0.05 && vel.absoluteValue < 0.05) {
            val newPos = Constants.ArmConstants.ArmHeights.entries.toTypedArray()
                .find { (it.position - p).absoluteValue < 0.05 }
            if (newPos != currentPosition) {
                currentPosition = newPos
                onStop(currentPosition)
            }
        }
        var output = pid.calculate(vel, targetSpeed)
        output += kv * targetSpeed
        output += ks + sin(p) * ksin
//
//        if (p < UPPER_SOFT_STOP) {
//            output = output.coerceAtLeast(ks + sin(p) * ksin - 0.2)
//            println("UPPER SOFT STOP")
//        } else if (p > LOWER_SOFT_STOP) {
//            output = output.coerceAtMost(ks + sin(p) * ksin + 0.2)
//            println("LOWER SOFT STOP")
//        } else {
//            println("ur good bro")
//        }
        SmartDashboard.putNumber("output", output)
        SmartDashboard.putNumber("target pos", setpoint)
        voltMove(output)
        last = p
    }

    fun setGoal(newPos: Double) {
        if (newPos !in UPPER_SOFT_STOP..LOWER_SOFT_STOP) return
        setpoint = newPos
        profileTimer.reset()
        profileTimer.start()
        initState = TrapezoidProfile.State(pos(), vel)
        goalState = TrapezoidProfile.State(setpoint, 0.0)
    }

    fun release() {
        armMotor.stopMotor()
        armMotorSecondary.stopMotor()
        armMotor.idleMode = CANSparkBase.IdleMode.kCoast
        armMotorSecondary.idleMode = CANSparkBase.IdleMode.kCoast
    }

    fun isMoving(): Boolean {
        return targetSpeed != 0.0
    }
    fun voltMove(volts: Double){
        armMotor.setVoltage(volts)
        armMotorSecondary.setVoltage(volts)
    }
    override fun initSendable(builder: SendableBuilder) {
        builder.addDoubleProperty("arm position", { pos() }) {}
        builder.addDoubleProperty("raw position", { encoder.absolutePosition }) {}
        builder.addDoubleProperty("arm motor rate", { armMotor.encoder.velocity }) {}
        builder.addDoubleProperty("average velocity rate", { movingAverage.average }) {}
        builder.addDoubleProperty("motor output", { armMotor.appliedOutput }) {}
        builder.addDoubleProperty("target position", { setpoint }) {}
        builder.addDoubleProperty("left voltage", { armMotor.busVoltage }) {}
        builder.addDoubleProperty("right voltage", { armMotorSecondary.busVoltage }) {}
        builder.addDoubleProperty("actual velocity", { vel }) {}
        builder.addDoubleProperty("target velocity", { targetSpeed }) {}
        builder.addDoubleProperty("current drawn (both motors)", { armMotor.busVoltage + armMotorSecondary.busVoltage }) {}

    }

}