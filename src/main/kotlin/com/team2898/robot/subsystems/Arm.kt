package com.team2898.robot.subsystems


import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import com.team2898.engine.utils.MovingAverage
import com.team2898.engine.utils.Sugar.clamp
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
import frc.engine.utils.initMotorControllers
import kotlin.math.PI
import kotlin.math.absoluteValue
import kotlin.math.sign
import kotlin.math.sin


object Arm : SubsystemBase() {

    private val armMotor = CANSparkMax(Arm_left, CANSparkLowLevel.MotorType.kBrushless)
    private val armMotorSecondary = CANSparkMax(Arm_right, CANSparkLowLevel.MotorType.kBrushless)
    private val encoder = DutyCycleEncoder(ArmDigitalInput)

    var setpoint = pos()
    private const val UPPER_SOFT_STOP = -0.275
    val LOWER_SOFT_STOP = 1.76
//    var ksin = 0.877281
    var ksin = 0.86
    var ks = -0.078825
    var kv = -1.82291
//    var kv = -3.42291
    var voltageApplied = 0.0

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
    // 2.65, 1.5, 0,0
//    val pid = PIDController(0.01, 0.0, 0.5)
//    val pid = PIDController(0.01, 0.0, 0.1)
    val pid = PIDController(4.0, 0.0, 0.5)
    var pidCalc = 0.0
    var profile = TrapezoidProfile(constraints)
    var initState = TrapezoidProfile.State(pos(), 0.0)
    var goalState = TrapezoidProfile.State(pos(),0.0)
    var targetSpeed = 0.0


    init {
        initMotorControllers(Constants.ArmConstants.CurrentLimit, CANSparkBase.IdleMode.kBrake, armMotor, armMotorSecondary)

        armMotor.inverted = true

        armMotor.burnFlash()
        armMotorSecondary.burnFlash()

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
        SmartDashboard.putNumber("arm current", armMotor.outputCurrent)
        SmartDashboard.putNumber("arm current 2", armMotorSecondary.outputCurrent)
        SmartDashboard.putNumber("arm duty cycle", armMotor.appliedOutput)
        SmartDashboard.putNumber("rate", movingAverage.average)
        SmartDashboard.putNumber("raw pos", encoder.absolutePosition)
        ks = SmartDashboard.getNumber("arm ks", ks)
        ksin = SmartDashboard.getNumber("arm ksin", ksin)
        kv = SmartDashboard.getNumber("arm kv", kv)

        val armAngle = pos()
        val deltaAngle = armAngle - last
        val deltaTime = timer.get()
        vel = deltaAngle / deltaTime
        timer.reset()
        timer.start()



        targetSpeed = profile.calculate(profileTimer.get(),
            initState,
            goalState
        ).velocity

        SmartDashboard.putNumber("arm target speed", targetSpeed)
        pidCalc = -pid.calculate(armAngle, setpoint)
        SmartDashboard.putNumber("pid calc", pidCalc)
        var output = pidCalc.clamp(-3.5,3.5)
        output += kv * targetSpeed
        output += ks + sin(armAngle) * ksin

        SmartDashboard.putNumber("output", output)
        SmartDashboard.putNumber("target pos", setpoint)
        setVoltage(output)
        last = armAngle
    }

    /**
     * Set the desired angle of the arm in radians
     * @param newPos desired arm angle in radians
     */
    fun setGoal(newPos: Double) {
        if (newPos !in UPPER_SOFT_STOP..LOWER_SOFT_STOP) return
        setpoint = newPos
        profileTimer.reset()
        profileTimer.start()
        initState = TrapezoidProfile.State(pos(), vel)
        goalState = TrapezoidProfile.State(setpoint, 0.0)
    }
    /** Stops the motors and sets to coast mode so the arm will drop */
    fun release() {
        armMotor.stopMotor()
        armMotorSecondary.stopMotor()
        armMotor.idleMode = CANSparkBase.IdleMode.kCoast
        armMotorSecondary.idleMode = CANSparkBase.IdleMode.kCoast
    }
    /** Returns true if the target speed is not 0 and pid is not moving the arm */
    fun isMoving(): Boolean {
        return (targetSpeed != 0.0) && (pidCalc.absoluteValue > 0.25)
    }

    /** Sets the voltage of the arm motors
     * @param volts Wanted voltage*/
    fun setVoltage(volts: Double){
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