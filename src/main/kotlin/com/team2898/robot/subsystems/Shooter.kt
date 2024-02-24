package com.team2898.robot.subsystems

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import com.team2898.robot.RobotMap.ShooterBottomId
import com.team2898.robot.RobotMap.ShooterTopId
import com.team2898.engine.utils.units.*
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.filter.LinearFilter
import edu.wpi.first.units.*
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import java.lang.System.currentTimeMillis
import kotlin.math.abs

object Shooter : SubsystemBase() {
    private val shooterMotorTop = CANSparkMax(ShooterTopId, CANSparkLowLevel.MotorType.kBrushless)
    private val shooterEncoderTop = shooterMotorTop.encoder
    val wheelSpeedTop get() = shooterEncoderTop.velocity
    private val shooterMotorBot = CANSparkMax(ShooterBottomId, CANSparkLowLevel.MotorType.kBrushless)
    private val shooterEncoderBot = shooterMotorBot.encoder
    val wheelSpeedBot get() = shooterEncoderBot.velocity
    // PID Constants
    val kP = 0.5
    val kI = 0.0
    val kD = 0.0
    private val pid = PIDController(kP, kI, kD)
    // Feed Forward Constants
    val kS = 0.15
    val kV = 0.5
    val kA = 0.0
    val ff = SimpleMotorFeedforward(kS, kV, kA)

    val botAverage = LinearFilter.movingAverage(3)
    var currentBotAverage = 0.0
    val topAverage = LinearFilter.movingAverage(3)
    var currentTopAverage = 0.0
    private var prevTime = currentTimeMillis() / 1000.0
    private var prevSpeedTop = 0.0
    private var prevSpeedBot = 0.0


    var motorStop = false
    var speed = 0.0

    var topGoal = 0.0
    var botGoal = 0.0
    var topPID = pid.calculate(wheelSpeedTop, topGoal)
    var topFF = ff.calculate(topGoal)
    var botPID = pid.calculate(wheelSpeedBot, botGoal)
    var botFF = ff.calculate(botGoal)
    init{
        shooterMotorTop.restoreFactoryDefaults()
        shooterMotorTop.setSmartCurrentLimit(40)
        shooterMotorTop.idleMode = CANSparkBase.IdleMode.kCoast
        shooterMotorTop.inverted = true
        shooterMotorTop.burnFlash()

        shooterMotorBot.restoreFactoryDefaults()
        shooterMotorBot.setSmartCurrentLimit(40)
        shooterMotorBot.idleMode = CANSparkBase.IdleMode.kCoast
        shooterMotorBot.inverted = true
        shooterMotorBot.burnFlash()

    }

    override fun periodic() {
        speed = SmartDashboard.getNumber("shooter speed", 2500.0)
        val deltaTime = (currentTimeMillis() / 1000.0) - prevTime
        val deltaSpeed = abs(prevSpeedTop - wheelSpeedTop)
        val deltaSpeedBot = abs(prevSpeedBot - wheelSpeedBot)

        currentTopAverage = topAverage.calculate(deltaSpeed / deltaTime)
        currentBotAverage = botAverage.calculate(deltaSpeedBot / deltaTime)
        prevTime = currentTimeMillis() / 1000.0


        topPID = pid.calculate(wheelSpeedTop, topGoal)
        topFF = ff.calculate(topGoal)
        botPID = pid.calculate(wheelSpeedBot, botGoal)
        botFF = ff.calculate(botGoal)
        SmartDashboard.putNumber("ff", topFF)
        SmartDashboard.putNumber("pid", topPID)
        SmartDashboard.putNumber("velocity top", wheelSpeedTop)

        if(!motorStop){
            shooterMotorTop.setVoltage(topFF + topPID)
            shooterMotorBot.setVoltage(botFF + botPID)
        }


        prevSpeedTop = wheelSpeedTop
        prevSpeedBot = wheelSpeedBot
    }

//    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
//    private val s_appliedVoltage: MutableMeasure<Voltage> = MutableMeasure.mutable(Units.Volts.of(0.0))
//
//    // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
//    private val s_acceleration: MutableMeasure<Angle> = MutableMeasure.mutable(Units.Revolutions.of(0.0))
//
//    // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
//    private val s_velocity: MutableMeasure<Velocity<Angle>> = MutableMeasure.mutable(Units.RevolutionsPerSecond.of(0.0))
//
//    val sysIdRoutine = SysIdRoutine(
//        SysIdRoutine.Config(),
//        SysIdRoutine.Mechanism(
//            { volts: Measure<Voltage> -> setVoltage(volts.`in`(Units.Volts))},
//            { log: SysIdRoutineLog ->
//                log.motor("shooter top")
//                    .voltage(
//                        s_appliedVoltage.mut_replace(
//                        shooterMotorTop.busVoltage, Units.Volts
//                    ))
//                    .angularAcceleration()
//                    .angularVelocity()
//            },
//            this
//        )
//    )

//    fun shooterSysIdDynamic(direction: SysIdRoutine.Direction): Command {
//        return sysIdRoutine.dynamic(direction)
//    }
//    fun shooterSysIdQuasistatic(direction: SysIdRoutine.Direction): Command {
//        return sysIdRoutine.quasistatic(direction)
//    }


    fun setWheelSpeed(rpm: Double) {
        topGoal = rpm
        botGoal = rpm
        motorStop = false
    }


    fun setVoltage(voltage: Double){
        shooterMotorTop.setVoltage(voltage)
        shooterMotorBot.setVoltage(voltage)
    }
    fun stop(){
        shooterMotorTop.stopMotor()
        shooterMotorBot.stopMotor()
        motorStop = true
    }


}