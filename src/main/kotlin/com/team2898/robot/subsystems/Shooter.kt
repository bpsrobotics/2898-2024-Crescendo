package com.team2898.robot.subsystems

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import com.team2898.robot.RobotMap.ShooterBottomId
import com.team2898.robot.RobotMap.ShooterTopId
import com.team2898.engine.utils.units.*
import com.team2898.robot.Constants.ShooterConstants
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.filter.LinearFilter
import edu.wpi.first.wpilibj2.command.SubsystemBase
import java.lang.System.currentTimeMillis
import kotlin.math.abs

object Shooter : SubsystemBase() {
    private val shooterMotorTop = CANSparkMax(ShooterTopId, CANSparkLowLevel.MotorType.kBrushless)
    private val shooterEncoderTop = shooterMotorTop.encoder
    val wheelSpeedTop = shooterEncoderTop.velocity
    private val shooterMotorBot = CANSparkMax(ShooterBottomId, CANSparkLowLevel.MotorType.kBrushless)
    private val shooterEncoderBot = shooterMotorBot.encoder
    val wheelSpeedBot = shooterEncoderBot.velocity
    // PID Constants
    val kP = 0.0
    val kI = 0.0
    val kD = 0.0
    private val pid = PIDController(kP, kI, kD)
    // Feed Forward Constants
    val kS = 0.0
    val kV = 0.0
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


    var topGoal = 0.0
    var botGoal = 0.0
    val topPID = pid.calculate(wheelSpeedTop, topGoal)
    val topFF = ff.calculate(topGoal)
    val botPID = pid.calculate(wheelSpeedBot, botGoal)
    val botFF = ff.calculate(botGoal)
    init{
        shooterMotorTop.restoreFactoryDefaults()
        shooterMotorTop.setSmartCurrentLimit(40)
        shooterMotorTop.idleMode = CANSparkBase.IdleMode.kCoast
        shooterMotorTop.enableVoltageCompensation(1.0)
        shooterMotorTop.burnFlash()

        shooterMotorBot.restoreFactoryDefaults()
        shooterMotorBot.setSmartCurrentLimit(40)
        shooterMotorBot.idleMode = CANSparkBase.IdleMode.kCoast
        shooterMotorBot.enableVoltageCompensation(1.0)
        shooterMotorBot.inverted
        shooterMotorBot.burnFlash()

    }

    override fun periodic() {
        val deltaTime = (currentTimeMillis() / 1000.0) - prevTime
        val deltaSpeed = abs(prevSpeedTop - wheelSpeedTop)
        val deltaSpeedBack = abs(prevSpeedBot - wheelSpeedBot)

        currentTopAverage = topAverage.calculate(deltaSpeed / deltaTime)
        currentBotAverage = botAverage.calculate(deltaSpeedBack / deltaTime)
        prevTime = currentTimeMillis() / 1000.0

        if(!motorStop){
            shooterMotorTop.setVoltage(topFF + topPID)
            shooterMotorBot.setVoltage(botFF + botPID)
        }

        prevSpeedTop = wheelSpeedTop
        prevSpeedBot = wheelSpeedBot
    }

    fun shoot(){

//        botAverage
//        val pidCalc = pid.calculate()
//        val ffCalc = ff.calculate()
//        shooterMotorTop.set(pidCalc + ffCalc)
    }
    fun setWheelSpeed(rpm: Double) {
        topGoal = rpm
        botGoal = rpm
        motorStop = false
    }

    fun setFlywheelSpeed(speed: MetersPerSecond) {
        pid.setpoint = (1.rot * (speed / ShooterConstants.FLYWHEEL_CIRCUMFERENCE.toMeters())).value
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


    fun setFlywheelSpeed(speed: Double) = setFlywheelSpeed(speed.mps)
}