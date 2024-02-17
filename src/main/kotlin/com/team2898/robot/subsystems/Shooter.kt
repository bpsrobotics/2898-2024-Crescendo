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
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.SubsystemBase

object Shooter : SubsystemBase() {
    private val shooterMotorTop = CANSparkMax(ShooterTopId, CANSparkLowLevel.MotorType.kBrushless)
    private val shooterEncoderTop = shooterMotorTop.encoder
    private val shooterMotorBot = CANSparkMax(ShooterBottomId, CANSparkLowLevel.MotorType.kBrushless)
    private val shooterEncoderBot = shooterMotorBot.encoder
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

    val botAverage: LinearFilter = LinearFilter.movingAverage(5)
    val topAverage: LinearFilter = LinearFilter.movingAverage(5)
    init{
        shooterMotorTop.restoreFactoryDefaults()
        shooterMotorTop.setSmartCurrentLimit(40)
        shooterMotorTop.idleMode = CANSparkBase.IdleMode.kCoast
        shooterMotorTop.enableVoltageCompensation(12.0)

        shooterMotorBot.restoreFactoryDefaults()
        shooterMotorBot.setSmartCurrentLimit(40)
        shooterMotorBot.idleMode = CANSparkBase.IdleMode.kCoast
        shooterMotorBot.enableVoltageCompensation(12.0)

    }

    val shootingTimer = Timer()
    var shooting = false
    fun shoot(){
//        botAverage
//        val pidCalc = pid.calculate()
//        val ffCalc = ff.calculate()
//        shooterMotorTop.set(pidCalc + ffCalc)
        shooting = true
        shootingTimer.reset()
        shootingTimer.start()
    }

    fun setFlywheelSpeed(speed: MetersPerSecond) {
        pid.setpoint = (1.rot * (speed / ShooterConstants.FLYWHEEL_CIRCUMFERENCE.toMeters())).value
    }

    fun setVoltage(voltage: Double){
        shooterMotorTop.setVoltage(voltage)
    }
    fun stop(){
        shooterMotorTop.stopMotor()
    }

    override fun periodic() {
        if (shooting && shootingTimer.hasElapsed(ShooterConstants.INTAKE_DURATION)) shooting = false
    }

}