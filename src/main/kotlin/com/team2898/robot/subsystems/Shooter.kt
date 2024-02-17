package com.team2898.robot.subsystems

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import com.team2898.robot.RobotMap.ShooterBottomId
import com.team2898.robot.RobotMap.ShooterTopId
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.filter.LinearFilter
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

    fun shoot(){
//        botAverage
//        val pidCalc = pid.calculate()
//        val ffCalc = ff.calculate()
//        shooterMotorTop.set(pidCalc + ffCalc)
    }

    fun setFlywheelSpeed(speed: Double) {
        // set shooter flywheel speed here
    }

    fun setVoltage(voltage: Double){
        shooterMotorTop.setVoltage(voltage)
        shooterMotorBot.setVoltage(voltage)
    }
    fun stop(){
        shooterMotorTop.stopMotor()
        shooterMotorBot.stopMotor()
    }


}