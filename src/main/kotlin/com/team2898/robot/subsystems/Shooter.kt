package com.team2898.robot.subsystems

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import com.team2898.engine.utils.units.*
import com.team2898.robot.Constants.ShooterConstants
import com.team2898.robot.RobotMap.ShooterId
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.wpilibj2.command.SubsystemBase

object Shooter : SubsystemBase() {
    private val shooterMotor = CANSparkMax(ShooterId, CANSparkLowLevel.MotorType.kBrushless)
    private val shooterEncoder = shooterMotor.encoder
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
        shooterMotor.restoreFactoryDefaults()
        shooterMotor.setSmartCurrentLimit(40)
        shooterMotor.idleMode = CANSparkBase.IdleMode.kCoast

    }

    fun shoot(){
        val pidCalc = pid.calculate(shooterEncoder.velocity)
        val ffCalc = ff.calculate(pid.setpoint)
        shooterMotor.set(pidCalc + ffCalc)
    }

    fun setFlywheelSpeed(speed: MetersPerSecond) {
        pid.setpoint = (1.rot * (speed / ShooterConstants.FLYWHEEL_CIRCUMFERENCE.toMeters())).value
    }

}