package com.team2898.robot.subsystems

import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import com.team2898.engine.utils.controls.Controller
import com.team2898.robot.Constants.DriveConstants.kShooterId
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.wpilibj2.command.SubsystemBase

object Shooter : SubsystemBase() {
    private val shooterMotor = CANSparkMax(kShooterId, CANSparkLowLevel.MotorType.kBrushless)
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
    }

    fun shoot(){
        val pidCalc = pid.calculate(shooterEncoder.velocity)
        val ffCalc = ff.calculate(pid.setpoint)
        shooterMotor.set(pidCalc + ffCalc)
    }

    fun setFlywheelSpeed(speed: Double) {
        // set shooter flywheel speed here
    }

}