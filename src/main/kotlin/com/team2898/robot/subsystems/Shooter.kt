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
    val kP = 0.0
    val kI = 0.0
    val kD = 0.0
    private val pid = PIDController(kP, kI, kD)
    private val shooterEncoder = shooterMotor.encoder
    init{
        shooterMotor.restoreFactoryDefaults()
        shooterMotor.setSmartCurrentLimit(40)
    }

    fun shoot(){
        val first = pid.calculate(shooterEncoder.velocity)
    }



}