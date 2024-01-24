package com.team2898.robot.subsystems

import com.ctre.phoenix6.hardware.CANcoder
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import com.team2898.engine.utils.SetpointMotorController
import com.team2898.robot.Constants.ClimberConstants
import com.team2898.robot.Constants.DriveConstants.kClimberId
import edu.wpi.first.wpilibj2.command.SubsystemBase

object Climber : SubsystemBase() {
    private val climberMotor = CANSparkMax(kClimberId, CANSparkLowLevel.MotorType.kBrushless)
    private val climberCoder = CANcoder(ClimberConstants.kShooterCANcoder)
    var currentState: ClimberConstants.ClimbHeights = ClimberConstants.ClimbHeights.STOWED
    private val climberController = SetpointMotorController(
        climberMotor,
        climberCoder,
        ClimberConstants.kClimberMaxSpeed,
        ClimberConstants.kClimberAcceleration,
        0.0,
        1.0
    )
    fun setState(newState: ClimberConstants.ClimbHeights) {
        val position = newState.position
        climberController.setGoal(position)
    }

    override fun periodic() {
        climberController.tick()
    }
}