package com.team2898.robot.subsystems

import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import com.team2898.robot.Constants.ClimberConstants.ClimbHeights
import com.team2898.robot.Constants.DriveConstants.kClimberId
import edu.wpi.first.wpilibj2.command.SubsystemBase

object Climber : SubsystemBase() {
    private val climberMotor = CANSparkMax(kClimberId, CANSparkLowLevel.MotorType.kBrushless)

}