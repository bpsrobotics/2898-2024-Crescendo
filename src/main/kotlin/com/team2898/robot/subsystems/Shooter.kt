package com.team2898.robot.subsystems

import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import com.team2898.robot.Constants.DriveConstants.kShooterId
import edu.wpi.first.wpilibj2.command.SubsystemBase

class Shooter : SubsystemBase() {
    private val shooterMotor = CANSparkMax(kShooterId, CANSparkLowLevel.MotorType.kBrushless)


}