package com.team2898.robot.subsystems

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import com.team2898.robot.Constants.DriveConstants.kIntakeId
import edu.wpi.first.wpilibj2.command.SubsystemBase

class Intake : SubsystemBase() {
    private val intakeMotor = CANSparkMax(kIntakeId, CANSparkLowLevel.MotorType.kBrushless)

    init {
        intakeMotor.restoreFactoryDefaults()
        intakeMotor.setSmartCurrentLimit(15)
        intakeMotor.idleMode = CANSparkBase.IdleMode.kBrake
    }

    fun runIntake(speed: Double){
        intakeMotor.set(speed)
    }


}