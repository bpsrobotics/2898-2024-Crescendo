package com.team2898.robot.subsystems

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax

import com.team2898.robot.RobotMap.IntakeId
import edu.wpi.first.wpilibj2.command.SubsystemBase

object Intake : SubsystemBase() {
    private val intakeMotor = CANSparkMax(IntakeId, CANSparkLowLevel.MotorType.kBrushless)
    var hasNote: Boolean = false
        private set

    init {
        intakeMotor.restoreFactoryDefaults()
        intakeMotor.setSmartCurrentLimit(20)
        intakeMotor.idleMode = CANSparkBase.IdleMode.kCoast

    }

    override fun periodic() {
    }

    fun runIntake(speed: Double){
        intakeMotor.set(speed)
    }

    fun stopIntake(){
        intakeMotor.stopMotor()
    }


}