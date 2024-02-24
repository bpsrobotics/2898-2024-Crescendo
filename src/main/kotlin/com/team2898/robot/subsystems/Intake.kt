package com.team2898.robot.subsystems

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax

import com.team2898.robot.RobotMap.IntakeId
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase

object Intake : SubsystemBase() {
    private val intakeMotor = CANSparkMax(IntakeId, CANSparkLowLevel.MotorType.kBrushed)
    var hasNote: Boolean = false
        private set

    var overCurrentTicks = -1

    init {
        intakeMotor.restoreFactoryDefaults()
        intakeMotor.setSmartCurrentLimit(40)
        intakeMotor.idleMode = CANSparkBase.IdleMode.kCoast
        intakeMotor.inverted = true
        intakeMotor.burnFlash()

    }

    override fun periodic() {
        SmartDashboard.putNumber("intake current", intakeMotor.outputCurrent)
        if (overCurrentTicks > -1) overCurrentTicks++
        if (overCurrentTicks > 6){
            reset()
        }
//        if ((intakeMotor.outputCurrent > 40.0 && autoIntake) || OI.opCtl.getRawButton(6)) {
//            if (overCurrentTicks < 0) overCurrentTicks = 0
//            autoIntake = false
//            latchState = true
//        }
    }

    fun runIntake(speed: Double){
        intakeMotor.set(speed)
    }

    fun stopIntake(){
        intakeMotor.stopMotor()
    }

    fun reset(){
        overCurrentTicks = -1
    }

}