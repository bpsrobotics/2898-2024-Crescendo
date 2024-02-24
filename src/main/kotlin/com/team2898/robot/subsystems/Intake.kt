package com.team2898.robot.subsystems

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import com.team2898.robot.Constants.IntakeConstants.STOP_BUFFER

import com.team2898.robot.RobotMap.IntakeId
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase

object Intake : SubsystemBase() {
    private val intakeMotor = CANSparkMax(IntakeId, CANSparkLowLevel.MotorType.kBrushed)
    var hasNote = false
    var overCurrentTicks = -2
    var stopTimer = Timer()
    var stopCount = STOP_BUFFER

    init {
        intakeMotor.restoreFactoryDefaults()
        intakeMotor.setSmartCurrentLimit(20)
        intakeMotor.idleMode = CANSparkBase.IdleMode.kBrake
        intakeMotor.inverted = true
        intakeMotor.burnFlash()

    }

    override fun periodic() {
        SmartDashboard.putNumber("intake current", intakeMotor.outputCurrent)
        SmartDashboard.putBoolean("has note", hasNote)
        if (overCurrentTicks > -1) overCurrentTicks++
        if (overCurrentTicks > 6){
            reset()
            hasNote = false
        }
        if (intakeMotor.outputCurrent > 15.0 && overCurrentTicks < 0) {
            overCurrentTicks++
            hasNote = true
            stopTimer.reset()
            stopTimer.start()
            stopCount = stopTimer.get()
        }
        if (stopCount < STOP_BUFFER && hasNote){
            println("stopping intake")
            stopIntake()
        }
    }

    fun runIntake(speed: Double){
        intakeMotor.set(speed)
    }

    fun stopIntake(){
        intakeMotor.stopMotor()
    }

    fun reset(){
        overCurrentTicks = -2
    }

}