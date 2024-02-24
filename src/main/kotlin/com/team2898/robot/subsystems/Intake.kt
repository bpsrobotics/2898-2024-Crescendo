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
    var overCurrentTicks = -1.0
    var stopTimer = Timer()
    var output = 0.0

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
        SmartDashboard.putNumber("intake output", output)
        if (overCurrentTicks > -1.0) overCurrentTicks++
        if (overCurrentTicks > 6.0){
            reset()
            hasNote = false
        }
        SmartDashboard.putNumber("overcurrentTicks", overCurrentTicks)
        if (intakeMotor.outputCurrent > 11.0 && overCurrentTicks < 0) {
            overCurrentTicks++
            output = 0.0
            hasNote = true
            stopTimer.reset()
            stopTimer.start()
        }
//        if (stopTimer.get() < STOP_BUFFER && hasNote){
//            println("stopping intake")
//            output = 0.0
//        }
        intakeMotor.set(output)
    }

    fun runIntake(speed: Double){
        output = speed
    }

    fun stopIntake(){
        intakeMotor.stopMotor()
    }

    fun reset(){
        overCurrentTicks = -1.0
    }

}