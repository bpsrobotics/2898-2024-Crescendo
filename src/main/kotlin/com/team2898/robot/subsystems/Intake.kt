package com.team2898.robot.subsystems

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import com.team2898.robot.Constants.IntakeConstants.STOP_BUFFER
import com.team2898.robot.OI

import com.team2898.robot.RobotMap.IntakeId
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.filter.LinearFilter
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase

object Intake : SubsystemBase() {
    private val intakeMotor = CANSparkMax(IntakeId, CANSparkLowLevel.MotorType.kBrushed)
    var hasNote = false
    var output = 0.0
    val currentFilter = LinearFilter.movingAverage(15)
    var currentAverage = 0.0
    val buffer = Debouncer(0.04, Debouncer.DebounceType.kRising)
    val bufferTimer = Timer()
    val intakeState get() = bufferTimer.hasElapsed(STOP_BUFFER)

    init {
        intakeMotor.restoreFactoryDefaults()
        intakeMotor.setSmartCurrentLimit(20)
        intakeMotor.idleMode = CANSparkBase.IdleMode.kBrake
        intakeMotor.inverted = true
        intakeMotor.burnFlash()
        bufferTimer.start()
    }

    override fun periodic() {
        SmartDashboard.putNumber("intake current", intakeMotor.outputCurrent)
        SmartDashboard.putBoolean("has note", hasNote)
        SmartDashboard.putNumber("intake output", output)
        SmartDashboard.putNumber("current average", currentAverage)
        SmartDashboard.putNumber("intake timer ", bufferTimer.get())
        currentAverage = currentFilter.calculate(intakeMotor.outputCurrent)
//        if (buffer.calculate(currentAverage > 10.0) && !hasNote) {
//            output = 0.0
//            hasNote = true
//        }
//        if (buffer.calculate(currentAverage < 10.0 && currentAverage > 3.0)) {
//            hasNote = false
//        }
        intakeMotor.set(output)
    }

    fun intake(speed: Double){
        if (intakeState) {
            if (buffer.calculate(currentAverage > 8.0) && !hasNote) {
                output = 0.0
                bufferTimer.reset()
                bufferTimer.start()
                hasNote = true
            } else {
                output = speed
                hasNote = false
            }
        } else {
            println("stopping intake")
            output = 0.0
        }
    }



}