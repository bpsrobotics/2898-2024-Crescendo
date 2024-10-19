package com.team2898.robot.subsystems

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import com.team2898.robot.Constants
import com.team2898.robot.Constants.IntakeConstants.STOP_BUFFER
import com.team2898.robot.RobotMap.IntakeBeamBreak

import com.team2898.robot.RobotMap.IntakeId
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.filter.LinearFilter
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.engine.utils.initMotorControllers
import kotlin.math.sign

object Intake : SubsystemBase() {
    private val intakeMotor = CANSparkMax(IntakeId, CANSparkLowLevel.MotorType.kBrushed)
    private val beamBreak = DigitalInput(IntakeBeamBreak)
    var output = 0.0
    val currentFilter = LinearFilter.movingAverage(20)
    var currentAverage = 0.0
    val buffer = Debouncer(0.1, Debouncer.DebounceType.kRising)
    val bufferTimer = Timer()
    val hasNote get() = beamBreak.get()
    val intakeState get() = bufferTimer.hasElapsed(STOP_BUFFER)
    val gracePeriod get() = !bufferTimer.hasElapsed(STOP_BUFFER + 5.0)

    init {
        initMotorControllers(Constants.IntakeConstants.CURRENT_LIMIT, CANSparkBase.IdleMode.kBrake, intakeMotor)
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
        SmartDashboard.putBoolean("beam break", beamBreak.get())
        currentAverage = currentFilter.calculate(intakeMotor.outputCurrent)

        intakeMotor.set(output)
    }

    fun intake(speed: Double){
        if (!intakeState){
            output = 0.0
            return
        }
        if (buffer.calculate(hasNote) && !gracePeriod) {
            output = 0.0
            bufferTimer.restart()
            return
        }
        if (gracePeriod && hasNote) {
            output = speed
        } else {
            output = speed
        }


    }

    fun outtake() {
        output = -0.4
    }



}