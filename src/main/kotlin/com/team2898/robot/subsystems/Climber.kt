package com.team2898.robot.subsystems

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import com.team2898.robot.Constants.ClimberConstants.STALL_CURRENT
import com.team2898.robot.RobotMap.ClimbPrimaryId
import com.team2898.robot.RobotMap.ClimbSecondaryId
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase


object Climber : SubsystemBase() {
    private val climbMotor = CANSparkMax(ClimbPrimaryId, CANSparkLowLevel.MotorType.kBrushed)
    private val climbMotorSecondary = CANSparkMax(ClimbSecondaryId, CANSparkLowLevel.MotorType.kBrushed)

    var output = 0.0
    val buffer = Debouncer(0.2, Debouncer.DebounceType.kRising)
    var stalled = false

    val stallTimer = Timer()


    init {
        SmartDashboard.putNumber("output climb", output)
        climbMotor.restoreFactoryDefaults()
        climbMotor.setSmartCurrentLimit(15)
        climbMotor.idleMode = CANSparkBase.IdleMode.kBrake
        climbMotor.burnFlash()

        climbMotorSecondary.restoreFactoryDefaults()
        climbMotorSecondary.setSmartCurrentLimit(15)
        climbMotorSecondary.idleMode = CANSparkBase.IdleMode.kBrake
        climbMotorSecondary.follow(climbMotor)
        climbMotor.burnFlash()
        stallTimer.reset()
        stallTimer.start()
    }

    override fun periodic() {
//        output = SmartDashboard.getNumber("output climb", output)
        SmartDashboard.putBoolean("Climb down", !stalled)
//        if (output > 0.0) {
//            release()
//        } else {
//            brake()
//        }
        setVoltage(output)
        println("output $output")
        println("current " + climbMotor.outputCurrent)
    }

    fun setSpeed(input: Double) {
//        if (stallTimer.hasElapsed(1.0)) {
//            if (!stalled) {
//                if (buffer.calculate(climbMotor.outputCurrent > STALL_CURRENT)) {
//                    output = 0.0
//                    stalled = true
//                } else {
//                    output = input
//                }
//            } else {
//                output = 0.0
//                stallTimer.reset()
//                stallTimer.start()
//            }
//        } else {
//            output = 0.0
//        }
        output = input

    }
    fun setVoltage(input: Double){
        climbMotor.setVoltage(input)
    }
    fun release() {
        climbMotor.idleMode = CANSparkBase.IdleMode.kCoast
        climbMotorSecondary.idleMode = CANSparkBase.IdleMode.kCoast
    }
    fun brake() {
        climbMotor.idleMode = CANSparkBase.IdleMode.kBrake
        climbMotorSecondary.idleMode = CANSparkBase.IdleMode.kBrake
    }


}