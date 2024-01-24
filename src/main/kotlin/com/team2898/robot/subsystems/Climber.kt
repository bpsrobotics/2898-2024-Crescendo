package com.team2898.robot.subsystems

import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import com.team2898.robot.Constants.ClimberConstants
import com.team2898.robot.Constants.DriveConstants.kClimberId
import edu.wpi.first.wpilibj.event.BooleanEvent
import edu.wpi.first.wpilibj.event.EventLoop
import edu.wpi.first.wpilibj2.command.SubsystemBase
import java.util.function.BooleanSupplier
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min

object Climber : SubsystemBase() {
    private val climberMotor = CANSparkMax(kClimberId, CANSparkLowLevel.MotorType.kBrushless)
    private val climberCoder = climberMotor.encoder
    var currentState: ClimberConstants.ClimbHeights? = null
        private set
    var targetState: ClimberConstants.ClimbHeights = ClimberConstants.ClimbHeights.STOWED
        private set
    private var setSpeed = 1.0 // speed control
    val distanceToGo get() = targetState.position - climberCoder.position
    val absDistanceToGo get() = abs(distanceToGo)
    fun finished(loop: EventLoop): BooleanEvent {
        return BooleanEvent(loop, ArrivedSignal)
    }
    fun arrived() = currentState == targetState
    object ArrivedSignal : BooleanSupplier {
        override fun getAsBoolean(): Boolean {
            return arrived()
        }
    }
    fun setState(newState: ClimberConstants.ClimbHeights) {
        targetState = newState
        currentState = null
    }

    override fun periodic() {
        val position = climberCoder.position
        val velocity = climberCoder.velocity
        if (abs(position - targetState.position) > 0.3) {
            if (abs(velocity) > 0.01) setSpeed = max(0.0, min(1.0, abs(ClimberConstants.kClimberMaxSpeed / velocity)))
            climberMotor.set(setSpeed)
        } else {
            climberMotor.set(0.0)
            currentState = targetState
        }
    }
}