package com.team2898.robot.subsystems

import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import com.team2898.engine.utils.async.Promise
import com.team2898.robot.Constants.ClimberConstants
import com.team2898.robot.RobotMap.ClimberId
import edu.wpi.first.wpilibj.event.BooleanEvent
import edu.wpi.first.wpilibj.event.EventLoop
import edu.wpi.first.wpilibj2.command.SubsystemBase
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min

object Climber : SubsystemBase() {
    private val climberMotor = CANSparkMax(ClimberId, CANSparkLowLevel.MotorType.kBrushless)
    private val climberCoder = climberMotor.encoder
    var currentState: Boolean? = null
        private set
    var targetState: Boolean = false
        private set
    private val waiting = mutableMapOf<Boolean, MutableSet<Promise.Companion.ResolversObject<Unit>>>()
    private var setSpeed = 1.0 // speed control
    fun Boolean.toDouble() = if (this) 1.0 else 0.0
    val distanceToGo get() = targetState.toDouble() - climberCoder.position
    val absDistanceToGo get() = abs(distanceToGo)
    fun finished(loop: EventLoop) = BooleanEvent(loop) { arrived() }
    fun arrived() = currentState != null && currentState == targetState
    fun setState(newState: Boolean) {
        targetState = newState
        currentState = null
    }
    fun go(newState: Boolean): Promise<Unit> {
        waiting.forEach { (t, u) ->
            if (t != newState) {
                u.forEach { it.reject(Exception("Arm movement cancelled")) }
                waiting.remove(t)
            }
        }
        targetState = newState
        currentState = null
        val r = Promise.withResolvers<Unit>()
        try {
            waiting[newState]!!.add(r)
        } catch (_: NullPointerException) { // EAFP: Easier to Ask for Forgiveness than Permission
            waiting[newState] = mutableSetOf(r)
        }
        return r.promise
    }

    override fun periodic() {
        val position = climberCoder.position
        val velocity = climberCoder.velocity
        if (abs(position - targetState.toDouble()) > 0.3) {
            if (abs(velocity) > 0.01) setSpeed = max(0.0, min(1.0, abs(ClimberConstants.ClimberMaxSpeed / velocity)))
            climberMotor.set(setSpeed)
        } else {
            climberMotor.set(0.0)
            currentState = targetState
            waiting[targetState]?.forEach { it.resolve(Unit) }
            waiting.remove(targetState)
        }
    }
}