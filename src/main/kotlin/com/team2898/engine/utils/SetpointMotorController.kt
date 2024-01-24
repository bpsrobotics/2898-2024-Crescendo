package com.team2898.engine.utils

import com.ctre.phoenix6.hardware.CANcoder
import com.revrobotics.CANSparkMax
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.Timer
import kotlin.math.*

// may need renaming (only used by Climber, should be pretty easy)
class SetpointMotorController(
    val motor: CANSparkMax,
    val encoder: CANcoder,
    maxSpeed: Double,
    acceleration: Double,
    val lowSoftStop: Double,
    val highSoftStop: Double,
    p: Double = 0.0,
    i: Double = 0.0,
    d: Double = 0.0,
    val kv: Double = 0.0,
    val ks: Double = 0.0,
    val ksin: Double = 0.0
) {
    var setpoint: Double = pos()
    fun pos(): Double {
        return encoder.absolutePosition.valueAsDouble * 2 * PI
    }
    val movingAverage = MovingAverage(15)
    val profileTimer = Timer()
    val constraints = TrapezoidProfile.Constraints(maxSpeed, acceleration)
    var profile: TrapezoidProfile? = null
    val integral = MovingAverage(50)
    var last = pos()
    val timer = Timer()
    val pid = PIDController(p, i, d)
    fun tick() {
        val currentTick = false

        val p = pos()
        val dp = p - last
        last = p
        val dt = timer.get()
        timer.start()
        timer.reset()
        movingAverage.add(dp / dt)
        val rate = movingAverage.average

        integral.add((rate - motor.encoder.velocity).absoluteValue)

        if (setpoint == 0.0 || setpoint !in lowSoftStop..highSoftStop || ((p - setpoint).absoluteValue < 0.05 && rate.absoluteValue < 0.1) || profileTimer.get() > (profile?.totalTime() ?: 0.0)) {
            profile = null
        }
//        val targetSpeed = profile?.calculate(profileTimer.get())?.velocity ?: 0.0
//        val targetSpeed = profile.calculate(profileTimer.get(), )
        val targetSpeed = profile?.calculate(profileTimer.get(),
            TrapezoidProfile.State(pos(), movingAverage.average),
            TrapezoidProfile.State(pos(), 0.0)
        )?.velocity ?: 0.0

        var output = pid.calculate(rate, targetSpeed)
        output += kv * targetSpeed
        output += ks + sin(p) * ksin

        if (p > highSoftStop) {
            output = output.coerceAtMost(0.0)
            println("UPPER SOFT STOP")
        } else if (p < lowSoftStop || currentTick) {
            output = output.coerceAtLeast(0.0)
            println("LOWER SOFT STOP")
        }
        motor.set(output)
    }
    fun setGoal(newPos: Double) {
        // clamp to soft-stop range
        setpoint = min(highSoftStop, max(lowSoftStop, newPos))

        profile?.calculate(profileTimer.get(),
            TrapezoidProfile.State(pos(), movingAverage.average),
            TrapezoidProfile.State(newPos, 0.0)
        )
        profileTimer.reset()
        profileTimer.start()
    }
}