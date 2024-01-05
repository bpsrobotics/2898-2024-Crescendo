package com.team2898.engine.utils

import com.team2898.engine.utils.Sugar.circleNormalize
import com.team2898.engine.utils.Sugar.clamp
import edu.wpi.first.wpilibj.Timer
import kotlin.math.PI
import kotlin.math.absoluteValue
import kotlin.math.sign

/**
 * PID Controller used for motors rotating to a specific angle
 * @author Anthony, Mike
 */
@Suppress("MemberVisibilityCanBePrivate")
class TurningPID(var kP: Double, var kD: Double) {
    var setPoint = 0.0
    var timePrevious = Timer.getFPGATimestamp()
    var previousError = 0.0

    companion object {
        /** Finds the smallest distance between [angleA] and [angleB] by wrapping around the circle in radians **/
        fun minCircleDist(angleA: Double, angleB: Double): Double{
            val normal = angleB - angleA
            val wrap = -((2 * PI) * normal.sign - normal)
//            println("normal: ${normal} wrap: ${wrap} sign: ${normal.sign} angleA: $angleA angleB: $angleB")
            val circleDistance = if (normal.absoluteValue < wrap.absoluteValue) {
                normal
            } else{
                wrap
            }
            return circleDistance
        }
    }

    /**
     * @return Voltage that should be given to the motor to reach [setPoint] given [sensorValue], [kP] and [kD]
     */
    fun motorOutput(sensorValue: Double): Double {
        val timeNow = Timer.getFPGATimestamp()


        val timeDif = timeNow - timePrevious
        val useSetPoint = setPoint.circleNormalize()
        val error = minCircleDist(sensorValue, useSetPoint)
        val errorDif = error - previousError

        val derivative = kD * ((errorDif) / (timeDif))


        previousError = error
        timePrevious = Timer.getFPGATimestamp()
        return error * kP + derivative
    }
    fun turnspeedOutput(sensorValue: Double): Double {
        val timeNow = Timer.getFPGATimestamp()


        val timeDif = timeNow - timePrevious
        var useSetPoint = setPoint.circleNormalize()
        val error = minCircleDist(sensorValue, useSetPoint)/(2* PI)
        val errorDif = error - previousError

        val derivative = kD * ((errorDif) / (timeDif))


        previousError = error
        timePrevious = Timer.getFPGATimestamp()
        return error * kP + derivative
    }
    fun turnspeedOutputNoNormalize(sensorValue: Double): Double {
        val timeNow = Timer.getFPGATimestamp()


        val timeDif = timeNow - timePrevious
        var useSetPoint = setPoint

        val error = ((useSetPoint - sensorValue)/(2* PI)).clamp(-1.0,1.0)
        val errorDif = error - previousError

        val derivative = kD * ((errorDif) / (timeDif))


        previousError = error
        timePrevious = Timer.getFPGATimestamp()
        return error * kP + derivative
    }
}

//fun main() {
//    println(TurningPID.minCircleDist(4 * 2 * PI, 2 * PI))
//}
