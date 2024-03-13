package com.team2898.test

import edu.wpi.first.math.controller.PIDController

object playground {

}
fun main() {
    val pid = PIDController(4.0, 0.0, 0.01)

    println(pid.calculate(0.9, 0.0))
}