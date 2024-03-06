package frc.engine.utils
// File adapted from 5970's 2024 robot code engine.utils

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkMax

/**
 * Restores the factory defaults for the given spark motor controllers, and sets the current limits
 * @param currentLimit Current limit to set the motors to
 */
fun initMotorControllers(currentLimit : Int, vararg  motors : CANSparkMax){
    motors.forEach {
        it.restoreFactoryDefaults()
        it.setSmartCurrentLimit(currentLimit)
    }
}
/**
 * Restores the factory defaults for the given spark motor controllers,
 * and sets the current limits & idle mode
 * @param currentLimit Current limit to set the motors to
 * @param idle The idle mode to set the controller to
 */
fun initMotorControllers(currentLimit : Int, idle: CANSparkBase.IdleMode, vararg  motors : CANSparkMax){
    motors.forEach {
        it.restoreFactoryDefaults()
        it.setSmartCurrentLimit(currentLimit)
        it.idleMode = idle
    }
}