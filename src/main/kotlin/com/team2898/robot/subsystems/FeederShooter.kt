package com.team2898.robot.subsystems

import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.SpeedController
import edu.wpi.first.wpilibj.Talon

class Robot : TimedRobot() {

    private val xboxController = XboxController(0)  // Assuming the Xbox controller is connected to port 0
    private val motorController: SpeedController = Talon(0)  // Assuming the motor controller is connected to PWM port 0

    override fun teleopPeriodic() {
        // Check if a specific button on the controller is pressed (e.g., A button)
        if (xboxController.getAButton()) {
            startMotor()
        } else {
            stopMotor()
        }
    }

    private fun startMotor() {
        // Code to spin the motor goes here
        motorController.set(1.0)  // Set motor speed to full forward
    }

    private fun stopMotor() {
        // Code to stop the motor goes here
        motorController.stopMotor()
    }
}
