package com.team2898.robot.subsystems


import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.team2898.robot.Constants
import com.team2898.robot.Constants.DriveConstants.kFeddderCanId
import com.team2898.robot.Constants.DriveConstants.kFlywheelCanId
import com.team2898.robot.Constants.ShooterConstants.kBaseFlywheelVoltage
import com.team2898.robot.Constants.ShooterConstants.kFeederDelay
import com.team2898.robot.Constants.ShooterConstants.kFeederVoltage
import com.team2898.robot.Constants.ShooterConstants.kMaxFlywheelVoltage
import com.team2898.robot.OI
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.SubsystemBase

object Shooter : SubsystemBase() {

    private val flywheelController = CANSparkMax(kFlywheelCanId, CANSparkMaxLowLevel.MotorType.kBrushless)
    private val feederController = CANSparkMax(kFeddderCanId, CANSparkMaxLowLevel.MotorType.kBrushless)
    private val feederTimer = Timer()
    private var timeLastFed = -1.0
    private var feedBall = false

    init{
        flywheelController.setSmartCurrentLimit(Constants.ShooterConstants.kFlywheelCurrent)
    }
    private fun HandleFlywheel(){
        println(OI.operatorThrottle)
        when {
            OI.operatorThrottle > 0.1 -> {
                val throttle = (OI.operatorThrottle - 0.1) * (1 / 0.9)
                val maxThrottle = kMaxFlywheelVoltage - kBaseFlywheelVoltage
                flywheelController.setVoltage(kBaseFlywheelVoltage + (throttle * maxThrottle))
                println(kBaseFlywheelVoltage + (throttle * maxThrottle))
            }

            OI.operatorThrottle < -0.1 -> {
                flywheelController.setVoltage(0.0)
            }

            else -> {
                flywheelController.setVoltage(kBaseFlywheelVoltage)
            }
        }
    }
    private fun HandleFeeder(){
        if(OI.operatorTrigger){
            feederTimer.start()
            if(timeLastFed == -1.0){
                feederTimer.reset()
                feederTimer.start()
            }
            StartFeeder_Time()
            feedBall = true


        }
        else {
            feederTimer.stop()
            timeLastFed = -1.0
        }
    }
    private val feedingTime = Timer()
    fun StartFeeder_Time(){
        feedingTime.reset()
        feedingTime.start()
    }
    fun RunFeeder_Time(){

        if(feedingTime.hasElapsed(0.25)){
            feedingTime.stop()
            feedBall = false
            feederController.setVoltage(0.0)
            return
        }
        feederController.setVoltage(-kFeederVoltage)
    }

    fun RunFeeder_Resistance(){

    }
    override fun periodic() {
        HandleFlywheel()
        HandleFeeder()
        if(feedBall) RunFeeder_Time()
    }


}