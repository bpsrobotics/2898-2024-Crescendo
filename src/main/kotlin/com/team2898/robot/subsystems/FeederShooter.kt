package com.team2898.robot.commands

//The commands for both the driver and operator
//import com.com.engine.utils.`M/s`
//import com.team2898.robot.Constants.ArmHeights.*
//import com.team2898.robot.Field
//import com.team2898.robot.OI
//import com.team2898.robot.subsystems.Arm

import com.team2898.engine.utils.Sugar.degreesToRadians
import com.team2898.engine.utils.Sugar.eqEpsilon
import com.team2898.engine.utils.TurningPID
import com.team2898.robot.Constants
import com.team2898.robot.OI
import com.team2898.robot.subsystems.Drivetrain
import com.team2898.robot.subsystems.NavX
import com.team2898.robot.subsystems.Odometry
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.pow
import kotlin.math.sign
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.util.HolonomicPathFollowerConfig
import com.pathplanner.lib.util.PIDConstants
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.util.HolonomicPathFollowerConfig
import com.pathplanner.lib.util.PIDConstants
import com.pathplanner.lib.util.ReplanningConfig
import com.team2898.engine.utils.SwerveUtils
import com.team2898.robot.Constants
import com.team2898.robot.Constants.DriveConstants
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveDriveOdometry
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.util.WPIUtilJNI
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import java.util.function.BooleanSupplier
import com.pathplanner.lib.util.ReplanningConfig
import com.team2898.engine.utils.SwerveUtils
import com.team2898.robot.Constants
import com.team2898.robot.Constants.DriveConstants
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveDriveOdometry
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.util.WPIUtilJNI
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import java.util.function.BooleanSupplier


// Assuming your app package is com.example.motorcontrol

package com.example.motorcontrol.ui

import android.os.Bundle
import android.view.KeyEvent
import android.view.MotionEvent
import android.view.View
import android.widget.Button
import androidx.appcompat.app.AppCompatActivity
import com.example.motorcontrol.R
import com.example.motorcontrol.hardware.MotorClass

class MotorControlActivity : AppCompatActivity(), View.OnGenericMotionListener, View.OnKeyListener {

    private val motor = MotorClass()

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_motor_control)

        val buttonStart: Button = findViewById(R.id.buttonStart)

        // Set listeners for button press events
        buttonStart.setOnGenericMotionListener(this)
        buttonStart.setOnKeyListener(this)
    }

    private fun startMotor() {
        motor.spin()
    }

    // Handle controller button events
    override fun onGenericMotion(v: View?, event: MotionEvent?): Boolean {
        // Check if the event is a button press
        if (event?.action == MotionEvent.ACTION_BUTTON_PRESS) {
            startMotor()
            return true
        }
        return false
    }

    override fun onKey(v: View?, keyCode: Int, event: KeyEvent?): Boolean {
        // Check if the event is a key press
        if (event?.action == KeyEvent.ACTION_DOWN) {
            startMotor()
            return true
        }
        return false
    }
}
