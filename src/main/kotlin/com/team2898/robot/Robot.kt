// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package com.team2898.robot

import com.fasterxml.jackson.databind.util.Named
import com.pathplanner.lib.auto.NamedCommands
import com.team2898.robot.commands.*
import com.team2898.robot.subsystems.*
import com.team2898.robot.subsystems.Arm.pos
import edu.wpi.first.cameraserver.CameraServer
import edu.wpi.first.wpilibj.DataLogManager
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
class Robot : TimedRobot() {
    private var m_autonomousCommand: Command? = null
    var autoCommand: Command = InstantCommand({})
    lateinit var robotContainer: RobotContainer
    val commandScheduler = CommandScheduler.getInstance()

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    override fun robotInit() {
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        robotContainer = RobotContainer()

//        CameraServer.startAutomaticCapture()
//        Climber
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     *
     * This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    override fun robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        commandScheduler.run()
    }

    /** This function is called once each time the robot enters Disabled mode.  */
    override fun disabledInit() {
        Arm.setGoal(Constants.ArmConstants.ArmHeights.STOWED.position)
    }
    override fun disabledPeriodic() {}

    /** This autonomous runs the autonomous command selected by your [RobotContainer] class.  */
    override fun autonomousInit() {
        autoCommand = robotContainer.getAutonomousCommand()
        autoCommand.let { autoCommand.schedule() }

    }

    /** This function is called periodically during autonomous.  */
    override fun autonomousPeriodic() {}
    override fun teleopInit() {

        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.

        autoCommand.cancel()



    }

    /** This function is called periodically during operator control.  */
    override fun teleopPeriodic() {

    }
    override fun testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll()
    }

    /** This function is called periodically during test mode.  */
    override fun testPeriodic() {}

    /** This function is called once when the robot is first started up.  */
    override fun simulationInit() {}

    /** This function is called periodically whilst in simulation.  */
    override fun simulationPeriodic() {}
}