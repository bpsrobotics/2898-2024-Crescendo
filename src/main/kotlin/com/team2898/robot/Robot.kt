// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package com.team2898.robot

import com.team2898.robot.commands.TeleOp
import com.team2898.robot.subsystems.Arm
import com.team2898.robot.subsystems.Drivetrain
import com.team2898.robot.subsystems.Intake
import com.team2898.robot.subsystems.Shooter
import edu.wpi.first.wpilibj.TimedRobot
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
    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    override fun robotInit() {
        Drivetrain
        Arm
        Shooter
        Intake
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        robotContainer = RobotContainer()
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
        CommandScheduler.getInstance().run()
    }

    /** This function is called once each time the robot enters Disabled mode.  */
    override fun disabledInit() {}
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

        if (m_autonomousCommand != null) {
            m_autonomousCommand!!.cancel()
        }
        TeleOp().schedule()

    }

    /** This function is called periodically during operator control.  */
    override fun teleopPeriodic() {
//        if (OI.driverX && OI.alignmentPad == OI.Direction.UP) {
//            Drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward).schedule()
//            println("quasi forward")
//        } else if (OI.driverX && OI.alignmentPad == OI.Direction.DOWN) {
//            Drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse).schedule()
//            println("quasi reverse")
//        } else if (OI.driverY && OI.alignmentPad == OI.Direction.UP){
//            Drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward).schedule()
//            println("dynamic forward")
//        } else if (OI.driverY && OI.alignmentPad == OI.Direction.DOWN){
//            Drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse).schedule()
//            print("dynamic reverse")
//        } else {
//            println("not doing")
//        }
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