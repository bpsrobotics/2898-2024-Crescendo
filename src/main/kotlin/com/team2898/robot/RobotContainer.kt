// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package com.team2898.robot

//import com.team2898.robot.Constants.OperatorConstants

import com.team2898.robot.Constants.AutoConstants.commandMap
import com.team2898.robot.commands.autos.TestAuto
import com.team2898.robot.commands.autos.TestAuto2
import com.team2898.robot.subsystems.Drivetrain.drive
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.Trigger


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
    // The robot's subsystems and commands are defined here...
    //private val m_exampleSubsystem = ExampleSubsystem()
    // Replace with CommandPS4Controller or CommandJoystick if needed
    private val m_driverController = CommandXboxController(0)

    private var autoCommandChooser: SendableChooser<Command> = SendableChooser()


    /** The container for the robot. Contains subsystems, OI devices, and commands.  */
    init {
        // Configure the trigger bindings
        configureBindings()
        autoCommandChooser.addOption("test auto", TestAuto())
        autoCommandChooser.setDefaultOption("testAuto2", TestAuto2())

        SmartDashboard.putData("Auto mode", autoCommandChooser)


    }
    fun getAutonomousCommand(): Command{
        return autoCommandChooser.selected
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * [Trigger.Trigger] constructor with an arbitrary
     * predicate, or via the named factories in [ ]'s subclasses for [ ]/[ PS4][edu.wpi.first.wpilibj2.command.button.CommandPS4Controller] controllers or [Flight][edu.wpi.first.wpilibj2.command.button.CommandJoystick].
     */
    private fun configureBindings() {
        // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
        //Trigger { m_exampleSubsystem.exampleCondition() }
        //        .onTrue(ExampleCommand(m_exampleSubsystem))

        // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
        // cancelling on release.
        //m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand())
        // guard for bot-on-board
        commandMap.put(
            "start",
            SequentialCommandGroup(PrintCommand("***Path Start"),
            InstantCommand({ println("path start") })
            )
        )
        commandMap.put(
            "middle",
            SequentialCommandGroup(PrintCommand("***Path Middle"))
        )
        commandMap.put(
            "end", SequentialCommandGroup(
                PrintCommand("***Path End"),
                InstantCommand({drive(0.0,0.0,0.0,false,false)})
            )
        )
        commandMap.put(
            "score", SequentialCommandGroup(
                PrintCommand("***Path score"),
            )
        )
    }

     /**
         * Use this to pass the autonomous command to the main [Robot] class.
         *
         * @return the command to run in autonomous
         */
        // An example command will be run in autonomous

            //Autos.exampleAuto(m_exampleSubsystem)
}