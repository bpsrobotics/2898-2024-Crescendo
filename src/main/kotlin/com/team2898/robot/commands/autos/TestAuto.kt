package com.team2898.robot.commands.autos

import com.pathplanner.lib.PathPlanner
import com.pathplanner.lib.auto.PIDConstants
import com.pathplanner.lib.auto.SwerveAutoBuilder
import com.pathplanner.lib.commands.PPSwerveControllerCommand

import com.team2898.robot.Constants
import com.team2898.robot.subsystems.Drivetrain
import com.team2898.robot.subsystems.Odometry
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup

class TestAuto : CommandBase() {
    private lateinit var autoCommandGroup: Command
    val path = PathPlanner.loadPath("Back1M", 0.2,1.0)
    override fun initialize() {
        autoCommandGroup = SequentialCommandGroup(
            PPSwerveControllerCommand(
                path,
                Odometry.poseSupplier,
                PIDController(Constants.ModuleConstants.kDrivingP, Constants.ModuleConstants.kDrivingI, Constants.ModuleConstants.kDrivingD),  // Translation PID constants
                PIDController(Constants.ModuleConstants.kDrivingP, Constants.ModuleConstants.kDrivingI, Constants.ModuleConstants.kDrivingD),  // Translation PID constants
                PIDController(Constants.ModuleConstants.kTurningP, Constants.ModuleConstants.kTurningI, Constants.ModuleConstants.kTurningD),  // Rotation PID constants
                Drivetrain.driveConsumer,
                true,
                Drivetrain
            ).andThen(InstantCommand({Drivetrain.drive(0.0,0.0,0.0,true,true)}))



        )
        autoCommandGroup.schedule()
    }

    override fun isFinished(): Boolean {
        return autoCommandGroup.isFinished
    }

    override fun end(interrupted: Boolean) {
        println("ENDED")
        autoCommandGroup.cancel()
    }
}
