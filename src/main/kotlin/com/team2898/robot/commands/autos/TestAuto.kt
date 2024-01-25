package com.team2898.robot.commands.autos


import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.commands.FollowPathWithEvents

import com.pathplanner.lib.path.PathPlannerPath

import com.team2898.robot.Constants
import com.team2898.robot.subsystems.Drivetrain
import com.team2898.robot.subsystems.Odometry
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup

class TestAuto : Command() {
    private lateinit var autoCommandGroup: Command
    val path = PathPlannerPath.fromPathFile("Practice")
    override fun initialize() {
        AutoBuilder.followPath(path)
    }

    override fun isFinished(): Boolean {
        return autoCommandGroup.isFinished
    }

    override fun end(interrupted: Boolean) {
        println("ENDED")
        autoCommandGroup.cancel()
    }
}
