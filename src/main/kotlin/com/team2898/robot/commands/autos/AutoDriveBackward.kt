package com.team2898.robot.commands.autos

import com.team2898.robot.subsystems.Drivetrain
import com.team2898.robot.subsystems.Odometry
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandBase

class AutoDriveBackward : Command(){
    private lateinit var autoCommandGroup: Command
    val timer = Timer()
    override fun initialize() {
//        SequentialCommandGroup(SuperSimpleAuto(), SimpleBalance())
        timer.start()
        timer.reset()
        Odometry.resetOdometry(Pose2d())
    }
    override fun execute() {
        println("Command executed: AutoDriveBackward")
        Drivetrain.drive(0.0, -0.25, 0.0, true, true)
    }
    override fun isFinished(): Boolean {
        return timer.hasElapsed(2.4) || Odometry.SwerveOdometry.poseMeters.y < 1
    }

}