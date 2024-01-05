package com.team2898.robot.commands.autos

import com.team2898.robot.subsystems.ToteGrabber
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandBase

class ActuateToteGrabber : CommandBase(){
    private lateinit var autoCommandGroup: Command
    val timer = Timer()
    override fun initialize() {
//        SequentialCommandGroup(SuperSimpleAuto(), SimpleBalance())
        timer.reset()
        timer.start()
        ToteGrabber.actuate(true)
    }
    override fun isFinished(): Boolean {
        return timer.hasElapsed(1.0)
    }
}