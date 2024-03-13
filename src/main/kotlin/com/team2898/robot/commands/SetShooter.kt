package com.team2898.robot.commands

import com.team2898.robot.subsystems.Intake
import com.team2898.robot.subsystems.Shooter
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command

class SetShooter : Command() {
    private val time = Timer()
    init {
        addRequirements(Shooter)
    }

    override fun initialize() {
        time.reset()
        time.start()
        Shooter.setVoltage(7.0)
        println("shooter")
    }

    override fun isFinished(): Boolean {
        return time.hasElapsed(1.0) || Shooter.wheelSpeedTop == 2000.0
    }

    override fun end(interrupted: Boolean) {
        Shooter.setVoltage(0.0)
    }
}