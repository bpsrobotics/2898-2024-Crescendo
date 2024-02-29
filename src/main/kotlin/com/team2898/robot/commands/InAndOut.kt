package com.team2898.robot.commands

import com.team2898.robot.subsystems.Intake
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command

class InAndOut : Command() {
    private val time = Timer()
    init {
        addRequirements(Intake)
    }

    override fun initialize() {
        time.reset()
        time.start()
        Intake.inAndOut()
    }

    override fun isFinished(): Boolean {
        return time.hasElapsed(1.0)
    }

    override fun end(interrupted: Boolean) {
        Intake.intake(0.0)
    }
}