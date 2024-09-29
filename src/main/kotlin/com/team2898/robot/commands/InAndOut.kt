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

    }

    override fun execute() {
        if (!time.hasElapsed(0.1) && Intake.hasNote) {
            Intake.output = -0.4
        } else {
            if (!time.hasElapsed(0.5)) {
                Intake.output = 0.7
            }
        }
    }

    override fun isFinished(): Boolean {
        return time.hasElapsed(0.5)
    }

    override fun end(interrupted: Boolean) {
        Intake.intake(0.0)
    }
}