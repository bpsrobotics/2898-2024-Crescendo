package com.team2898.robot.commands

import com.team2898.robot.subsystems.Drivetrain
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine

class SysIdQuasistatic(private val direction: SysIdRoutine.Direction): Command() {
    override fun initialize() {
        Drivetrain.sysIdRoutine.quasistatic(direction)
    }

}