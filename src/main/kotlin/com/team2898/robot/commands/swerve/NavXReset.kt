package com.team2898.robot.commands.swerve

import com.team2898.robot.OI
import com.team2898.robot.subsystems.Arm
import com.team2898.robot.subsystems.NavX
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command

class NavXReset: Command() {
    private val time = Timer()
    override fun initialize() {
        time.reset()
        time.start()
        NavX.reset()
        OI.Rumble.set(0.25,1.0, GenericHID.RumbleType.kRightRumble)
    }

    override fun isFinished(): Boolean {
        return time.hasElapsed(0.1)
    }
}