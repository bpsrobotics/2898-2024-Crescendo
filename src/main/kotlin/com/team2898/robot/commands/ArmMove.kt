package com.team2898.robot.commands

import com.team2898.robot.Constants
import com.team2898.robot.subsystems.Arm
import edu.wpi.first.wpilibj2.command.Command

/**
 * Moves arm to given height
 * @property goal Height to set arm to
 * @author Ori
 */
class ArmMove(private val goal: Double) : Command() {

    init {
        addRequirements(Arm)
    }
    constructor(goal: Constants.ArmConstants.ArmHeights) : this(goal.position)

    override fun initialize() {
        Arm.setGoal(goal)
        println("trying to $goal")
    }

    override fun isFinished(): Boolean {
        return !Arm.isMoving()
    }
}
