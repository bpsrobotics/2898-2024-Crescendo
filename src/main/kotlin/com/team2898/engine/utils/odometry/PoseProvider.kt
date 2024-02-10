package com.team2898.engine.utils.odometry

import com.team2898.engine.utils.units.Degrees
import com.team2898.engine.utils.units.Meters
import com.team2898.engine.utils.units.deg
import com.team2898.engine.utils.units.m
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.util.sendable.SendableRegistry

/** Provides an orientation and position for the robot */
interface PoseProvider : Sendable {
    /** Provides the pose as a WPIlib Pose2D */
    val pose: Pose2d

    /** Updates the [pose] variable */
    fun update()

    fun reset(x: Meters, y: Meters, theta: Degrees)
    fun reset(pose2d: Pose2d) {
        reset(pose2d.x.m, pose2d.y.m, pose2d.rotation.degrees.deg)
    }
    override fun initSendable(builder: SendableBuilder) {
        SendableRegistry.setName(this, toString())
        builder.addDoubleProperty("x", { pose.x }, null)
        builder.addDoubleProperty("y", { pose.y }, null)
        builder.addDoubleProperty("rotation", { pose.rotation.radians }, null)
    }
}