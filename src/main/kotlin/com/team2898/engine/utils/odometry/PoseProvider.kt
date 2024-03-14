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


    /** Updates the [pose] variable */
    fun update()

    fun reset(x: Meters, y: Meters, theta: Degrees)
    fun reset(pose2d: Pose2d) {
        reset(pose2d.x.m, pose2d.y.m, pose2d.rotation.degrees.deg)
    }

}