package com.team2898.robot.subsystems
import com.kauailabs.navx.frc.AHRS
import com.team2898.engine.utils.Sugar.degreesToRadians
import com.team2898.engine.utils.Sugar.radiansToDegrees
import com.team2898.engine.utils.TurningPID
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase

/** Container object for an instantiated NavX class, as well as other functions relating to the gyroscope */
object NavX : SubsystemBase() {
    /** Gyroscope used on robot */
    var navx = AHRS()
    var totalRotation = 0.0
    private var lastRotation = 0.0
    var rotationalSpeed = 0.0

    /** @return The NavX's angle multiplied by -1 */
    fun getInvertedAngle(): Double{
        return -navx.angle
    }
    fun getAngle(): Double {
        return navx.angle
//        .plus(90.0).mod(360.0)
    }
    fun update(timeSinceUpdate: Double){
        navx.angleAdjustment
        totalRotation += TurningPID.minCircleDist(navx.angle.degreesToRadians(), lastRotation.degreesToRadians()).radiansToDegrees()
        rotationalSpeed = TurningPID.minCircleDist(navx.angle.degreesToRadians(), lastRotation.degreesToRadians()).radiansToDegrees()/timeSinceUpdate
        lastRotation = navx.angle
        SmartDashboard.putNumber("Odometry/TotalRotation", totalRotation)
        SmartDashboard.putNumber("Odometry/Rotation", -navx.angle)

    }
    fun reset(){
        navx.reset()
        navx.angleAdjustment = -90.0
        totalRotation = 0.0
    }
}