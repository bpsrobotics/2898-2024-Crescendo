package com.team2898.robot.subsystems

import com.bpsrobotics.engine.utils.Degrees
import com.bpsrobotics.engine.utils.Meters
import com.team2898.engine.utils.odometry.PoseProvider
import com.team2898.engine.utils.odometry.Vision
import com.team2898.robot.Constants
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.MatBuilder
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.Nat
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.Odometry
import edu.wpi.first.math.kinematics.SwerveDriveOdometry
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.util.sendable.SendableRegistry
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.photonvision.PhotonPoseEstimator
import org.photonvision.PhotonPoseEstimator.PoseStrategy
import java.util.function.Supplier


object Odometry : SubsystemBase(), PoseProvider {
    private val vision = Vision("Camera_Module_v1")
    val SwerveOdometry = SwerveDrivePoseEstimator(
        Constants.DriveConstants.kDriveKinematics,
        Rotation2d.fromDegrees(NavX.getInvertedAngle()), arrayOf(
            Drivetrain.m_frontLeft.position,
            Drivetrain.m_frontRight.position,
            Drivetrain.m_rearLeft.position,
            Drivetrain.m_rearRight.position
        ),
        Pose2d(Translation2d(0.0,0.0),Rotation2d(0.0,0.0))

    )

    override val pose: Pose2d
        get() = SwerveOdometry.estimatedPosition
    var poseSupplier: Supplier<Pose2d> = Supplier {pose}
    fun supplyPose(): Pose2d {
        return Pose2d(pose.x, pose.y, pose.rotation)
    }

    val chassisSpeeds: ChassisSpeeds
        get() = Constants.DriveConstants.kDriveKinematics.toChassisSpeeds(Drivetrain.m_frontLeft.state, Drivetrain.m_frontRight.state, Drivetrain.m_rearLeft.state, Drivetrain.m_rearRight.state)
    val chassisSpeedsConsumer = {
        x: ChassisSpeeds -> chassisSpeeds
        Unit
    }
    val chassisSpeedsSupplier: Supplier<ChassisSpeeds> = Supplier{ chassisSpeeds }




    /** Robot rotation speed in m/s */
    var velocity: Translation2d = Translation2d()
    private var lastPose = Pose2d()
    private val timer = Timer()

    fun zero(){
        reset(Pose2d(0.0,0.0,Rotation2d.fromDegrees(0.0)))
    }

    fun setpoint(p: Pose2d) {
        reset(p)
    }

    init {
        timer.start()
        zero()
    }
    override fun reset(x: Meters, y: Meters, theta: Degrees) {
        val p = Pose2d(x.value, y.value, Rotation2d.fromDegrees(theta.value))
    }
    override fun periodic(){
        update()
    }
    override fun update(){
        var currentVisionValues = vision.getEstimatedPose(pose)
        if (currentVisionValues!= null) {
            if(currentVisionValues.isPresent){
                val getCurrentVisionValues = currentVisionValues.get()
                val stdDevs: Matrix<N3, N1> = Matrix(Nat.N3(), Nat.N1())
                stdDevs.fill(10.0)
                //stdDevs.set(1,3,1000.0)
                SwerveOdometry.setVisionMeasurementStdDevs(stdDevs) //TODO figure out STDev method
                SwerveOdometry.addVisionMeasurement(
                    getCurrentVisionValues.estimatedPose.toPose2d(),
                    getCurrentVisionValues.timestampSeconds
                )//TODO Check timestamp seconds is in synch with robot code
            }

        }
        NavX.update(timer.get())
        SwerveOdometry.update(
            Rotation2d.fromDegrees(NavX.getInvertedAngle()), arrayOf(
                Drivetrain.m_frontLeft.position,
                Drivetrain.m_frontRight.position,
                Drivetrain.m_rearLeft.position,
                Drivetrain.m_rearRight.position
            ))
        velocity = Translation2d((lastPose.x - pose.x)/timer.get(), (lastPose.y - pose.y)/timer.get())
        lastPose = pose
        SmartDashboard.putNumber("Odometry/FieldX", pose.x.toDouble())
        SmartDashboard.putNumber("Odometry/FieldY", pose.y.toDouble())
        SmartDashboard.putNumberArray("Odometry/velocity", arrayOf(velocity.x,velocity.y))
        SmartDashboard.putNumber("Odometry/Rotation", pose.rotation.degrees)
        SmartDashboard.putNumber("Odometry/test", timer.get())
        SmartDashboard.putBoolean("Odometry/HasTarget", vision.hasTargets())
        timer.reset()
    }
    @Suppress("unused")
    fun resetOdometry(pose: Pose2d?) {
        SwerveOdometry.resetPosition(
            Rotation2d.fromDegrees(NavX.getInvertedAngle()), arrayOf(
                Drivetrain.m_frontLeft.position,
                Drivetrain.m_frontRight.position,
                Drivetrain.m_rearLeft.position,
                Drivetrain.m_rearRight.position
            ),
            pose)
    }

    override fun initSendable(builder: SendableBuilder) {
        SendableRegistry.setName(this, toString())
        builder.addDoubleProperty("x", { pose.x }, null)
        builder.addDoubleProperty("y", { pose.y }, null)
        builder.addDoubleProperty("rotation", { pose.rotation.radians }, null)
    }
}