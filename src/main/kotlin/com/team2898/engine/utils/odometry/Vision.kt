package com.team2898.engine.utils.odometry

import edu.wpi.first.apriltag.AprilTag
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.*
import org.photonvision.EstimatedRobotPose
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.targeting.PhotonTrackedTarget
import java.util.*


val aprilTags1: MutableList<AprilTag> = mutableListOf(
    AprilTag(1,
        Pose3d(
            Translation3d(1.05,1.07,0.0),
            Rotation3d()
        )
    ),
    AprilTag(3,
        Pose3d(
            Translation3d(0.15,1.07,0.0),
            Rotation3d()
        )
    )
)
val testLayout1 = AprilTagFieldLayout(aprilTags1,10.0,5.0)

class Vision (
    CameraName: String
) {

    val cam = PhotonCamera(CameraName);
    var robotToCam = Transform3d(
        Translation3d(0.5, 0.0, 0.5),
        Rotation3d(0.0, 0.0, 0.0)
    )
    val aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile)


    val PoseEstimator = PhotonPoseEstimator(
        aprilTagFieldLayout,
        PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        cam,
        robotToCam
    )
    fun getCameraData() : PhotonTrackedTarget {
        var result = cam.getLatestResult();
        val targets: PhotonTrackedTarget = result.getBestTarget()
        return targets
    }
    fun hasTargets() : Boolean{
        val result = cam.latestResult
        return result.hasTargets()
    }
    fun getEstimatedPose(prevEstimatedRobotPose: Pose2d?): Optional<EstimatedRobotPose>? {
        if(prevEstimatedRobotPose != null) PoseEstimator.setReferencePose(prevEstimatedRobotPose)
        val pose = PoseEstimator.update() ?: return null
        return pose
    }
}