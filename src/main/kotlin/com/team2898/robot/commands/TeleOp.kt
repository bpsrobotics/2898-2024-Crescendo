package com.team2898.robot.commands

//The commands for both the driver and operator
//import com.com.engine.utils.`M/s`
//import com.team2898.robot.Constants.ArmHeights.*
//import com.team2898.robot.Field
//import com.team2898.robot.OI
//import com.team2898.robot.subsystems.Arm

import com.team2898.engine.utils.MovingAverage
import com.team2898.engine.utils.Sugar.degreesToRadians
import com.team2898.engine.utils.Sugar.eqEpsilon
import com.team2898.engine.utils.Sugar.radiansToDegrees
import com.team2898.engine.utils.TurningPID
import com.team2898.engine.utils.Vector
import com.team2898.engine.utils.odometry.Vision
import com.team2898.robot.Constants
import com.team2898.robot.OI
import com.team2898.robot.subsystems.*
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import kotlin.math.*
import com.team2898.robot.Constants.*
import edu.wpi.first.math.filter.LinearFilter
import edu.wpi.first.wpilibj.DriverStation
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.pow
import kotlin.math.sign

/**
    Called when the Tele-Operated stage of the game begins.
 */
class TeleOp : Command() {
    init { addRequirements(Drivetrain) }
    // Called when the command is started.
    override fun initialize() {
        NavX.reset() //FIXME: If auto doesn't end pointing the right way, field oriented driving will be messed up
    }

    var angle = 0.0

    val robotTurningPID = TurningPID(3.5,0.05)
    val robotTurningKS = 0.1
    var resetGyroTimer = Timer()
    private val vision = Vision("Camera_Module_v1")
    var alignMode = false
    val targetID = if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
        Constants.VisionConstants.RED_ALLIANCE_SPEAKER_TAG_ID
    } else{
        Constants.VisionConstants.BLUE_ALLIANCE_SPEAKER_TAG_ID
    }
//    val targetID = 4
    // Called every time the scheduler runs while the command is scheduled.
    fun turnSpeedNormal():Double {
        return -OI.turnX
    }
    fun turnSpeedFieldOriented(): Double {
        if((abs(OI.turnY)+abs(OI.turnX)).eqEpsilon(0,0.2)) return 0.0
        angle = atan2(OI.turnY,OI.turnX)
        SmartDashboard.putNumber("angle", angle)

        robotTurningPID.setPoint = angle
        var turnSpeed = robotTurningPID.turnspeedOutputNoNormalize(NavX.getInvertedAngle().degreesToRadians())
        if (!turnSpeed.eqEpsilon(0, 0.04)) turnSpeed += robotTurningKS * turnSpeed.sign
        return turnSpeed
    }
    fun handleResetGyro(){
//        if(OI.resetGyroStart){
//            resetGyroTimer.reset()
//            resetGyroTimer.start()
//        }
        if(OI.resetGyro) {
//            if(resetGyroTimer.hasElapsed(0.5)){
                NavX.reset()
                OI.Rumble.set(0.25,1.0, GenericHID.RumbleType.kRightRumble)
//            }
        }
//        if(OI.resetGyroEnd){
//            resetGyroTimer.stop()
//        }
    }
    val climbReachInputBuffer = Timer()
    val climbLiftInputBuffer = Timer()
    fun getAngleToSpeaker(d:Double) : Double{
        if (!vision.hasSpecificTarget(targetID)) return ArmConstants.ArmHeights.SHOOTER1.position
        val velocity = 28.33
        var x1 = 0.0
        var y1 = 0.0
        var h = 1.41 - 0.19125
        var h2 = 2.03

        var distToSpeaker = sqrt(d.pow(2)-h.pow(2))
        var angleToSpeaker = 0.0

        for(i in 1..5) {
            angleToSpeaker = (180.0 - atan2(h2 - y1, distToSpeaker + x1).radiansToDegrees() - (26.42+90))
            x1 = 0.1 + 0.6*cos(angleToSpeaker.degreesToRadians())
            y1 = 0.355 + 0.6*sin(angleToSpeaker.degreesToRadians())
        }
        val a = (180 - angleToSpeaker - (90+26.42)).degreesToRadians()
        h2 = 2.03 + (2.03 - ((x1 + distToSpeaker)*tan(a) - 9.8*(x1+distToSpeaker).pow(2)/(2*velocity.pow(2)*cos(a).pow(2))+y1))
        val angleReturn =(0.5 * PI) - (180.0 - atan2(h2 - y1, distToSpeaker + x1).radiansToDegrees() - (26.42+90)).degreesToRadians()
        SmartDashboard.putNumber("AngleToSpeaker", distToSpeaker)
        SmartDashboard.putNumber("arm angle b4", angleReturn)
        return angleReturn

    }
    var climbDown = false
    val distanceAverage = LinearFilter.movingAverage(5)
    fun peripheralControls() {
        var visiondist = 0.0
        if (vision.hasSpecificTarget(targetID)) {
            visiondist = distanceAverage.calculate(vision.getCameraData(targetID).x)
        }
        when {
            OI.armSelectUp ->   Arm.setGoal(Arm.pos() - 0.1)
            OI.armSelectDown -> Arm.setGoal(Arm.pos() + 0.1)
        }
        when {
            OI.armDirectGround ->   Arm.setGoal(ArmConstants.ArmHeights.GROUND.position)
            OI.armDirectStowed ->   Arm.setGoal(ArmConstants.ArmHeights.STOWED.position)
            OI.armDirectAmp ->      Arm.setGoal(ArmConstants.ArmHeights.AMP.position)
            OI.armDirectShooter1Pressed -> Arm.setGoal( getAngleToSpeaker(visiondist) )
            OI.armDirectShooter2 -> Arm.setGoal(ArmConstants.ArmHeights.SHOOTER2.position)
        }
        when {
            OI.operatorTrigger ->              Shooter.setVoltage(6.0)
            OI.hatVector == Vector(0,1) -> Shooter.setVoltage(-0.75)
            else ->                            Shooter.stop()

        }
        when (OI.hatVector) {
            Vector(0, -1) -> Intake.intake(0.55)
            Vector(0,1) -> Intake.outtake()
            else ->               Intake.intake(0.0)
        }

        when {
            OI.climbUp ->   Climber.setSpeed(-12.0)
            OI.climbDown -> Climber.setSpeed(12.0)
            else ->         Climber.setSpeed(0.0)
        }
    }

    val turnController = PIDController(0.1, 0.0, 0.0)
    private var targetRotation = Odometry.pose.rotation.degrees
    fun alignRobot() {
        // Doesn't run the function if there isn't a target.
        if (vision.hasSpecificTarget(targetID)){
            targetRotation = Odometry.pose.rotation.degrees + vision.getCameraYaw(targetID)
        }
        if (OI.alignButton) {
                // Get the desired rotation of the robot by adding the degrees needed to face the target
            var rotationSpeed = turnController.calculate(Odometry.pose.rotation.radians, targetRotation.degreesToRadians())
            Drivetrain.drive(0.0, 0.0, rotationSpeed, true, true, true)
                // Use our forward/turn speeds to control the drivetrain
        }
    }
    override fun execute() {
        handleResetGyro()
//        alignRobot()
        peripheralControls()
        Drivetrain.drive(
            OI.translationX,
            OI.translationY,
           -OI.turnX,
            fieldRelative = true,
            rateLimit = true,
            secondOrder = true
        )


    }

    // Called once the command ends or is interrupted.
    override fun end(interrupted: Boolean) {}


    // Returns true when the command should end.
    override fun isFinished(): Boolean {
        return false
    }
}
