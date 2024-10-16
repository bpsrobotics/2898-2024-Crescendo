package com.team2898.robot.commands.swerve

import com.team2898.robot.subsystems.Drivetrain
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import swervelib.SwerveController
import java.util.function.DoubleSupplier

class TeleopDriveCommand( vForward: DoubleSupplier,
vStrafe: DoubleSupplier,
omega: DoubleSupplier,
slowMode: DoubleSupplier
) : Command() {
    private val vForward: DoubleSupplier
    private val vStrafe: DoubleSupplier
    private val omega: DoubleSupplier
    private val slowMode: DoubleSupplier
    private val controller: SwerveController
    private val swerve: Drivetrain

    init {
        this.swerve = Drivetrain
        this.vForward = vForward
        this.vStrafe = vStrafe
        this.omega = omega
        this.slowMode = slowMode
        controller = swerve.getSwerveController()
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(swerve)
    }

    /** @suppress */
    override fun initialize() {}

    /** @suppress */
    override fun execute() {
        var forwardVelocity = vForward.asDouble
        var strafeVelocity = vStrafe.asDouble
        var angVelocity = omega.asDouble
        var slowMode = slowMode.asDouble
        SmartDashboard.putNumber("vX", forwardVelocity)
        SmartDashboard.putNumber("vY", strafeVelocity)
        SmartDashboard.putNumber("omega", angVelocity)

        var slowModeMultiplier: Double = (-0.8 * slowMode) + 1.0

        forwardVelocity *= slowModeMultiplier
        strafeVelocity *= slowModeMultiplier
        angVelocity *= slowModeMultiplier


        // Drive using raw values.
        swerve.drive(
            Translation2d(forwardVelocity * swerve.maximumSpeed, strafeVelocity * swerve.maximumSpeed),
            angVelocity * controller.config.maxAngularVelocity,
            swerve.fieldOriented,
        )
    }

    /** @suppress */
    override fun end(interrupted: Boolean) {}

    /** @suppress */
    override fun isFinished(): Boolean {
        return false
    }
}