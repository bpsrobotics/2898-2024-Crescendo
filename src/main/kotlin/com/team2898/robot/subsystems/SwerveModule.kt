// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package com.team2898.robot.subsystems


import com.ctre.phoenix6.hardware.CANcoder
import com.revrobotics.CANSparkLowLevel.MotorType.kBrushless
import com.revrobotics.CANSparkMax
import com.revrobotics.RelativeEncoder
import com.team2898.robot.Constants.ModuleConstants
import com.team2898.robot.Constants.ModuleConstants.DrivingD
import com.team2898.robot.Constants.ModuleConstants.DrivingI
import com.team2898.robot.Constants.ModuleConstants.DrivingKa
import com.team2898.robot.Constants.ModuleConstants.DrivingKs
import com.team2898.robot.Constants.ModuleConstants.DrivingKv
import com.team2898.robot.Constants.ModuleConstants.DrivingMotorCurrentLimit
import com.team2898.robot.Constants.ModuleConstants.DrivingP
import com.team2898.robot.Constants.ModuleConstants.TurningMotorCurrentLimit
import edu.wpi.first.math.MathUtil.angleModulus
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import kotlin.math.PI


@Suppress("MemberVisibilityCanBePrivate", "PropertyName", "PrivatePropertyName", "RedundantVisibilityModifier",
    "unused", "SpellCheckingInspection"
)
class SwerveModule(drivingCANId: Int, turningCANId: Int, chassisAngularOffset: Double, absEncoderId: Int, val moduleID: String = "None", inverted: Boolean) {
    private val drivingSparkMax: CANSparkMax
    private val turningSparkMax: CANSparkMax
    public val drivingEncoder: RelativeEncoder
    public val turningEncoder: CANcoder
    private val drivingPIDController: PIDController
    private val drivingFeedForward: SimpleMotorFeedforward
    private val turningPIDController: PIDController
    private var m_chassisAngularOffset = 0.0
    public var m_desiredState = SwerveModuleState(0.0, Rotation2d())
    val encInvert = inverted

    /**
     * Constructs a MAXSwerveModule and configures the driving and turning motor,
     * encoder, and PID controller. This configuration is specific to the REV
     * MAXSwerve Module built with NEOs, SPARKS MAX, and a CTRE CANCoder
     */

    init {
        drivingSparkMax = CANSparkMax(drivingCANId, kBrushless)
        turningSparkMax = CANSparkMax(turningCANId, kBrushless)
        // Factory reset, so we get the SPARKS MAX to a known state before configuring
        // them. This is useful in case a SPARK MAX is swapped out.
        drivingSparkMax.restoreFactoryDefaults()
        turningSparkMax.restoreFactoryDefaults()
        drivingSparkMax.inverted
        drivingSparkMax.setSmartCurrentLimit(DrivingMotorCurrentLimit)

        turningSparkMax.setSmartCurrentLimit(TurningMotorCurrentLimit)

        // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
        drivingEncoder = drivingSparkMax.encoder

        turningEncoder = CANcoder(absEncoderId)
        drivingPIDController = PIDController(
            DrivingP, DrivingI, DrivingD)
        drivingFeedForward = SimpleMotorFeedforward(
            DrivingKs,
            DrivingKv,
            DrivingKa
        )

        turningPIDController = PIDController(
            ModuleConstants.TurningP, ModuleConstants.TurningI, ModuleConstants.TurningD)

        turningPIDController.enableContinuousInput(-PI, PI)

        drivingSparkMax.idleMode = ModuleConstants.DrivingMotorIdleMode
        turningSparkMax.idleMode = ModuleConstants.TurningMotorIdleMode


        drivingEncoder.positionConversionFactor = ModuleConstants.DrivingEncoderPositionFactor
        drivingEncoder.velocityConversionFactor = ModuleConstants.DrivingEncoderVelocityFactor

        // Save the SPARK MAX configurations. If a SPARK MAX browns out during
        // operation, it will maintain the above configurations.
        drivingSparkMax.burnFlash()
        turningSparkMax.burnFlash()
        m_chassisAngularOffset = chassisAngularOffset

        m_desiredState.angle = readEnc()
        drivingEncoder.position = 0.0


        update()

    }

    /** The current state of the module. */
    var state =
            SwerveModuleState(drivingEncoder.velocity,
                readEnc())
    /** The current position of the module. */
    var position =
            SwerveModulePosition(
                drivingEncoder.position,
                readEnc())
    /** Updates swerve state and position */
    fun update(){
        val encAngle = readEnc()
        state = SwerveModuleState(drivingEncoder.velocity, encAngle)
        position = SwerveModulePosition(drivingEncoder.position, encAngle)
        SmartDashboard.putNumber("driving pos " + moduleID, drivingEncoder.position)
//        SmartDashboard.putNumber("Module State " + moduleID, state.angle.radians)
        SmartDashboard.putNumber("Module Position " + moduleID, position.distanceMeters)
//        SmartDashboard.putNumber("Raw Enc " + moduleID, turningEncoder.absolutePosition.valueAsDouble)

    }

    fun readEnc(): Rotation2d {
        val encPos = ((turningEncoder.absolutePosition.valueAsDouble) * 2 * PI) - m_chassisAngularOffset
        val modPos = angleModulus(encPos)
        if (encInvert) {
            return Rotation2d(-modPos)
        } else {
            return Rotation2d(modPos)
        }
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    fun setDesiredState(desiredState: SwerveModuleState) {
        if (Math.abs(desiredState.speedMetersPerSecond) < 0.001){
            stop()
            return
        }

        val optimizedDesiredState = SwerveModuleState.optimize(desiredState,
            state.angle)

//        SmartDashboard.putNumber("desired state angle" + moduleID, optimizedDesiredState.angle.radians)
//        SmartDashboard.putNumber("angle dif" + moduleID, optimizedDesiredState.angle.degrees - state.angle.degrees)

        val drivePid = drivingPIDController.calculate(state.speedMetersPerSecond, optimizedDesiredState.speedMetersPerSecond)
        val feedforward = drivingFeedForward.calculate(optimizedDesiredState.speedMetersPerSecond)
//        SmartDashboard.putNumber("drivePid " + moduleID, drivePid)
//        SmartDashboard.putNumber("FeedForward " + moduleID, feedforward)
        drivingSparkMax.set(feedforward + drivePid)
        turningSparkMax.set(turningPIDController.calculate(state.angle.radians, optimizedDesiredState.angle.radians))
//        SmartDashboard.putNumber("turn voltage " + moduleID, turningPIDController.calculate(state.angle.radians, optimizedDesiredState.angle.radians))

        m_desiredState = desiredState

    }

    /** Zeroes all the SwerveModule encoders.  */
    fun resetEncoders() {
        drivingEncoder.position = 0.0
    }

    fun stop(){
        drivingSparkMax.set(0.0)
        turningSparkMax.set(0.0)
    }
    fun voltageDrive(voltage: Double){
        drivingSparkMax.setVoltage(voltage)
        turningSparkMax.set(turningPIDController.calculate(state.angle.radians, 0.0))
        println("voltage " + voltage + "module id: " + moduleID)
    }
    fun getVoltage(): Double {
        return drivingSparkMax.busVoltage
    }
}