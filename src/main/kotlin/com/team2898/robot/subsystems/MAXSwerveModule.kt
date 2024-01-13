// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package com.team2898.robot.subsystems


import com.revrobotics.*
import com.revrobotics.CANSparkLowLevel.MotorType.kBrushless
import com.team2898.engine.utils.Sugar.circleNormalize
import com.team2898.engine.utils.Sugar.eqEpsilon
import com.team2898.engine.utils.TurningPID
import com.team2898.robot.Constants.ModuleConstants
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.AnalogEncoder
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import kotlin.math.PI
import kotlin.math.absoluteValue

@Suppress("MemberVisibilityCanBePrivate", "PropertyName", "PrivatePropertyName", "RedundantVisibilityModifier",
    "unused", "SpellCheckingInspection"
)
class MAXSwerveModule(drivingCANId: Int, turningCANId: Int, chassisAngularOffset: Double, analogPort: Int, val moduleID: String = "None") {
    private val m_drivingSparkMax: CANSparkMax
    private val m_turningSparkMax: CANSparkMax
    public val m_drivingEncoder: RelativeEncoder
    public val m_turningEncoder: AnalogEncoder
    private val m_drivingPIDController: SparkPIDController
    private val m_turningPIDController: TurningPID
    private var m_chassisAngularOffset = 0.0
    public var m_desiredState = SwerveModuleState(0.0, Rotation2d())

    /**
     * Constructs a MAXSwerveModule and configures the driving and turning motor,
     * encoder, and PID controller. This configuration is specific to the REV
     * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
     * Encoder.
     */

    init {
        m_drivingSparkMax = CANSparkMax(drivingCANId, kBrushless)
        m_turningSparkMax = CANSparkMax(turningCANId, kBrushless)

        // Factory reset, so we get the SPARKS MAX to a known state before configuring
        // them. This is useful in case a SPARK MAX is swapped out.
        //m_drivingSparkMax.restoreFactoryDefaults()
        //m_turningSparkMax.restoreFactoryDefaults()

        // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
        m_drivingEncoder = m_drivingSparkMax.encoder
        m_turningEncoder = AnalogEncoder(analogPort)
        m_drivingPIDController = m_drivingSparkMax.pidController
        m_turningPIDController = TurningPID(ModuleConstants.kTurningP, ModuleConstants.kTurningD)
        m_drivingPIDController.setFeedbackDevice(m_drivingEncoder)
        //m_turningPIDController.setFeedbackDevice(m_turningEncoder)

        // Apply position and velocity conversion factors for the driving encoder. The
        // native units for position and velocity are rotations and RPM, respectively,
        // but we want meters and meters per second to use with WPILib's swerve APIs.
        m_drivingEncoder.positionConversionFactor = ModuleConstants.kDrivingEncoderPositionFactor
        m_drivingEncoder.velocityConversionFactor = ModuleConstants.kDrivingEncoderVelocityFactor

        // Apply position and velocity conversion factors for the turning encoder. We
        // want these in radians and radians per second to use with WPILib's swerve
        // APIs.
        //m_turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor)
        //m_turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor)

        // Invert the turning encoder, since the output shaft rotates in the opposite direction of
        // the steering motor in the MAXSwerve Module.
        //m_turningEncoder.setInverted(ModuleConstants.kTurningEncoderInverted)

        // Enable PID wrap around for the turning motor. This will allow the PID
        // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
        // to 10 degrees will go through 0 rather than the other direction which is a
        // longer route.
        //m_turningPIDController.positionPIDWrappingEnabled = true
        //m_turningPIDController.positionPIDWrappingMinInput = ModuleConstants.kTurningEncoderPositionPIDMinInput
        //m_turningPIDController.positionPIDWrappingMaxInput = ModuleConstants.kTurningEncoderPositionPIDMaxInput

        // Set the PID gains for the driving motor. Note these are example gains, and you
        // may need to tune them for your own robot!
        m_drivingPIDController.p = ModuleConstants.kDrivingP
        m_drivingPIDController.i = ModuleConstants.kDrivingI
        m_drivingPIDController.d = ModuleConstants.kDrivingD
        m_drivingPIDController.ff = ModuleConstants.kDrivingFF
        m_drivingPIDController.setOutputRange(ModuleConstants.kDrivingMinOutput,
            ModuleConstants.kDrivingMaxOutput)

        // Set the PID gains for the turning motor. Note these are example gains, and you
        // may need to tune them for your own robot!
        //m_turningPIDController.ff = ModuleConstants.kTurningFF
        //m_turningPIDController.(ModuleConstants.kTurningMinOutput,
        //    ModuleConstants.kTurningMaxOutput)
        m_drivingSparkMax.idleMode = ModuleConstants.kDrivingMotorIdleMode
        m_turningSparkMax.idleMode = ModuleConstants.kTurningMotorIdleMode
        m_drivingSparkMax.setSmartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit)
        m_turningSparkMax.setSmartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit)

        // Save the SPARK MAX configurations. If a SPARK MAX browns out during
        // operation, it will maintain the above configurations.
        m_drivingSparkMax.burnFlash()
        m_turningSparkMax.burnFlash()
        m_chassisAngularOffset = chassisAngularOffset
        m_desiredState.angle = Rotation2d(readEnc())
        m_drivingEncoder.position = 0.0
        m_drivingSparkMax.idleMode
    }

    /** The current state of the module. */
    val state: SwerveModuleState
        get() =// Apply chassis angular offset to the encoder position to get the position
            // relative to the chassis.
            SwerveModuleState(m_drivingEncoder.velocity,
                Rotation2d(readEnc()))
    /** The current position of the module. */
    val position: SwerveModulePosition
        get() =// Apply chassis angular offset to the encoder position to get the position
            // relative to the chassis.
            SwerveModulePosition(
                m_drivingEncoder.position,
                Rotation2d(readEnc()))

    fun readEnc(): Double {
        var encPos = (m_turningEncoder.absolutePosition * 2.0 * PI) - m_chassisAngularOffset
        encPos %= 2 * PI
        if(encPos < 0) return (2*PI) + encPos
        return encPos
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    fun setDesiredState(desiredState: SwerveModuleState) {
        // Apply chassis angular offset to the desired state.
        val correctedDesiredState = SwerveModuleState()
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond
        correctedDesiredState.angle = desiredState.angle

        // Optimize the reference state to avoid spinning further than 90 degrees.
        val optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
            Rotation2d(readEnc())) //correctedDesiredState

        // Command driving and turning SPARKS MAX towards their respective setpoints.
        m_drivingPIDController.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkBase.ControlType.kVelocity)
        SmartDashboard.putNumber(moduleID + "_SetPoint", optimizedDesiredState.angle.radians.circleNormalize())
        m_turningPIDController.setPoint = optimizedDesiredState.angle.radians.circleNormalize()
        m_turningPIDController.kP = ModuleConstants.kTurningP
        m_turningPIDController.kD = ModuleConstants.kTurningD
        var turningVoltage = m_turningPIDController.motorOutput(readEnc())
        turningVoltage = when{
            turningVoltage.eqEpsilon(0,0.04) -> 0.0
            turningVoltage < 0 -> turningVoltage - ModuleConstants.Ks
            else -> turningVoltage + ModuleConstants.Ks
        }
        SmartDashboard.putNumber("Voltage", turningVoltage)
        SmartDashboard.putNumber("error", (readEnc() - desiredState.angle.radians).absoluteValue)
        if(readEnc().eqEpsilon(desiredState.angle.radians,0.07)) turningVoltage = 0.0

        //if(reversed) m_turningSparkMax.set(-turningVoltage)
        //else m_turningSparkMax.set(turningVoltage)
        m_turningSparkMax.set(-turningVoltage)
        m_desiredState = desiredState
        //SmartDashboard.putNumber("turningMotorVoltage" + moduleID, turningVoltage)
        //SmartDashboard.putNumber("drivingMotorVoltage" + moduleID, drivingVoltage)
    }

    /** Zeroes all the SwerveModule encoders.  */
    fun resetEncoders() {
        m_drivingEncoder.position = 0.0
    }
}