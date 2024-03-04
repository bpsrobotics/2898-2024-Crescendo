package com.team2898.robot

import com.team2898.engine.utils.Vector
import com.team2898.engine.utils.async.Promise
import com.team2898.robot.Constants.ButtonConstants.SHOOT
import com.team2898.robot.subsystems.Arm
import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.event.BooleanEvent
import edu.wpi.first.wpilibj.event.EventLoop
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import kotlin.math.pow
import kotlin.math.sign

/**
 * The Operating Interface object.
 * This is where you put joystick, button, or keyboard inputs.
 *
 * A note about delegated properties, which are used in this object:
 *  A delegated property is where getting (or setting) a field is outsourced
 *  to another object.  Then, whenever you read the property, it asks the
 *  object the property is delegated to for the value.
 */
@Suppress("unused")
object OI : SubsystemBase() {
    /**
     * Threshold below which [process] will return 0.
     * 0.1 historically used, but optimal value unknown.
     */
    private const val DEADZONE_THRESHOLD = 0.1

    val loop = EventLoop()
    /**
     * Utility function for controller axis, optional deadzone and square/cube for extra fine-grain control
     */
    private fun process(
        input: Double,
        deadzone: Boolean = false,
        square: Boolean = false,
        cube: Boolean = false
    ): Double {
        var output = 0.0

        if (deadzone) {
            output = MathUtil.applyDeadband(input, DEADZONE_THRESHOLD)
        }

        if (square) {
            // To keep the signage for output, we multiply by sign(output). This keeps negative inputs resulting in negative outputs.
            output = output.pow(2) * sign(output)
        }

        if (cube) {
            // Because cubing is an odd number of multiplications, we don't need to multiply by sign(output) here.
            output = output.pow(3)
        }

        return output
    }

    // conflicts with the other definition, name it something else after compilation
    @JvmName("process1")
    fun Double.process(deadzone: Boolean = false, square: Boolean = false, cube: Boolean = false) =
        process(this, deadzone, square, cube)

    private val driverController = XboxController(0)
    private val operatorController = Joystick(1)

    // Left and right shoulder switches (the ones next to the trigger) for quickturn
    val quickTurnRight
        get() = process(driverController.rightTriggerAxis, deadzone = true, square = true)
    val quickTurnLeft
        get() = process(driverController.leftTriggerAxis, deadzone = true, square = true)

    // Right joystick y-axis.  Controller mapping can be tricky, the best way is to use the driver station to see what buttons and axis are being pressed.
    // Squared for better control on turn, cubed on throttle
    /** Driver controller's throttle on the left joystick for the X Axis, from -1 (left) to 1 (right) */
    val translationX
        get() = process(driverController.leftX, deadzone = true, square = false)

    /** Driver controller's throttle on the left joystick for the Y Axis, from -1 (down) to 1 (up) */
    val translationY
        get() = process(driverController.leftY, deadzone = true, square = false)

    /** Driver controller's throttle on the right joystick for the X Axis, from -1 (left) to 1 (right) */
    val turnX
        get() = process(driverController.rightX, deadzone = true, square = false)
    /** Driver controller's throttle on the right joystick for the Y Axis, from -1 (down) to 1 (up) */
    val turnY
        get() = process(driverController.rightY, deadzone = true, square = false)
    val leftTrigger
        get() = driverController.leftTriggerAxis
    val rightTrigger
        get() = driverController.rightTriggerAxis
    val driverY
        get() = driverController.yButton
    val driverX
        get() = driverController.xButton
    val resetGyro: BooleanEvent = driverController.rightBumper(loop).debounce(0.5).rising()


    val alignButton
        get() = driverController.yButton
    val alignButtonRelease
        get() = driverController.yButtonReleased
    val alignButtonPressed
        get() = driverController.yButtonPressed
    val highHat get() = operatorController.pov
    val hatVector get() = when (operatorController.pov) {
        0 -> Vector(0,1)
        90 -> Vector(1,0)
        180 -> Vector(0,-1)
        270 -> Vector(-1,0)
        else -> Vector.zero
    }

//    val climb: BooleanEvent = operatorController.button(Constants.ButtonConstants.CLIMBER_UP, loop).debounce(Constants.ButtonConstants.CLIMBER_WAIT_DURATION)

    val armSelectUp: BooleanEvent = operatorController.button(Constants.ButtonConstants.ARM_UP, loop).rising()
    val armSelectDown: BooleanEvent = operatorController.button(Constants.ButtonConstants.ARM_DOWN, loop).rising()

    val armDirectGround: BooleanEvent = operatorController.button(Constants.ButtonConstants.ARM_DIRECT_GROUND, loop).debounce(Constants.ButtonConstants.ARM_DIRECT_WAIT_DURATION).rising()
    val armDirectStowed: BooleanEvent = operatorController.button(Constants.ButtonConstants.ARM_DIRECT_STOWED, loop).debounce(Constants.ButtonConstants.ARM_DIRECT_WAIT_DURATION).rising()
    val armDirectAmp: BooleanEvent = operatorController.button(Constants.ButtonConstants.ARM_DIRECT_AMP, loop).debounce(Constants.ButtonConstants.ARM_DIRECT_WAIT_DURATION).rising()
    val armDirectShooter1: BooleanEvent = operatorController.button(Constants.ButtonConstants.ARM_DIRECT_SHOOTER1, loop).debounce(Constants.ButtonConstants.ARM_DIRECT_WAIT_DURATION)
    val armDirectShooter2: BooleanEvent = operatorController.button(Constants.ButtonConstants.ARM_DIRECT_SHOOTER2, loop).debounce(Constants.ButtonConstants.ARM_DIRECT_WAIT_DURATION).rising()

    val runIntake: BooleanEvent = BooleanEvent(loop) { hatVector == Vector(0,-1) }
    val shooterOutake: BooleanEvent = BooleanEvent(loop) { hatVector == Vector(0, 1) }


    val climbUp: BooleanEvent = operatorController.button(6, loop).debounce(0.25)
    val climbDown: BooleanEvent = operatorController.button(4, loop).debounce(0.25)
    enum class Direction {
        LEFT, RIGHT, UP, DOWN, UPLEFT, UPRIGHT, DOWNLEFT, DOWNRIGHT, INACTIVE;

        fun mirrored() = when (this) {
            LEFT  -> RIGHT
            RIGHT -> LEFT
            else  -> this
        }
        fun toVector() = when (this) {
            LEFT -> Vector(-1,0)
            RIGHT -> Vector(1,0)
            UP -> Vector(0,1)
            DOWN -> Vector(0,-1)
            INACTIVE -> Vector.zero
            UPLEFT -> Vector(-1,1)
            UPRIGHT -> Vector(1, 1)
            DOWNLEFT -> Vector(-1, -1)
            DOWNRIGHT -> Vector(1, -1)
        }
    }

    val alignmentPad get() = when(driverController.pov) {
        0    -> Direction.UP
        45   -> Direction.UPRIGHT
        90   -> Direction.RIGHT
        135  -> Direction.DOWNRIGHT
        180  -> Direction.DOWN
        225  -> Direction.DOWNLEFT
        270  -> Direction.LEFT
        315  -> Direction.UPLEFT
        else -> Direction.INACTIVE
    }

    val operatorTrigger: BooleanEvent = operatorController.button(1, loop)
    val operatorTriggerReleased: BooleanEvent = operatorTrigger.falling()
    object Rumble {
        private var isRumbling  = false
        private var rumbleTime  = 0.0
        private val rumblePower = 0.0
        private val rumbleSide = GenericHID.RumbleType.kRightRumble
        private val rumbleTimer = Timer()
        private val waiting = mutableSetOf<Promise<Unit>>()
        fun set(time: Double, power: Double, side: GenericHID.RumbleType = GenericHID.RumbleType.kBothRumble){
            rumbleTimer.reset()
            rumbleTime = time
            driverController.setRumble(side, power)
            rumbleTimer.start()
        }
        fun until(promise: Promise<Unit>, power: Double = 1.0, side: GenericHID.RumbleType = GenericHID.RumbleType.kBothRumble) {
            driverController.setRumble(side, power)
            waiting.add(promise)
            promise.then { update(); Promise.resolve(Unit) }
        }
        fun update(){
            if(rumbleTimer.hasElapsed(rumbleTime) && !(try {waiting.map { it.hasFulfilled }.reduce { acc, b -> acc or b }} catch (_:Throwable) {false})) {
                driverController.setRumble(GenericHID.RumbleType.kBothRumble, 0.0)
            }
            waiting.forEach {
                if (it.hasFulfilled) waiting.remove(it)
            }
        }
    }
    override fun periodic(){
        Rumble.update()
    }

//    init {
//        armUp.debounce(0.05).onTrue(InstantCommand({
//            Arm.setGoal(Arm.setpoint + 0.1)
//        }))
//        armDown.debounce(0.05).onTrue(InstantCommand({
//            Arm.setGoal(Arm.setpoint - 0.1)
//        }))
//    }


}
