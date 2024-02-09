@file:Suppress("unused", "MemberVisibilityCanBePrivate", "NOTHING_TO_INLINE") // Not all of these classes are used

/**
 * A set of value classes that represent a value with a unit.  These value classes are represented
 * at runtime as java primitive doubles, so they don't require heap ram allocation and are performant.
 *
 * Note: if you specify an argument type as one of the interfaces declared in this file, it will box
 * the type, leading to a performance hit.  _This is likely fine_, because computers are fast.
 */
package com.team2898.engine.utils

import com.team2898.engine.utils.Sugar.radiansToDegrees
import kotlin.math.PI

interface Unit {
    val value: Double
}

// Distance
interface DistanceUnit : Unit {
    fun meterValue(): Double
}

@JvmInline
value class Meters(override val value: Double) : DistanceUnit {
    override fun meterValue() = value

    override fun toString(): String {
        return "$value m"
    }
}

@JvmInline
value class Feet(override val value: Double) : DistanceUnit {
    override fun meterValue() = value * 0.3048

    override fun toString(): String {
        return "$value ft"
    }
}

@JvmInline
value class Inches(override val value: Double) : DistanceUnit {
    override fun meterValue() = value * 0.0254

    override fun toString(): String {
        return "$value in"
    }
}

// Velocity
interface VelocityUnit : Unit {
    fun metersPerSecondValue(): Double
}

@JvmInline
value class MetersPerSecond(override val value: Double) : VelocityUnit {
    override fun metersPerSecondValue() = value

    override fun toString(): String {
        return "$value m/s"
    }
}

@JvmInline
value class FeetPerSecond(override val value: Double) : VelocityUnit {
    override fun metersPerSecondValue() = value * 0.3084

    override fun toString(): String {
        return "$value ft/s"
    }
}

@JvmInline
value class KilometersPerHour(override val value: Double) : VelocityUnit {
    override fun metersPerSecondValue() = value / 3.6
}

@JvmInline
value class MilesPerHour(override val value: Double) : VelocityUnit {
    override fun metersPerSecondValue() = value * 0.44704

    override fun toString(): String {
        return "$value mph"
    }
}

// Acceleration
interface AccelerationUnit : Unit {
    fun metersPerSecondSquaredValue(): Double
}

@JvmInline
value class MetersPerSecondSquared(override val value: Double) : AccelerationUnit {
    override fun metersPerSecondSquaredValue() = value

    override fun toString(): String {
        return "$value m/s^2"
    }
}

// Rotations
interface AngularVelocityUnit : Unit {
    fun rotationsPerMinute(): Double
}

@JvmInline
value class RPM(override val value: Double) : AngularVelocityUnit {
    override fun rotationsPerMinute() = value
}

// Angles
interface AngleUnit : Unit {
    fun radiansValue(): Double
}

@JvmInline
value class Radians(override val value: Double) : AngleUnit {
    override fun radiansValue() = value

    override fun toString(): String {
        return "$value rad"
    }
}

@JvmInline
value class Degrees(override val value: Double) : AngleUnit {
    companion object {
        const val DEGREES_TO_RADS = (2 * PI) / 360
    }

    override fun radiansValue() = value * DEGREES_TO_RADS

    override fun toString(): String {
        return "$value \u00b0"
    }
}

@JvmInline
value class Rotations(override val value: Double): AngleUnit {
    override fun radiansValue() = value * 2 * PI

    override fun toString(): String {
        return "$value rot"
    }
}

// Mass
interface MassUnit : Unit {
    fun kilogramsValue(): Double
}

@JvmInline
value class Kilograms(override val value: Double) : MassUnit, WeightUnit {  // special case, both mass and weight
    override fun kilogramsValue() = value

    override fun kilogramsWeightValue() = value

    override fun toString(): String {
        return "$value kg"
    }
}

// Weight
interface WeightUnit : Unit {
    fun kilogramsWeightValue(): Double
}

@JvmInline
value class Pounds(override val value: Double) : WeightUnit {
    override fun kilogramsWeightValue() = value * 0.453592

    override fun toString(): String {
        return "$value lbs"
    }
}

// Time
interface TimeUnit : Unit {
    fun secondsValue(): Double
}

@JvmInline
value class Seconds(override val value: Double) : TimeUnit {
    override fun secondsValue() = value

    override fun toString(): String {
        return "$value s"
    }
}

// Frequency
interface FrequencyValue : Unit {
    fun hertzValue(): Double
}

@JvmInline
value class Hertz(override val value: Double) : FrequencyValue {
    override fun hertzValue() = value

    override fun toString(): String {
        return "$value Hz"
    }
}

fun FrequencyValue.toMillis() = Millis(1000 / hertzValue())

@JvmInline
value class Milliseconds(override val value: Double) : TimeUnit {
    /**
     * Creates a new [Milliseconds], but with a long input instead. It's converted to a double,
     * this is purely so that you don't have to specify 10.0 milliseconds, which would be weird.
     */
    constructor(count: Long) : this(count.toDouble())

    override fun secondsValue() = value / 1000

    override fun toString(): String {
        return "$value ms"
    }
}

// Electrical (no interface because there is only one set of electrical units)
@JvmInline
value class Volts(override val value: Double) : Unit {
    override fun toString(): String {
        return "$value v"
    }
}

@JvmInline
value class Amps(override val value: Double) : Unit {
    override fun toString(): String {
        return "$value a"
    }
}

// Typealiases for ease of use
typealias M = Meters
typealias Ft = Feet
typealias In = Inches

typealias mps = MetersPerSecond
typealias Kmph = KilometersPerHour
typealias Mph = MilesPerHour
typealias Fps = FeetPerSecond

typealias Kg = Kilograms

typealias Lb = Pounds

typealias Millis = Milliseconds


// Conversion functions
fun DistanceUnit.toMeters() = Meters(meterValue())

fun VelocityUnit.toMetersPerSecond() = mps(metersPerSecondValue())

fun Radians.toDegrees() = Degrees(value.radiansToDegrees())

val Double.meters get() = Meters(this)
val Int.meters get() = Meters(this.toDouble())
val Double.m get() = Meters(this)
val Int.m get() = Meters(this.toDouble())
val Double.feet get() = Feet(this)
val Int.feet get() = Feet(this.toDouble())
val Double.ft get() = Feet(this)
val Int.ft get() = Feet(this.toDouble())
val Double.inches get() = Inches(this)
val Int.inches get() = Inches(this.toDouble())
val Double.metersPerSecond get() = MetersPerSecond(this)
val Int.metersPerSecond get() = MetersPerSecond(this.toDouble())
val Double.feetPerSecond get() = FeetPerSecond(this)
val Int.feetPerSecond get() = FeetPerSecond(this.toDouble())
val Double.kilometersPerHour get() = KilometersPerHour(this)
val Int.kilometersPerHour get() = KilometersPerHour(this.toDouble())
val Double.kph get() = KilometersPerHour(this)
val Int.kph get() = KilometersPerHour(this.toDouble())
val Double.milesPerHour get() = MilesPerHour(this)
val Int.milesPerHour get() = MilesPerHour(this.toDouble())
val Double.mph get() = MilesPerHour(this)
val Int.mph get() = MilesPerHour(this.toDouble())
val Double.metersPerSecondSquared get() = MetersPerSecondSquared(this)
val Int.metersPerSecondSquared get() = MetersPerSecondSquared(this.toDouble())
val Double.RPM get() = RPM(this)
val Int.RPM get() = RPM(this.toDouble())
val Double.rpm get() = RPM(this)
val Int.rpm get() = RPM(this.toDouble())
val Double.radians get() = Radians(this)
val Int.radians get() = Radians(this.toDouble())
val Double.rad get() = Radians(this)
val Int.rad get() = Radians(this.toDouble())
val Double.degrees get() = Degrees(this)
val Int.degrees get() = Degrees(this.toDouble())
val Double.deg get() = Degrees(this)
val Int.deg get() = Degrees(this.toDouble())
val Double.rotations get() = Rotations(this)
val Int.rotations get() = Rotations(this.toDouble())
val Double.rot get() = Rotations(this)
val Int.rot get() = Rotations(this.toDouble())
val Double.kilograms get() = Kilograms(this)
val Int.kilograms get() = Kilograms(this.toDouble())
val Double.kg get() = Kilograms(this)
val Int.kg get() = Kilograms(this.toDouble())
val Double.pounds get() = Pounds(this)
val Int.pounds get() = Pounds(this.toDouble())
val Double.lbs get() = Pounds(this)
val Int.lbs get() = Pounds(this.toDouble())
val Double.seconds get() = Seconds(this)
val Int.seconds get() = Seconds(this.toDouble())
val Double.hertz get() = Hertz(this)
val Int.hertz get() = Hertz(this.toDouble())
val Double.hz get() = Hertz(this)
val Int.hz get() = Hertz(this.toDouble())
val Double.milliseconds get() = Milliseconds(this)
val Int.milliseconds get() = Milliseconds(this.toDouble())
val Double.ms get() = Milliseconds(this)
val Int.ms get() = Milliseconds(this.toDouble())
val Double.volts get() = Volts(this)
val Int.volts get() = Volts(this.toDouble())
val Double.amps get() = Amps(this)
val Int.amps get() = Amps(this.toDouble())

inline operator fun Meters.plus(other: Meters) = (value + other.value).meters
inline operator fun Meters.unaryPlus() = value.meters
inline operator fun Meters.minus(other: Meters) = (value - other.value).meters
inline operator fun Meters.unaryMinus() = (-value).meters
inline operator fun Meters.times(other: Hertz) = (value * other.value).metersPerSecond // TECHNICALLY Herts are the reciprocal of Seconds
inline operator fun Meters.div(other: Seconds) = (value / other.value).metersPerSecond
inline operator fun Meters.div(other: Milliseconds) = (value / (other.value * 0.001)).metersPerSecond
inline operator fun Feet.plus(other: Feet) = (value + other.value).feet
inline operator fun Feet.unaryPlus() = value.feet
inline operator fun Feet.minus(other: Feet) = (value - other.value).feet
inline operator fun Feet.unaryMinus() = (-value).feet
inline operator fun Feet.div(other: Seconds) = (value / other.value).feetPerSecond
inline operator fun Feet.div(other: Milliseconds) = (value / (other.value * 0.001)).feetPerSecond
inline operator fun Inches.plus(other: Inches) = (value + other.value).inches
inline operator fun Inches.unaryPlus() = value.inches
inline operator fun Inches.minus(other: Inches) = (value - other.value).inches
inline operator fun Inches.unaryMinus() = (-value).inches
inline operator fun Inches.div(other: Seconds) = ((value / 12) / other.value).feetPerSecond
inline operator fun Inches.div(other: Milliseconds) = ((value / 12) / (other.value * 0.001)).feetPerSecond
inline operator fun MetersPerSecond.plus(other: MetersPerSecond) = (value + other.value).metersPerSecond
inline operator fun MetersPerSecond.unaryPlus() = value.metersPerSecond
inline operator fun MetersPerSecond.minus(other: MetersPerSecond) = (value - other.value).metersPerSecond
inline operator fun MetersPerSecond.unaryMinus() = (-value).metersPerSecond
inline operator fun MetersPerSecond.times(other: Seconds) = (value * other.value).m
inline operator fun MetersPerSecond.times(other: Milliseconds) = (value * other.value * 0.001).m
inline operator fun MetersPerSecond.div(other: Seconds) = (value / other.value).metersPerSecondSquared
inline operator fun MetersPerSecond.div(other: Milliseconds) = (value / (other.value * 0.001)).metersPerSecondSquared
inline operator fun FeetPerSecond.plus(other: FeetPerSecond) = (value + other.value).feetPerSecond
inline operator fun FeetPerSecond.unaryPlus() = value.feetPerSecond
inline operator fun FeetPerSecond.minus(other: FeetPerSecond) = (value - other.value).feetPerSecond
inline operator fun FeetPerSecond.unaryMinus() = (-value).feetPerSecond
inline operator fun FeetPerSecond.times(other: Seconds) = (value * other.value).feet
inline operator fun FeetPerSecond.times(other: Milliseconds) = (value * other.value * 0.001).feet
inline operator fun KilometersPerHour.plus(other: KilometersPerHour) = (value + other.value).kilometersPerHour
inline operator fun KilometersPerHour.unaryPlus() = value.kilometersPerHour
inline operator fun KilometersPerHour.minus(other: KilometersPerHour) = (value - other.value).kilometersPerHour
inline operator fun KilometersPerHour.unaryMinus() = (-value).kilometersPerHour
inline operator fun MilesPerHour.plus(other: MilesPerHour) = (value + other.value).milesPerHour
inline operator fun MilesPerHour.unaryPlus() = value.milesPerHour
inline operator fun MilesPerHour.minus(other: MilesPerHour) = (value - other.value).milesPerHour
inline operator fun MilesPerHour.unaryMinus() = (-value).milesPerHour
inline operator fun MetersPerSecondSquared.plus(other: MetersPerSecondSquared) = (value + other.value).metersPerSecondSquared
inline operator fun MetersPerSecondSquared.unaryPlus() = value.metersPerSecondSquared
inline operator fun MetersPerSecondSquared.minus(other: MetersPerSecondSquared) = (value - other.value).metersPerSecondSquared
inline operator fun MetersPerSecondSquared.unaryMinus() = (-value).metersPerSecondSquared
inline operator fun MetersPerSecondSquared.times(other: Seconds) = (value * other.value).metersPerSecond
inline operator fun MetersPerSecondSquared.times(other: Milliseconds) = (value * other.value * 0.001).metersPerSecond
inline operator fun RPM.plus(other: RPM) = (value + other.value).rpm
inline operator fun RPM.unaryPlus() = value.rpm
inline operator fun RPM.minus(other: RPM) = (value - other.value).rpm
inline operator fun RPM.unaryMinus() = (-value).rpm
inline operator fun RPM.times(other: Seconds) = (value * (other.value / 60)).rotations
inline operator fun RPM.times(other: Milliseconds) = (value * (other.value / 60000)).rotations
inline operator fun Radians.plus(other: Radians) = (value + other.value).radians
inline operator fun Radians.unaryPlus() = value.radians
inline operator fun Radians.minus(other: Radians) = (value - other.value).radians
inline operator fun Radians.unaryMinus() = (-value).radians
inline operator fun Degrees.plus(other: Degrees) = (value + other.value).degrees
inline operator fun Degrees.unaryPlus() = value.degrees
inline operator fun Degrees.minus(other: Degrees) = (value - other.value).degrees
inline operator fun Degrees.unaryMinus() = (-value).degrees
inline operator fun Rotations.plus(other: Degrees) = (value + other.value).rotations
inline operator fun Rotations.unaryPlus() = value.rotations
inline operator fun Rotations.minus(other: Degrees) = (value - other.value).rotations
inline operator fun Rotations.unaryMinus() = (-value).rotations
inline operator fun Rotations.div(other: Seconds) = (value / (other.value / 60)).rpm
inline operator fun Rotations.div(other: Milliseconds) = (value / (other.value / 60000)).rpm
inline operator fun Kilograms.plus(other: Kilograms) = (value + other.value).kilograms
inline operator fun Kilograms.unaryPlus() = value.kilograms
inline operator fun Kilograms.minus(other: Kilograms) = (value - other.value).kilograms
inline operator fun Kilograms.unaryMinus() = (-value).kilograms
inline operator fun Pounds.plus(other: Pounds) = (value + other.value).pounds
inline operator fun Pounds.unaryPlus() = value.pounds
inline operator fun Pounds.minus(other: Pounds) = (value - other.value).pounds
inline operator fun Pounds.unaryMinus() = (-value).pounds
inline operator fun Seconds.plus(other: Seconds) = (value + other.value).seconds
inline operator fun Seconds.unaryPlus() = value.seconds
inline operator fun Seconds.minus(other: Seconds) = (value - other.value).seconds
inline operator fun Seconds.unaryMinus() = (-value).seconds
inline operator fun Hertz.plus(other: Hertz) = (value + other.value).hertz
inline operator fun Hertz.unaryPlus() = value.hertz
inline operator fun Hertz.minus(other: Hertz) = (value - other.value).hertz
inline operator fun Hertz.unaryMinus() = (-value).hertz
inline operator fun Milliseconds.plus(other: Milliseconds) = (value + other.value).milliseconds
inline operator fun Milliseconds.unaryPlus() = value.milliseconds
inline operator fun Milliseconds.minus(other: Milliseconds) = (value - other.value).milliseconds
inline operator fun Milliseconds.unaryMinus() = (-value).milliseconds
inline operator fun Volts.plus(other: Volts) = (value + other.value).volts
inline operator fun Volts.unaryPlus() = value.volts
inline operator fun Volts.minus(other: Volts) = (value - other.value).volts
inline operator fun Volts.unaryMinus() = (-value).volts
inline operator fun Amps.plus(other: Amps) = (value + other.value).amps
inline operator fun Amps.unaryPlus() = value.amps
inline operator fun Amps.minus(other: Amps) = (value - other.value).amps
inline operator fun Amps.unaryMinus() = (-value).amps

@JvmInline
value class RGBA(val packed: UInt) {
    constructor(r: UByte, g: UByte, b: UByte, a: UByte = 255.toUByte()) : this(r.toUInt() or (g.toUInt() shl 8) or (b.toUInt() shl 16) or (a.toUInt() shl 24))

    constructor(r: Int, g: Int, b: Int) : this(r.toUByte(), g.toUByte(), b.toUByte())

    val r get() = packed.toUByte()
    val g get() = (packed shr 8).toUByte()
    val b get() = (packed shr 16).toUByte()
    val a get() = (packed shr 24).toUByte()
}

@JvmInline
value class HSVA(val packed: UInt) {
    /**
     * [s], [v], and [a] are from 0 to 255, [h] is from 0 to 180
     */
    constructor(h: Int, s: UByte, v: UByte, a: UByte = 255.toUByte()) : this(h.toUInt() or (s.toUInt() shl 8) or (v.toUInt() shl 16) or (a.toUInt() shl 24))

    /**
     * [s], [v], and [a] are from 0 to 255, [h] is from 0 to 180
     */
    constructor(h: Int, s: Int, v: Int) : this(h, s.toUByte(), v.toUByte())

    /** 0 to 180 */
    val h get() = packed.toUByte()
    val s get() = (packed shr 8).toUByte()
    val v get() = (packed shr 16).toUByte()
    val a get() = (packed shr 24).toUByte()
}

@OptIn(ExperimentalUnsignedTypes::class)
@Suppress("EXPERIMENTAL_API_USAGE", "FINAL_UPPER_BOUND")
@JvmInline
value class RGBAArray(val array: UIntArray) {
    constructor(size: Int) : this(UIntArray(size))

    companion object {
        // this is dumb, but inline classes can't be varargs, so we trick it into having them be objects
        fun <T : RGBA> of(vararg colors: T): RGBAArray {
            return RGBAArray(UIntArray(colors.size) { colors[it].packed })
        }
    }

    val size get() = array.size
    val indices get() = array.indices

    operator fun get(index: Int) = RGBA(array[index])

    operator fun set(index: Int, value: RGBA) {
        array[index] = value.packed
    }

    fun fill(value: RGBA, fromIndex: Int = 0, toIndex: Int = size - 1) {
        array.fill(value.packed, fromIndex, toIndex)
    }
}
