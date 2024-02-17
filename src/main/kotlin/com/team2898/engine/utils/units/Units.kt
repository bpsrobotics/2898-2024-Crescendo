@file:Suppress("unused", "MemberVisibilityCanBePrivate", "NOTHING_TO_INLINE") // Not all of these classes are used

/**
 * A set of value classes that represent a value with a unit.  These value classes are represented
 * at runtime as java primitive doubles, so they don't require heap ram allocation and are performant.
 *
 * Note: if you specify an argument type as one of the interfaces declared in this file, it will box
 * the type, leading to a performance hit.  _This is likely fine_, because computers are fast.
 */
package com.team2898.engine.utils.units

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
    constructor(v: Int) : this(v.toDouble())
    override fun meterValue() = value

    override fun toString() = "$value m"
}

@JvmInline
value class Kilometers(override val value: Double) : DistanceUnit {
    constructor(v: Int) : this(v.toDouble())
    override fun meterValue() = value * 1000
    override fun toString() = "$value km"
}

@JvmInline
value class Feet(override val value: Double) : DistanceUnit {
    constructor(v: Int) : this(v.toDouble())
    override fun meterValue() = value * 0.3048

    override fun toString(): String {
        return "$value ft"
    }
}

@JvmInline
value class Inches(override val value: Double) : DistanceUnit {
    constructor(v: Int) : this(v.toDouble())
    override fun meterValue() = value * 0.0254

    override fun toString(): String {
        return "$value in"
    }
}

//@JvmInline
//value class AstronomicalUnits(override val value: Double) : DistanceUnit {
//    constructor(v: Int) : this(v.toDouble())
//    override fun meterValue() = value * 149_597_870_700
//    override fun toString() = "$value au"
//}

interface AreaUnit : Unit {
    fun metersSquaredValue(): Double
}

@JvmInline
value class MetersSquared(override val value: Double) : AreaUnit {
    constructor(v: Int) : this(v.toDouble())
    override fun metersSquaredValue() = value
    override fun toString() = "$value m^2"
}

interface VolumeUnit : Unit {
    fun metersCubedValue(): Double
}

@JvmInline
value class MetersCubed(override val value: Double) : VolumeUnit {
    constructor(v: Int) : this(v.toDouble())
    override fun metersCubedValue() = value
    override fun toString() = "$value m^3"
}

@JvmInline
value class Liters(override val value: Double): VolumeUnit {
    constructor(v: Int) : this(v.toDouble())
    override fun metersCubedValue() = 1000 * value
    override fun toString() = "$value L"
}

// Velocity
interface VelocityUnit : Unit {
    fun metersPerSecondValue(): Double
}

@JvmInline
value class MetersPerSecond(override val value: Double) : VelocityUnit {
    constructor(v: Int) : this(v.toDouble())
    override fun metersPerSecondValue() = value

    override fun toString(): String {
        return "$value m/s"
    }
}

@JvmInline
value class FeetPerSecond(override val value: Double) : VelocityUnit {
    constructor(v: Int) : this(v.toDouble())
    override fun metersPerSecondValue() = value * 0.3084

    override fun toString(): String {
        return "$value ft/s"
    }
}

@JvmInline
value class KilometersPerHour(override val value: Double) : VelocityUnit {
    constructor(v: Int) : this(v.toDouble())
    override fun metersPerSecondValue() = value / 3.6
    override fun toString() = "$value kph"
}

@JvmInline
value class MilesPerHour(override val value: Double) : VelocityUnit {
    constructor(v: Int) : this(v.toDouble())
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
    constructor(v: Int) : this(v.toDouble())
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
    constructor(v: Int) : this(v.toDouble())
    override fun rotationsPerMinute() = value
    override fun toString() = "$value rpm"
}

@JvmInline
value class RPS(override val value: Double) : AngularVelocityUnit {
    constructor(v: Int) : this(v.toDouble())
    override fun rotationsPerMinute() = value * 60
    override fun toString() = "$value rps"
}

// Angles
interface AngleUnit : Unit {
    fun radiansValue(): Double
}

@JvmInline
value class Radians(override val value: Double) : AngleUnit {
    constructor(v: Int) : this(v.toDouble())
    override fun radiansValue() = value

    override fun toString(): String {
        return "$value rad"
    }
}

@JvmInline
value class Degrees(override val value: Double) : AngleUnit {
    constructor(v: Int) : this(v.toDouble())
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
    constructor(v: Int) : this(v.toDouble())
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
    constructor(v: Int) : this(v.toDouble())
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
    constructor(v: Int) : this(v.toDouble())
    override fun kilogramsWeightValue() = value * 0.453592

    override fun toString(): String {
        return "$value lb${if (value == 1.0) "" else "s"}"
    }
}

// Time
interface TimeUnit : Unit {
    fun secondsValue(): Double
}

@JvmInline
value class Seconds(override val value: Double) : TimeUnit {
    constructor(v: Int) : this(v.toDouble())
    override fun secondsValue() = value

    override fun toString(): String {
        return "$value s"
    }
}

@JvmInline
value class Minutes(override val value: Double) : TimeUnit {
    constructor(v: Int) : this(v.toDouble())
    override fun secondsValue() = value * 60
    override fun toString() = "$value min"
}

@JvmInline
value class Hours(override val value: Double) : TimeUnit {
    constructor(v: Int) : this(v.toDouble())
    override fun secondsValue() = value * 3600
    override fun toString() = "$value hr${if (value == 1.0) "" else "s"}"
}

@JvmInline
value class Milliseconds(override val value: Double) : TimeUnit {
    constructor(v: Int) : this(v.toDouble())
    /**
     * Creates a new [Milliseconds], but with a long input instead. It's converted to a double,
     * this is purely so that you don't have to specify 10.0 milliseconds, which would be weird.
     */
    constructor(count: Long) : this(count.toDouble())

    override fun secondsValue() = value * 0.001

    override fun toString(): String {
        return "$value ms"
    }
}

// Frequency
interface FrequencyUnit : Unit {
    fun hertzValue(): Double
}

@JvmInline
value class Hertz(override val value: Double) : FrequencyUnit {
    constructor(v: Int) : this(v.toDouble())
    override fun hertzValue() = value

    override fun toString(): String {
        return "$value Hz"
    }
}


// Electrical (no interface because there is only one set of electrical units)
@JvmInline
value class Volts(override val value: Double) : Unit {
    constructor(v: Int) : this(v.toDouble())
    override fun toString(): String {
        return "$value V"
    }
}

@JvmInline
value class Amps(override val value: Double) : Unit {
    constructor(v: Int) : this(v.toDouble())
    override fun toString(): String {
        return "$value A"
    }
}

@JvmInline
value class Watts(override val value: Double) : Unit {
    constructor(v: Int) : this(v.toDouble())

    override fun toString() = "$value W"
}

// Typealiases for ease of use
typealias M = Meters
typealias KM = Kilometers
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
fun DistanceUnit.toKilometers() = Kilometers(meterValue() / Kilometers(1.0).meterValue())
fun DistanceUnit.toFeet() = Feet(meterValue() / Feet(1.0).meterValue())
fun DistanceUnit.toInches() = Inches(meterValue() / Inches(1.0).meterValue())
val DistanceUnit.meters get() = toMeters()
val DistanceUnit.m get() = toMeters()
val DistanceUnit.kilometers get() = toKilometers()
val DistanceUnit.km get() = toKilometers()
val DistanceUnit.feet get() = toFeet()
val DistanceUnit.ft get() = toFeet()
val DistanceUnit.inches get() = toInches()

fun VelocityUnit.toMetersPerSecond() = mps(metersPerSecondValue())
fun VelocityUnit.toFeetPerSecond() = FeetPerSecond(metersPerSecondValue() / FeetPerSecond(1.0).metersPerSecondValue())
fun VelocityUnit.toKilometersPerHour() = KilometersPerHour(metersPerSecondValue() / KilometersPerHour(1.0).metersPerSecondValue())
fun VelocityUnit.toMilesPerHour() = MilesPerHour(metersPerSecondValue() / MilesPerHour(1.0).metersPerSecondValue())
val VelocityUnit.metersPerSecond get() = toMetersPerSecond()
val VelocityUnit.mps: MetersPerSecond get() = toMetersPerSecond()
val VelocityUnit.feetPerSecond get() = toFeetPerSecond()
val VelocityUnit.fps get() = toFeetPerSecond()
val VelocityUnit.kilometersPerHour get() = toKilometersPerHour()
val VelocityUnit.kph get() = toKilometersPerHour()
val VelocityUnit.milesPerHour get() = toMilesPerHour()
val VelocityUnit.mph get() = toMilesPerHour()

fun AngularVelocityUnit.toRotationsPerMinute() = RPM(rotationsPerMinute())
fun AngularVelocityUnit.toRotationsPerSecond() = RPS(rotationsPerMinute() / RPS(1.0).rotationsPerMinute())
val AngularVelocityUnit.rotationsPerMinute get() = toRotationsPerMinute()
val AngularVelocityUnit.rpm get() = toRotationsPerMinute()
val AngularVelocityUnit.rotationsPerSecond get() = toRotationsPerSecond()
val AngularVelocityUnit.rps get() = toRotationsPerSecond()

fun AngleUnit.toRadians() = Radians(radiansValue())
fun AngleUnit.toDegrees() = Degrees(radiansValue() / Degrees(1.0).radiansValue())
fun AngleUnit.toRotations() = Rotations(radiansValue() / Rotations(1.0).radiansValue())

fun TimeUnit.toSeconds() = Seconds(secondsValue())
fun TimeUnit.toMinutes() = Minutes(secondsValue() / Minutes(1.0).secondsValue())
fun TimeUnit.toHours() = Hours(secondsValue() / Hours(1.0).secondsValue())
fun TimeUnit.toMilliseconds() = Milliseconds(secondsValue() / Milliseconds(1.0).secondsValue())

fun FrequencyUnit.toMillis() = Millis(1000 / hertzValue())

val Double.meters get() = Meters(this)
val Int.meters get() = Meters(this.toDouble())
val Long.meters get() = Meters(this.toDouble())
val Double.m get() = Meters(this)
val Int.m get() = Meters(this.toDouble())
val Long.m get() = Meters(this.toDouble())
val Double.kilometers get() = Kilometers(this)
val Int.kilometers get() = Kilometers(this.toDouble())
val Long.kilometers get() = Kilometers(this.toDouble())
val Double.km get() = Kilometers(this)
val Int.km get() = Kilometers(this.toDouble())
val Long.km get() = Kilometers(this.toDouble())
val Double.feet get() = Feet(this)
val Int.feet get() = Feet(this.toDouble())
val Long.feet get() = Feet(this.toDouble())
val Double.ft get() = Feet(this)
val Int.ft get() = Feet(this.toDouble())
val Long.ft get() = Feet(this.toDouble())
val Double.inches get() = Inches(this)
val Int.inches get() = Inches(this.toDouble())
val Long.inches get() = Inches(this.toDouble())
val Double.metersSquared get() = MetersSquared(this)
val Int.metersSquared get() = MetersSquared(this.toDouble())
val Long.metersSquared get() = MetersSquared(this.toDouble())
val Double.m2 get() = MetersSquared(this)
val Int.m2 get() = MetersSquared(this.toDouble())
val Long.m2 get() = MetersSquared(this.toDouble())
val Double.metersPerSecond get() = MetersPerSecond(this)
val Int.metersPerSecond get() = MetersPerSecond(this.toDouble())
val Long.metersPerSecond get() = MetersPerSecond(this.toDouble())
val Double.mps get() = MetersPerSecond(this)
val Int.mps get() = MetersPerSecond(this.toDouble())
val Long.mps get() = MetersPerSecond(this.toDouble())
val Double.feetPerSecond get() = FeetPerSecond(this)
val Int.feetPerSecond get() = FeetPerSecond(this.toDouble())
val Long.feetPerSecond get() = FeetPerSecond(this.toDouble())
val Double.kilometersPerHour get() = KilometersPerHour(this)
val Int.kilometersPerHour get() = KilometersPerHour(this.toDouble())
val Long.kilometersPerHour get() = KilometersPerHour(this.toDouble())
val Double.kph get() = KilometersPerHour(this)
val Int.kph get() = KilometersPerHour(this.toDouble())
val Long.kph get() = KilometersPerHour(this.toDouble())
val Double.milesPerHour get() = MilesPerHour(this)
val Int.milesPerHour get() = MilesPerHour(this.toDouble())
val Long.milesPerHour get() = MilesPerHour(this.toDouble())
val Double.mph get() = MilesPerHour(this)
val Int.mph get() = MilesPerHour(this.toDouble())
val Long.mph get() = MilesPerHour(this.toDouble())
val Double.metersPerSecondSquared get() = MetersPerSecondSquared(this)
val Int.metersPerSecondSquared get() = MetersPerSecondSquared(this.toDouble())
val Long.metersPerSecondSquared get() = MetersPerSecondSquared(this.toDouble())
val Double.rotationsPerMinute get() = RPM(this)
val Int.rotationsPerMinute get() = RPM(this.toDouble())
val Long.rotationsPerMinute get() = RPM(this.toDouble())
val Double.rpm get() = RPM(this)
val Int.rpm get() = RPM(this.toDouble())
val Long.rpm get() = RPM(this.toDouble())
val Double.rotationsPerSecond get() = RPS(this)
val Int.rotationsPerSecond get() = RPS(this.toDouble())
val Long.rotationsPerSecond get() = RPS(this.toDouble())
val Double.rps get() = RPS(this)
val Int.rps get() = RPS(this.toDouble())
val Long.rps get() = RPS(this.toDouble())
val Double.radians get() = Radians(this)
val Int.radians get() = Radians(this.toDouble())
val Long.radians get() = Radians(this.toDouble())
val Double.rad get() = Radians(this)
val Int.rad get() = Radians(this.toDouble())
val Long.rad get() = Radians(this.toDouble())
val Double.degrees get() = Degrees(this)
val Int.degrees get() = Degrees(this.toDouble())
val Long.degrees get() = Degrees(this.toDouble())
val Double.deg get() = Degrees(this)
val Int.deg get() = Degrees(this.toDouble())
val Long.deg get() = Degrees(this.toDouble())
val Double.rotations get() = Rotations(this)
val Int.rotations get() = Rotations(this.toDouble())
val Long.rotations get() = Rotations(this.toDouble())
val Double.rot get() = Rotations(this)
val Int.rot get() = Rotations(this.toDouble())
val Long.rot get() = Rotations(this.toDouble())
val Double.kilograms get() = Kilograms(this)
val Int.kilograms get() = Kilograms(this.toDouble())
val Long.kilograms get() = Kilograms(this.toDouble())
val Double.kg get() = Kilograms(this)
val Int.kg get() = Kilograms(this.toDouble())
val Long.kg get() = Kilograms(this.toDouble())
val Double.pounds get() = Pounds(this)
val Int.pounds get() = Pounds(this.toDouble())
val Long.pounds get() = Pounds(this.toDouble())
val Double.lbs get() = Pounds(this)
val Int.lbs get() = Pounds(this.toDouble())
val Long.lbs get() = Pounds(this.toDouble())
val Double.seconds get() = Seconds(this)
val Int.seconds get() = Seconds(this.toDouble())
val Long.seconds get() = Seconds(this.toDouble())
val Double.minutes get() = Minutes(this)
val Int.minutes get() = Minutes(this.toDouble())
val Long.minutes get() = Minutes(this.toDouble())
val Double.hours get() = Hours(this)
val Int.hours get() = Hours(this.toDouble())
val Long.hours get() = Hours(this.toDouble())
val Double.milliseconds get() = Milliseconds(this)
val Int.milliseconds get() = Milliseconds(this.toDouble())
val Long.milliseconds get() = Milliseconds(this.toDouble())
val Double.hertz get() = Hertz(this)
val Int.hertz get() = Hertz(this.toDouble())
val Long.hertz get() = Hertz(this.toDouble())
val Double.hz get() = Hertz(this)
val Int.hz get() = Hertz(this.toDouble())
val Long.hz get() = Hertz(this.toDouble())
val Double.ms get() = Milliseconds(this)
val Int.ms get() = Milliseconds(this.toDouble())
val Long.ms get() = Milliseconds(this.toDouble())
val Double.volts get() = Volts(this)
val Int.volts get() = Volts(this.toDouble())
val Long.volts get() = Volts(this.toDouble())
val Double.amps get() = Amps(this)
val Int.amps get() = Amps(this.toDouble())
val Long.amps get() = Amps(this.toDouble())
val Double.watts get() = Watts(this)
val Int.watts get() = Watts(this.toDouble())
val Long.watts get() = Watts(this.toDouble())

inline operator fun Meters.plus(other: Meters) = (value + other.value).meters
inline operator fun Meters.unaryPlus() = value.meters
inline operator fun Meters.minus(other: Meters) = (value - other.value).meters
inline operator fun Meters.unaryMinus() = (-value).meters
inline operator fun Meters.times(other: Hertz) = (value * other.value).metersPerSecond // TECHNICALLY Herts are the reciprocal of Seconds
inline operator fun Meters.div(other: TimeUnit) = (value / other.toSeconds().value).metersPerSecond
inline operator fun Kilometers.plus(other: Kilometers) = (value + other.value).km
inline operator fun Kilometers.unaryPlus() = value.km
inline operator fun Kilometers.minus(other: Kilometers) = (value - other.value).km
inline operator fun Kilometers.unaryMinus() = (-value).km
inline operator fun Kilometers.div(other: TimeUnit) = (value / other.toHours().value).kilometersPerHour
inline operator fun Feet.plus(other: Feet) = (value + other.value).feet
inline operator fun Feet.unaryPlus() = value.feet
inline operator fun Feet.minus(other: Feet) = (value - other.value).feet
inline operator fun Feet.unaryMinus() = (-value).feet
inline operator fun Feet.div(other: TimeUnit) = (value / other.toSeconds().value).feetPerSecond
inline operator fun Inches.plus(other: Inches) = (value + other.value).inches
inline operator fun Inches.unaryPlus() = value.inches
inline operator fun Inches.minus(other: Inches) = (value - other.value).inches
inline operator fun Inches.unaryMinus() = (-value).inches
inline operator fun Inches.div(other: TimeUnit) = ((value / 12) / other.toSeconds().value).feetPerSecond
inline operator fun DistanceUnit.times(other: DistanceUnit) = (toMeters().value * other.toMeters().value).m2
inline operator fun MetersPerSecond.plus(other: MetersPerSecond) = (value + other.value).metersPerSecond
inline operator fun MetersPerSecond.unaryPlus() = value.metersPerSecond
inline operator fun MetersPerSecond.minus(other: MetersPerSecond) = (value - other.value).metersPerSecond
inline operator fun MetersPerSecond.unaryMinus() = (-value).metersPerSecond
inline operator fun MetersPerSecond.times(other: TimeUnit) = (value * other.toSeconds().value).m
inline operator fun VelocityUnit.div(other: TimeUnit) = (toMetersPerSecond().value / other.toSeconds().value).metersPerSecondSquared
inline operator fun VelocityUnit.div(other: DistanceUnit) = (toMetersPerSecond().value / other.toMeters().value).hz
inline operator fun FeetPerSecond.plus(other: FeetPerSecond) = (value + other.value).feetPerSecond
inline operator fun FeetPerSecond.unaryPlus() = value.feetPerSecond
inline operator fun FeetPerSecond.minus(other: FeetPerSecond) = (value - other.value).feetPerSecond
inline operator fun FeetPerSecond.unaryMinus() = (-value).feetPerSecond
inline operator fun FeetPerSecond.times(other: TimeUnit) = (value * other.toSeconds().value).feet
inline operator fun KilometersPerHour.plus(other: KilometersPerHour) = (value + other.value).kilometersPerHour
inline operator fun KilometersPerHour.unaryPlus() = value.kilometersPerHour
inline operator fun KilometersPerHour.minus(other: KilometersPerHour) = (value - other.value).kilometersPerHour
inline operator fun KilometersPerHour.unaryMinus() = (-value).kilometersPerHour
inline operator fun KilometersPerHour.times(other: TimeUnit) = (value * other.toHours().value).km
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
inline operator fun AngleUnit.div(other: TimeUnit) = (toRotations().value / other.toSeconds().value).rps
inline operator fun AngleUnit.times(other: FrequencyUnit) = (toRotations().value / other.toMillis().toSeconds().value).rps
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
inline operator fun Minutes.plus(other: Minutes) = (value + other.value).minutes
inline operator fun Minutes.unaryPlus() = value.minutes
inline operator fun Minutes.minus(other: Minutes) = (value - other.value).minutes
inline operator fun Minutes.unaryMinus() = (-value).minutes
inline operator fun Hours.plus(other: Hours) = (value + other.value).hours
inline operator fun Hours.unaryPlus() = value.hours
inline operator fun Hours.minus(other: Hours) = (value - other.value).hours
inline operator fun Hours.unaryMinus() = (-value).hours
inline operator fun Milliseconds.plus(other: Milliseconds) = (value + other.value).milliseconds
inline operator fun Milliseconds.unaryPlus() = value.milliseconds
inline operator fun Milliseconds.minus(other: Milliseconds) = (value - other.value).milliseconds
inline operator fun Milliseconds.unaryMinus() = (-value).milliseconds
inline operator fun Hertz.plus(other: Hertz) = (value + other.value).hertz
inline operator fun Hertz.unaryPlus() = value.hertz
inline operator fun Hertz.minus(other: Hertz) = (value - other.value).hertz
inline operator fun Hertz.unaryMinus() = (-value).hertz
inline operator fun Volts.plus(other: Volts) = (value + other.value).volts
inline operator fun Volts.unaryPlus() = value.volts
inline operator fun Volts.minus(other: Volts) = (value - other.value).volts
inline operator fun Volts.unaryMinus() = (-value).volts
inline operator fun Amps.plus(other: Amps) = (value + other.value).amps
inline operator fun Amps.unaryPlus() = value.amps
inline operator fun Amps.minus(other: Amps) = (value - other.value).amps
inline operator fun Amps.unaryMinus() = (-value).amps
inline operator fun Volts.times(other: Amps) = (value * other.value).watts
inline operator fun Amps.times(other: Volts) = (value * other.value).watts
inline operator fun Watts.plus(other: Watts) = (value + other.value).watts
inline operator fun Watts.unaryPlus() = value.watts
inline operator fun Watts.minus(other: Watts) = (value - other.value).watts
inline operator fun Watts.unaryMinus() = (-value).watts
inline operator fun Watts.div(other: Amps) = (value / other.value).volts
inline operator fun Watts.div(other: Volts) = (value / other.value).amps

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
