package com.team2898.engine.utils

import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

class Vector(
    cps: Map<Basis, Double>
) {
    val components = cps.toMutableMap()
    constructor(x: Double, y: Double) : this(mutableMapOf(Pair(Bases.AxisX, x), Pair(Bases.AxisY, y)))
    constructor(x: Int, y: Double) : this(x.toDouble(), y)
    constructor(x: Double, y: Int) : this(x, y.toDouble())
    constructor(x: Int, y: Int) : this(x.toDouble(), y.toDouble())
    constructor(x: Double, y: Double, z: Double) : this(mutableMapOf(
        Pair(Bases.AxisX, x),
        Pair(Bases.AxisY, y),
        Pair(Bases.AxisZ, z)
    ))
    var magnitude get() = sqrt(components.values.map { v -> v * v }.reduceOrNull { acc, v -> acc + v } ?: 0.0)
        set(value) {
            normalize()
            timesAssign(value)
        }
    var magnitudeSquared get() = components.values.map { v -> v * v }.reduceOrNull { acc, v -> acc + v } ?: 0.0
        set(value) {
            normalize()
            timesAssign(sqrt(value))
        }
    fun clone(): Vector {
        val copy = mutableMapOf<Basis, Double>()
        components.toMap(copy)
        return Vector(copy)
    }
    operator fun plus(other: Vector): Vector {
        val copy = mutableMapOf<Basis, Double>()
        components.toMap(copy)
        other.components.forEach { e ->
            if (copy.contains(e.key)) copy[e.key] = copy[e.key]!! + e.value
            else copy[e.key] = e.value
        }
        return Vector(copy)
    }
    operator fun plusAssign(other: Vector) {
        other.components.forEach { e ->
            if (components.contains(e.key)) components[e.key] = components[e.key]!! + e.value
            else components[e.key] = e.value
        }
    }
    operator fun minus(other: Vector): Vector {
        val copy = mutableMapOf<Basis, Double>()
        components.toMap(copy)
        other.components.forEach { e ->
            if (copy.contains(e.key)) copy[e.key] = copy[e.key]!! - e.value
            else copy[e.key] = e.value
        }
        return Vector(copy)
    }
    operator fun minusAssign(other: Vector) {
        other.components.forEach { e ->
            if (components.contains(e.key)) components[e.key] = components[e.key]!! - e.value
            else components[e.key] = e.value
        }
    }

    /**
     * returns a multivector
     *
     * see [https://www.youtube.com/watch?v=htYh-Tq7ZBI](https://www.youtube.com/watch?v=htYh-Tq7ZBI)
     */
    operator fun times(other: Vector): Vector {
        val cs = mutableMapOf<Basis, Double>()
        components.forEach { (t, u) ->
            other.components.forEach { (t2, u2) ->
                val bv = t.bivectors[t2] ?: t2.bivectors[t] ?: if (t == t2) Bases.Axis1 else Bases.Bivector(t, t2)
                if (cs.contains(bv)) cs[bv] = cs[bv]!! + u * u2
                else cs[bv] = u * u2
            }
        }
        return Vector(cs)
    }
    operator fun timesAssign(other: Vector) {
        components.forEach { (t, u) ->
            other.components.forEach { (t2, u2) ->
                val bv = t.bivectors[t2] ?: t2.bivectors[t] ?: if (t == t2) Bases.Axis1 else Bases.Bivector(t, t2)
                if (components.contains(bv)) components[bv] = components[bv]!! + u * u2
                else components[bv] = u * u2
            }
        }
    }
    operator fun times(other: Double): Vector {
        return Vector(components.mapValues { e -> e.value * other })
    }
    operator fun timesAssign(other: Double) {
        components.forEach { (t, u) -> components[t] = u * other }
    }
    operator fun div(other: Vector): Vector {
        val cs = mutableMapOf<Basis, Double>()
        components.forEach { (t, u) ->
            other.components.forEach { (t2, u2) ->
                val bv = t.bivectors[t2] ?: t2.bivectors[t] ?: if (t == t2) Bases.Axis1 else Bases.Bivector(t, t2)
                if (cs.contains(bv)) cs[bv] = cs[bv]!! + u / u2
                else cs[bv] = u / u2
            }
        }
        return Vector(cs)
    }
    operator fun divAssign(other: Vector) {
        components.forEach { (t, u) ->
            other.components.forEach { (t2, u2) ->
                val bv = t.bivectors[t2] ?: t2.bivectors[t] ?: if (t == t2) Bases.Axis1 else Bases.Bivector(t, t2)
                if (components.contains(bv)) components[bv] = components[bv]!! + u / u2
                else components[bv] = u / u2
            }
        }
    }
    operator fun div(other: Double): Vector {
        return Vector(components.mapValues { e -> e.value / other })
    }
    operator fun divAssign(other: Double) {
        components.forEach { (t, u) -> components[t] = u / other }
    }
    operator fun rem(other: Vector): Vector {
        val copy = mutableMapOf<Basis, Double>()
        components.toMap(copy)
        other.components.forEach { e ->
            if (copy.contains(e.key)) copy[e.key] = copy[e.key]!! % e.value
            else copy[e.key] = e.value
        }
        return Vector(copy)
    }
    operator fun remAssign(other: Vector) {
        other.components.forEach { e ->
            if (components.contains(e.key)) components[e.key] = components[e.key]!! % e.value
            else components[e.key] = e.value
        }
    }
    operator fun rem(other: Double): Vector {
        return Vector(components.mapValues { entry -> entry.value % other })
    }
    operator fun remAssign(other: Double) {
        components.forEach { (t, u) -> components[t] = u % other }
    }
    override operator fun equals(other: Any?): Boolean {
        return other is Vector && components == other.components
    }
    fun normalized() = this.div(this.magnitude)
    fun normalize(): Vector {
        this.divAssign(this.magnitude)
        return this
    }
    operator fun get(basis: Basis): Double = components.getOrDefault(basis, 0.0)
    operator fun set(basis: Basis, value: Double) {
        components[basis] = value
    }

    /**
     * Returns the Hadamard product (componentwise multiplication) with another vector
     */
    fun hadamard(other: Vector): Vector {
        val copy = mutableMapOf<Basis, Double>()
        components.toMap(copy)
        other.components.forEach { e ->
            components[e.key] = (components[e.key] ?: 0.0) * e.value
        }
        return Vector(copy)
    }
    fun translate(vararg vs: Vector): Vector {
        vs.forEach { v -> this.plusAssign(v) }
        return this
    }
    fun translated(vararg vs: Vector) = this.clone().translate(*vs)
    fun dilate(factor: Double): Vector {
        timesAssign(factor)
        return this
    }
    fun dilated(factor: Double) = times(factor)
    fun dilate(factor: Double, origin: Vector): Vector {
        minusAssign(origin)
        timesAssign(factor)
        plusAssign(origin)
        return this
    }
    fun dilated(factor: Double, origin: Vector) = origin + factor * (this - origin)

    override fun hashCode(): Int {
        return components.hashCode()
    }

    /**
     * Returns a new Vector with the given bases extracted (or 0 if the vector does not have a basis)
     */
    fun extract(bases: Set<Basis>): Vector {
        val cs = mutableMapOf<Basis, Double>()
        bases.forEach { basis -> cs[basis] = get(basis) }
        return Vector(cs)
    }
    fun extract(bases: List<Basis>): Vector {
        val cs = mutableMapOf<Basis, Double>()
        bases.forEach { basis -> cs[basis] = get(basis) }
        return Vector(cs)
    }
    fun cross(other: Vector) = cross(this, other)
    fun dot(other: Vector) = dot(this, other)

    companion object {
        val zero get() = Vector(mapOf())
        /**
         * Returns the dot (scalar) product of two vectors
         */
        fun dot(a: Vector, b: Vector): Double {
            var dp = 0.0
            a.components.forEach { (t, u) -> if (b.components.contains(t)) dp += u * (b.components[t] ?: 0.0)}
            return dp
        }

        /**
         * Returns the cross product of two vectors
         */
        fun cross(a: Vector, b: Vector): Vector {
            if (a.components.keys == b.components.keys && a.components.keys.size == 3) {
                val bases = a.components.keys.toList()
                val (i, j, k) = bases
                return Vector(mapOf(
                    Pair(i, a[j] * b[k] - a[k] * b[j]),
                    Pair(j, a[k] * b[i] - a[i] * b[k]),
                    Pair(k, a[i] * b[j] - a[j] * b[i])
                ))
            } else throw Exception("Vectors must be 3D or 7D and share the same basis vectors")
        }
        fun midpoint(a: Vector, b: Vector): Vector {
            val bs = mutableSetOf<Basis>()
            bs.addAll(a.components.keys)
            bs.addAll(b.components.keys)
            val copy = mutableMapOf<Basis, Double>()
            bs.forEach { basis -> copy[basis] = (a[basis]) + 0.5 * (b[basis] - a[basis]) }
            return Vector(copy)
        }
        fun lerp(a: Vector, b: Vector, t: Double) = multimap(a, b) { bs: Basis, _: List<Double> -> a[bs] + t * (b[bs] - a[bs]) }
        fun offX(angle: Double) = Vector(cos(angle), sin(angle))
        fun offY(angle: Double) = Vector(sin(angle), cos(angle))
        fun multimap(vararg vs: Vector, fn: (Basis, List<Double>) -> Double): Vector {
            val bs = mutableSetOf<Basis>()
            vs.forEach { v -> bs.addAll(v.components.keys) }
            val copy = mutableMapOf<Basis, Double>()
            bs.forEach { b -> copy[b] = fn(b, vs.map { v -> v[b] }) }
            return Vector(copy)
        }
        interface Plane {
            val i: Basis
            val j: Basis
            val perpendicular: Boolean
            val normal: Boolean
        }
        interface Basis {
            val identifier: Symbol
            val bivectors: Map<Basis, Basis>
        }

        object Bases {
            object ImaginaryAxis : Basis {
                override val identifier: Symbol = Symbol("Basis.Im")
                override val bivectors: Map<Basis, Basis> = mapOf()
            }
            object Axis1 : Basis {
                override val identifier: Symbol = Symbol("Basis.Re")
                override val bivectors: Map<Basis, Basis> = mapOf(
                    Pair(AxisX, AxisX),
                    Pair(AxisY, AxisY),
                    Pair(AxisZ, AxisZ),
                    Pair(XYBivector, XYBivector),
                    Pair(YZBivector, YZBivector),
                    Pair(XZBivector, XZBivector),
                    Pair(XYZTrivector, XYZTrivector),
                    Pair(Axis1, Axis1)
                )
            }
            object AxisX : Basis {
                override val identifier: Symbol = Symbol("Basis.X")
                override val bivectors: Map<Basis, Basis> = mapOf(
                    Pair(AxisX, Axis1),
                    Pair(AxisY, XYBivector),
                    Pair(AxisZ, XZBivector),
                    Pair(XYBivector, AxisY),
                    Pair(YZBivector, XYZTrivector),
                    Pair(XZBivector, AxisZ),
                    Pair(XYZTrivector, YZBivector),
                    Pair(Axis1, AxisX)
                )
            }
            object AxisY : Basis {
                override val identifier: Symbol = Symbol("Basis.Y")
                override val bivectors: Map<Basis, Basis> = mapOf(
                    Pair(AxisX, XYBivector),
                    Pair(AxisY, Axis1),
                    Pair(AxisZ, YZBivector),
                    Pair(XYBivector, AxisX),
                    Pair(YZBivector, AxisZ),
                    Pair(XZBivector, XYZTrivector),
                    Pair(XYZTrivector, XZBivector),
                    Pair(Axis1, AxisY)
                )
            }
            object AxisZ : Basis {
                override val identifier: Symbol = Symbol("Basis.Z")
                override val bivectors: Map<Basis, Basis> = mapOf(
                    Pair(AxisX, XZBivector),
                    Pair(AxisY, YZBivector),
                    Pair(AxisZ, Axis1),
                    Pair(XYBivector, XYZTrivector),
                    Pair(YZBivector, AxisY),
                    Pair(XZBivector, AxisX),
                    Pair(XYZTrivector, XYBivector),
                    Pair(Axis1, AxisZ)
                )
            }
            object XYPlane : Plane {
                override val i: Basis = AxisX
                override val j: Basis = AxisY
                override val perpendicular: Boolean = true
                override val normal: Boolean = true
            }
            object YZPlane : Plane {
                override val i: Basis = AxisY
                override val j: Basis = AxisZ
                override val perpendicular: Boolean = true
                override val normal: Boolean = true
            }
            object XZPlane : Plane {
                override val i: Basis = AxisX
                override val j: Basis = AxisZ
                override val perpendicular: Boolean = true
                override val normal: Boolean = true
            }
            object XYBivector : Basis {
                override val identifier: Symbol = Symbol("Basis.XY")
                override val bivectors: Map<Basis, Basis> = mapOf(
                    Pair(AxisX, AxisY),
                    Pair(AxisY, AxisX),
                    Pair(AxisZ, XYZTrivector),
                    Pair(XYBivector, Axis1),
                    Pair(YZBivector, XZBivector),
                    Pair(XZBivector, YZBivector),
                    Pair(XYZTrivector, AxisZ),
                    Pair(Axis1, XYBivector)
                )
            }
            object YZBivector : Basis {
                override val identifier: Symbol = Symbol("Basis.YZ")
                override val bivectors: Map<Basis, Basis> = mapOf(
                    Pair(AxisX, XYZTrivector),
                    Pair(AxisY, AxisZ),
                    Pair(AxisZ, AxisY),
                    Pair(XYBivector, XZBivector),
                    Pair(YZBivector, Axis1),
                    Pair(XZBivector, XYBivector),
                    Pair(XYZTrivector, AxisX),
                    Pair(Axis1, YZBivector)
                )
            }
            object XZBivector : Basis {
                override val identifier: Symbol = Symbol("Basis.XZ")
                override val bivectors: Map<Basis, Basis> = mapOf(
                    Pair(AxisX, AxisZ),
                    Pair(AxisY, XYZTrivector),
                    Pair(AxisZ, AxisX),
                    Pair(XYBivector, YZBivector),
                    Pair(YZBivector, XYBivector),
                    Pair(XZBivector, Axis1),
                    Pair(XYZTrivector, AxisY),
                    Pair(Axis1, XZBivector)
                )
            }
            object XYZTrivector : Basis {
                override val identifier: Symbol = Symbol("Basis.XYZ")
                override val bivectors: Map<Basis, Basis> = mapOf(
                    Pair(AxisX, YZBivector),
                    Pair(AxisY, XZBivector),
                    Pair(AxisZ, XYBivector),
                    Pair(XYBivector, AxisZ),
                    Pair(YZBivector, AxisX),
                    Pair(XZBivector, AxisY),
                    Pair(XYZTrivector, Axis1),
                    Pair(Axis1, XYZTrivector)
                )
            }
            class Bivector(a: Basis, b: Basis) : Basis {
                override val identifier: Symbol = Symbol("${a.identifier.description ?: "unnamed basis vector"} * ${b.identifier.description ?: "unnamed basis vector"}")
                override val bivectors: Map<Basis, Basis> = mapOf()
            }
        }
    }
    operator fun Double.times(other: Vector) = other * this
}