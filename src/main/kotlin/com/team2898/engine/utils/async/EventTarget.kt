package com.team2898.engine.utils.async

import com.team2898.engine.utils.Symbol

typealias EventHandler<DataType, TargetType> = (ev: Event<DataType, TargetType>) -> Unit
@Suppress("unused")
class EventTarget<DataType, TargetType>(private val target: TargetType) {
    private val listeners: MutableMap<Symbol, EventHandler<DataType, TargetType>> = mutableMapOf()
    private val onceListeners: MutableMap<Symbol, EventHandler<DataType, TargetType>> = mutableMapOf()
    operator fun invoke(fn: EventHandler<DataType, TargetType>) = addListener(fn)
    operator fun invoke(event: DataType) = dispatch(event)
    fun addListener(fn: EventHandler<DataType, TargetType>): Symbol {
        val s = Symbol()
        listeners[s] = fn
        return s
    }
    fun addOnceListener(fn: EventHandler<DataType, TargetType>): Symbol {
        val s = Symbol()
        onceListeners[s] = fn
        return s
    }
    fun removeListener(identifier: Symbol) {
        listeners.remove(identifier)
        onceListeners.remove(identifier)
    }
    fun next() = Promise {resolve, _ -> addOnceListener { resolve(it) }}
    fun dispatch(data: DataType, preventable: Boolean = false): Boolean {
        var prevented = false
        val evt = object : Event<DataType, TargetType> {
            override fun prevent() {
                if (preventable) {
                    prevented = true
                } else throw Exception("Cannot prevent the default action for this event")
            }
            override val prevented get() = prevented
            override val preventable = preventable
            override val data = data
            override val target = this@EventTarget.target
        }
        listeners.forEach { (_, u) ->
            try { u(evt) }
            catch (e: Throwable) {
                println("Uncaught in event listener: $e")
            }
        }
        onceListeners.forEach { (t, u) ->
            try { u(evt) }
            catch (e: Throwable) {
                println("Uncaught in event listener: $e")
            }
            onceListeners.remove(t)
        }
        return !prevented
    }
}