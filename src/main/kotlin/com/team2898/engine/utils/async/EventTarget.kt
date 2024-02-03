package com.team2898.engine.utils.async

import com.team2898.engine.utils.Symbol

class EventTarget<DataType> {
    private val listeners: MutableMap<Symbol, (ev: DataType) -> Unit> = mutableMapOf()
    private val onceListeners: MutableMap<Symbol, (ev: DataType) -> Unit> = mutableMapOf()
    fun addListener(fn: (ev: DataType) -> Unit): Symbol {
        val s = Symbol()
        listeners[s] = fn
        return s
    }
    fun addOnceListener(fn: (ev: DataType) -> Unit): Symbol {
        val s = Symbol()
        onceListeners[s] = fn
        return s
    }
    fun removeListener(identifier: Symbol) {
        listeners.remove(identifier)
    }
    fun dispatch(event: DataType) {
        listeners.forEach { (_, u) ->
            try { u(event) }
            catch (e: Throwable) {
                println("Uncaught in event listener: $e")
            }
        }
        onceListeners.forEach { (t, u) ->
            try { u(event) }
            catch (e: Throwable) {
                println("Uncaught in event listener: $e")
            }
            onceListeners.remove(t)
        }
    }

}