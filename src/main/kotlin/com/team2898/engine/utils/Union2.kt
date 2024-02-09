package com.team2898.engine.utils

class Union2<A, B> (private var valueA: A?, private var valueB: B?) {
    private var current: Int = 0
    fun setA(value: A) {
        valueA = value
        valueB = null
        current = 1
    }
    fun setB(value: B) {
        valueA = null
        valueB = value
        current = 2
    }
    fun<T> using(callbackIfA: (value: A) -> T, callbackIfB: (value: B) -> T): T {
        return when (current) {
            1 -> callbackIfA(valueA!!)
            2 -> callbackIfB(valueB!!)
            else -> throw Exception("not set")
        }
    }

    init {
        current = when {
            valueA != null -> 1
            valueB != null -> 2
            else -> throw Exception("not set")
        }
    }
}