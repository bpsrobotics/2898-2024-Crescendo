package com.team2898.engine.utils

import kotlin.random.Random



/**
 * A class that is unique until you run out of String length.
 * Similar to [the JavaScript version](https://developer.mozilla.org/en-US/docs/Web/JavaScript/Reference/Global_Objects/Symbol).
 */
class Symbol(val description: String?) {
    private constructor(id: String, description: String?) : this(description) {
        this.id = id
    }
    constructor() : this(null)
    private var id: String
    init {
        var l = 32
        id = makeid(64, l)
        var tries = 0
        while (symbolRegistry.contains(id)) {
            id = makeid(64, l)
            tries++
            if (tries % 10 == 0) l++
        }
        symbolRegistry[id] = description
        if (description is String) {
            if (descriptionMap.contains(description)) descriptionMap[description]?.add(id)
            else descriptionMap[description] = mutableListOf(id)
        }
    }
    override operator fun equals(other: Any?): Boolean {
        return other is Symbol && other.id == id
    }

    override fun hashCode(): Int {
        var result = description?.hashCode() ?: 0
        result = 31 * result + id.hashCode()
        return result
    }
    companion object {
        fun forDescription(description: String): List<Symbol> {
            return try {
                descriptionMap[description]!!.toList().map { id: String -> Symbol(id, description) }
            } catch (_: Exception) {
                listOf()
            }
        }
        private val symbolRegistry: MutableMap<String, String?> = mutableMapOf()
        private val descriptionMap: MutableMap<String, MutableList<String>> = mutableMapOf()
        fun makeid(base: Int, len: Int): String {
            val abc = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz+/".slice(0..<base)
            var s = ""
            for (i in 1..<len) {
                s += abc[Random.nextInt(len)]
            }
            return s
        }
    }
}