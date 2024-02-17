package com.team2898.engine.utils

class AggregateException(message: String?, primary: Exception?, vararg others: Throwable) : Exception(message, primary) {
    init {
        others.forEach {err -> addSuppressed(err)}
    }
}