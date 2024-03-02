package com.team2898.engine.utils.async

interface Event<DataType, TargetType> {
    fun prevent()
    val preventable: Boolean
    val prevented: Boolean
    val data: DataType
    val target: TargetType
}