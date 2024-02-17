package com.team2898.engine.utils.async

interface Thenable<T> {
    fun<N> then(success: (value: T) -> Promise<N>): Promise<N>
    fun<N> then(success: (value: T) -> Promise<N>, failure: (error: Throwable) -> Promise<N>) : Promise<N>
    fun catch(failure: (error: Throwable) -> Promise<T>): Promise<T>
}