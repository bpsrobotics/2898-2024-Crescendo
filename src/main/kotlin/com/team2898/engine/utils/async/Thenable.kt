package com.team2898.engine.utils.async

import com.team2898.engine.utils.Union2

interface Thenable<T> {
    fun<N> then(success: (value: T) -> Union2<N, Promise<N>>): Promise<N>
    fun<N> then(success: (value: T) -> Union2<N, Promise<N>>, failure: (error: Throwable) -> Union2<N, Promise<N>>) : Promise<N>
    fun catch(failure: (error: Throwable) -> Union2<T, Promise<T>>): Promise<T>
}