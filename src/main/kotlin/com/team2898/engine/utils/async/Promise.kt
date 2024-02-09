package com.team2898.engine.utils.async

import com.team2898.engine.utils.AggregateException
import edu.wpi.first.wpilibj.event.BooleanEvent
import edu.wpi.first.wpilibj.event.EventLoop

class Promise<T>(private val fn: (resolve: (value: T) -> Unit, reject: (error: Throwable) -> Unit) -> Unit) : Thenable<T> {
    private var resolvedWith: T? = null
    private var rejectedWith: Throwable? = null
    private var currentState: PromiseState = PromiseState.NOT_RUN
    private var listenerApplied = false
    private val onResolved = EventTarget<T>()
    private val onRejected = EventTarget<Throwable>()
    fun hasResolved(loop: EventLoop) = BooleanEvent(loop) { currentState == PromiseState.RESOLVED }
    fun hasRejected(loop: EventLoop) = BooleanEvent(loop) { currentState == PromiseState.REJECTED }

    init {
        run()
    }
    private fun run() {
        if (currentState != PromiseState.NOT_RUN) return
        currentState = PromiseState.WAITING
        fn({d -> resolve(d)}, {e -> reject(e)})
    }
    private fun resolve(value: T) {
        if (currentState != PromiseState.WAITING) return
        currentState = PromiseState.RESOLVED
        resolvedWith = value
        onResolved.dispatch(value)
    }
    private fun reject(error: Throwable) {
        if (currentState != PromiseState.WAITING) return
        currentState = PromiseState.REJECTED
        rejectedWith = error
        onRejected.dispatch(error)
    }
    override fun<N> then(success: (value: T) -> Promise<N>)
        = Promise { rs, rj ->
            onResolved.addListener { value ->
                try {
                    val x = success(value)
                    x.onResolved.addListener { value2 -> rs(value2) }
                    x.onRejected.addListener(rj)
                } catch (e: Throwable) {
                    rj(e)
                }
            }
            onRejected.addListener { error -> rj(error) }
        }
    override fun <N> then(
        success: (value: T) -> Promise<N>,
        failure: (error: Throwable) -> Promise<N>
    ): Promise<N> {
        return Promise { rs, rj ->
            onResolved.addListener { value ->
                try {
                    val x = success(value)
                    x.onResolved.addListener { value2 -> rs(value2) }
                    x.onRejected.addListener(rj)
                } catch (e: Throwable) {rj(e)}
            }
            onRejected.addListener { error ->
                try {
                    val x = failure(error)
                    x.onResolved.addListener { value2 -> rs(value2) }
                    x.onRejected.addListener(rj)
                } catch (e: Throwable) {rj(e)}
            }
        }
    }

    override fun catch(failure: (error: Throwable) -> Promise<T>): Promise<T> {
        return Promise { rs, rj ->
            onResolved.addListener { value -> rs(value) }
            onRejected.addListener { error ->
                try {
                    val x = failure(error)
                    x.onResolved.addListener { value2 -> rs(value2) }
                    x.onRejected.addListener(rj)
                } catch (e: Throwable) {
                    rj(e)
                }
            }
        }
    }
    companion object {
        enum class PromiseState {
            NOT_RUN, WAITING, RESOLVED, REJECTED
        }
        enum class PromiseSettledState {
            RESOLVED, REJECTED
        }

        /**
         * Returns a Promise that resolves when any promise passed resolves, and rejects when all promises passed reject.
         */
        fun <T> any(iter: Set<Promise<T>>): Promise<T> {
            val icpy = iter.map { v -> v }
            val l = icpy.size
            var rjc = 0
            val rejections = mutableListOf<Throwable>()
            return Promise { rs, rj ->
                if (l == 0) rj(IllegalArgumentException("Iterable passed to Promise.any was empty"))
                else icpy.forEach { item ->
                    item.onResolved.addListener(rs)
                    item.onRejected.addListener { error ->
                        rjc++
                        rejections.add(error)
                        if (rjc >= l) rj(AggregateException(null, null, *rejections.toTypedArray()))
                    }
                }
            }
        }

        /**
         * Returns a Promise that resolves when all promises passed resolve, and rejects when any promise rejects.
         */
        fun <T> all(iter: Set<Promise<T>>): Promise<Set<T>> {
            val icpy = iter.map { v -> v }
            val l = icpy.size
            val resolutions = mutableSetOf<T>()
            return Promise { rs, rj ->
                if (l == 0) rs(setOf())
                else icpy.forEach { item ->
                    item.onResolved.addListener {
                        resolutions.add(it)
                        if (resolutions.size >= l) rs(resolutions)
                    }
                    item.onRejected.addListener(rj)
                }
            }
        }

        /**
         * Returns a Promise that adopts the state of the first promise to resolve or reject.
         */
        fun <T> race(iter: Set<Promise<T>>): Promise<T> {
            return Promise { rs, rj ->
                iter.forEach {
                    it.onResolved.addListener(rs)
                    it.onRejected.addListener(rj)
                }
            }
        }

        /**
         * Returns a Promise that resolves when all promises resolve or reject, with an iterable of the eventual states.
         */
        fun <T> allSettled(iter: Set<Promise<T>>): Promise<Set<PromiseFulfillment<T>>> {
            val fulfillments = mutableSetOf<PromiseFulfillment<T>>()
            return Promise { rs, _ ->
                if (iter.isEmpty()) rs(setOf())
                else iter.forEach { item ->
                    item.onResolved.addListener { value ->
                        fulfillments.add(object : PromiseFulfillment<T> {
                            override val status = PromiseSettledState.RESOLVED
                            override val value = value
                            override val reason = null
                        })
                        if (fulfillments.size >= iter.size) rs(fulfillments)
                    }
                    item.onRejected.addListener { error ->
                        fulfillments.add(object : PromiseFulfillment<T> {
                            override val status = PromiseSettledState.REJECTED
                            override val value = null
                            override val reason = error
                        })
                        if (fulfillments.size >= iter.size) rs(fulfillments)
                    }
                }
            }
        }

        /**
         * Returns a Promise that resolves with the values from all the promises in the iterable that resolved.
         * Rejections are ignored.
         */
        fun <T> filter(iter: Set<Promise<T>>): Promise<Set<T>> {
            val fulfillments = mutableSetOf<T>()
            return Promise { rs, _ ->
                if (iter.isEmpty()) rs(setOf())
                else iter.forEach {
                    it.onResolved.addListener { value ->
                        fulfillments.add(value)
                        if (fulfillments.size >= iter.size) rs(fulfillments)
                    }
                }
            }
        }

        /**
         * Returns a Promise that resolves immediately with the given value.
         */
        fun <T> resolve(value: T): Promise<T> {
            return Promise { resolve, _ -> resolve(value) }
        }

        /**
         * Returns a Promise that rejects immediately with the given reason.
         */
        fun reject(error: Throwable): Promise<Nothing> {
            return Promise { _, reject -> reject(error) }
        }

        /**
         * Returns an object containing a new Promise and two functions to resolve or reject it.
         */
        fun <T> withResolvers(): ResolversObject<T> {
            val p = Promise<T>{ _, _ -> }
            return object : ResolversObject<T> {
                override val promise = p
                override val resolve: (value: T) -> Unit = {value -> p.resolve(value)}
                override val reject: (error: Throwable) -> Unit = {error -> p.reject(error)}
            }
        }
        interface PromiseFulfillment<T> {
            val status: PromiseSettledState
            val value: T?
            val reason: Throwable?
        }
        interface ResolversObject<T> {
            val promise: Promise<T>
            val resolve: (value: T) -> Unit
            val reject: (error: Throwable) -> Unit
        }
    }
}