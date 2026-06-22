package com.signalgen.companion.data.net

import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.cancelAndJoin
import kotlinx.coroutines.channels.Channel
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.SharedFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asSharedFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext
import java.io.BufferedReader
import java.io.InputStreamReader
import java.io.OutputStream
import java.net.InetSocketAddress
import java.net.Socket
import java.nio.charset.StandardCharsets

/**
 * Raw-TCP transport to the board (plan §4.0/D2, port 3333). One persistent
 * bidirectional socket, NDJSON newline framing, zero okhttp/ktor — just
 * [java.net.Socket] on [Dispatchers.IO].
 *
 * Responsibilities:
 *  - connect/reconnect to a host:port (single client; the board drops any prior).
 *  - read loop: [BufferedReader.readLine] -> decode -> emit on [inbound] flow.
 *  - write API: enqueue an NDJSON line; a writer coroutine drains it.
 *  - coalesce/debounce rapid RPM & sweep sends (D10) — latest-only, ~60ms window.
 *
 * Lifecycle: [connect] starts read+write+coalescer coroutines on [scope];
 * [disconnect] tears them down and closes the socket. Safe to re-[connect].
 */
class TcpClient(
    private val scope: CoroutineScope,
    private val coalesceWindowMs: Long = 60L,
) {

    enum class State { DISCONNECTED, CONNECTING, CONNECTED }

    private val _state = MutableStateFlow(State.DISCONNECTED)
    val state: StateFlow<State> = _state.asStateFlow()

    /** Decoded inbound frames. replay=0 — late collectors only see new frames. */
    private val _inbound = MutableSharedFlow<NdjsonProtocol.Inbound>(
        replay = 0,
        extraBufferCapacity = 256,
    )
    val inbound: SharedFlow<NdjsonProtocol.Inbound> = _inbound.asSharedFlow()

    /**
     * Outbound line queue. Buffered + conflated-on-overflow at the channel level is
     * avoided; instead we keep ordering and rely on the explicit coalescers below
     * for the high-rate frames. UNLIMITED so control bursts are never lost.
     */
    private val outQueue = Channel<String>(Channel.UNLIMITED)

    /**
     * Latest-only holders for the two high-rate frame kinds. A single coalescer
     * coroutine samples them every [coalesceWindowMs] and forwards only the most
     * recent — matching the firmware's "process one inbound line per loop" + queue
     * coalescing (D10). Low-rate frames (start/stop/select/...) bypass this and go
     * straight to [outQueue] for immediate, ordered delivery.
     */
    private val pendingRpm = MutableStateFlow<String?>(null)
    private val pendingSweep = MutableStateFlow<String?>(null)

    private var host: String = ""
    private var port: Int = 3333

    private var socket: Socket? = null
    private var output: OutputStream? = null

    private var readJob: Job? = null
    private var writeJob: Job? = null
    private var coalesceJob: Job? = null

    @Volatile
    private var running = false

    /**
     * Connect to [host]:[port]. Tears down any prior connection first. Idempotent
     * target — calling again with a new host reconnects there.
     */
    suspend fun connect(host: String, port: Int = 3333) {
        disconnect()
        this.host = host
        this.port = port
        running = true
        _state.value = State.CONNECTING
        startCoalescer()
        startWriter()
        startReader()
    }

    suspend fun disconnect() {
        running = false
        readJob?.cancelAndJoin(); readJob = null
        writeJob?.cancelAndJoin(); writeJob = null
        coalesceJob?.cancelAndJoin(); coalesceJob = null
        closeSocketQuietly()
        pendingRpm.value = null
        pendingSweep.value = null
        _state.value = State.DISCONNECTED
    }

    // ---------------------------------------------------------------------------
    // Outbound API
    // ---------------------------------------------------------------------------

    /** Enqueue a raw NDJSON line (newline appended by the writer). Immediate path. */
    fun send(line: String) {
        outQueue.trySend(line)
    }

    /** Coalesced RPM send — only the latest within a window reaches the socket. */
    fun sendRpmCoalesced(line: String) {
        pendingRpm.value = line
    }

    /** Coalesced sweep send — only the latest within a window reaches the socket. */
    fun sendSweepCoalesced(line: String) {
        pendingSweep.value = line
    }

    // ---------------------------------------------------------------------------
    // Coroutines
    // ---------------------------------------------------------------------------

    private fun startCoalescer() {
        coalesceJob = scope.launch(Dispatchers.IO) {
            while (isActive && running) {
                kotlinx.coroutines.delay(coalesceWindowMs)
                pendingRpm.value?.let { latest ->
                    pendingRpm.value = null
                    outQueue.trySend(latest)
                }
                pendingSweep.value?.let { latest ->
                    pendingSweep.value = null
                    outQueue.trySend(latest)
                }
            }
        }
    }

    private fun startWriter() {
        writeJob = scope.launch(Dispatchers.IO) {
            while (isActive && running) {
                val line = try {
                    outQueue.receive()
                } catch (e: Exception) {
                    break
                }
                val out = output ?: continue
                try {
                    val bytes = (line + "\n").toByteArray(StandardCharsets.UTF_8)
                    synchronized(out) {
                        out.write(bytes)
                        out.flush()
                    }
                } catch (e: Exception) {
                    // write failed -> socket is dead; the read loop will reconnect.
                    closeSocketQuietly()
                }
            }
        }
    }

    private fun startReader() {
        readJob = scope.launch(Dispatchers.IO) {
            var backoffMs = 500L
            while (isActive && running) {
                try {
                    openSocket()
                    backoffMs = 500L
                    _state.value = State.CONNECTED
                    val reader = BufferedReader(
                        InputStreamReader(socket!!.getInputStream(), StandardCharsets.UTF_8)
                    )
                    while (isActive && running) {
                        val line = reader.readLine() ?: break // EOF -> peer closed
                        NdjsonProtocol.decode(line)?.let { _inbound.emit(it) }
                    }
                } catch (e: Exception) {
                    // connect/read error -> fall through to backoff + retry.
                } finally {
                    closeSocketQuietly()
                    if (running) _state.value = State.CONNECTING
                }
                if (!running) break
                kotlinx.coroutines.delay(backoffMs)
                backoffMs = (backoffMs * 2).coerceAtMost(5000L)
            }
        }
    }

    private suspend fun openSocket() {
        withContext(Dispatchers.IO) {
            val s = Socket()
            s.tcpNoDelay = true
            s.keepAlive = true
            s.connect(InetSocketAddress(host, port), CONNECT_TIMEOUT_MS)
            socket = s
            output = s.getOutputStream()
        }
    }

    private fun closeSocketQuietly() {
        try { output?.flush() } catch (_: Exception) {}
        try { socket?.close() } catch (_: Exception) {}
        socket = null
        output = null
    }

    companion object {
        private const val CONNECT_TIMEOUT_MS = 6000
    }
}
