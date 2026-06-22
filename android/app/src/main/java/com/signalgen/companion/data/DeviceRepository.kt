package com.signalgen.companion.data

import android.content.Context
import com.signalgen.companion.data.net.NdjsonProtocol
import com.signalgen.companion.data.net.TcpClient
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.async
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.SharedFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asSharedFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.first
import kotlinx.coroutines.flow.launchIn
import kotlinx.coroutines.flow.onEach
import kotlinx.coroutines.withTimeoutOrNull
import java.util.concurrent.atomic.AtomicInteger

/**
 * Board link/network state (from the `link` frame). All fields are non-null Strings
 * (empty when unknown) so the UI can use `.ifBlank { ... }` directly.
 *  - mode : "AP"|"STA"|"CONNECTING"|"DISCONNECTED".
 *  - ssid : associated/AP SSID.
 *  - ip   : current IP (the manual-IP fallback value, §4.2).
 *  - rssi : signal strength dBm (0 in AP mode).
 *  - mdns : mDNS instance name (default "siggen").
 *  - appw : SoftAP passphrase (AP mode only; empty in STA).
 */
data class LinkInfo(
    val mode: String = "DISCONNECTED",
    val ssid: String = "",
    val ip: String = "",
    val rssi: Int = 0,
    val mdns: String = "",
    val appw: String = "",
)

/**
 * Immutable snapshot of everything the UI renders about the board.
 *
 * Folded from inbound NDJSON `tel` / `link` / `ack` / catalog frames (§4.0).
 * Field semantics:
 *  - rpm      : effective live RPM (sweepCurrentRpm) from `tel`.
 *  - baseRpm  : slider/base RPM (g_rpm) from `tel`.
 *  - run      : generator running (`tel.run`).
 *  - pattern  : active pattern name_key (`tel.pat`); empty when unknown.
 *  - degrees  : 360 or 720 crank degrees of the active table (`tel.deg`).
 *  - mask     : channel mask bit0=crank,bit1=cam1,bit2=cam2 (`tel.mask`).
 *  - inv      : output polarity inverted (`tel.inv`).
 *  - edge     : wrapping uint16 edge counter (`tel.edge`) — D15 RPM-simulated cursor.
 *  - cycleUs  : full-table cycle period in microseconds (`tel.cycUs`).
 *  - drop     : gUiMsgDropCount — queue-full drops (`tel.drop`).
 *  - link     : network/link state (`link` frame).
 *  - connected: TCP socket up (from [TcpClient.State]).
 *  - catalog  : cached raw catalog rows (folded from catBegin/pat/catEnd). The
 *               UI-facing [DeviceRepository.catalog] StateFlow exposes the mapped
 *               [PatternEntry] form.
 */
data class DeviceState(
    val rpm: Int = 0,
    val baseRpm: Int = 0,
    val run: Boolean = false,
    val pattern: String = "",
    val degrees: Int = 360,
    val mask: Int = 0,
    val inv: Boolean = false,
    val edge: Int = 0,
    val cycleUs: Long = 0,
    val drop: Int = 0,
    val link: LinkInfo = LinkInfo(),
    val connected: Boolean = false,
    val catalog: List<NdjsonProtocol.PatRow> = emptyList(),
)

/**
 * Owns the [TcpClient], folds inbound frames into a single [StateFlow]<[DeviceState]>,
 * and exposes one function per control action (plan §4.5). This is the public surface
 * the ViewModels (Agent N2) call — UI code never touches [TcpClient] or
 * [NdjsonProtocol] directly.
 *
 * Constructed with an Android [Context]; it owns a private [CoroutineScope] for the
 * inbound-fold and connection-state collectors (the app keeps a single repository for
 * its whole lifetime, so the scope is never explicitly cancelled).
 *
 * Control frames carry an auto-incrementing `id` echoed by the board's `ack`, so the
 * UI can correlate success/failure (e.g. DSL compile errors) via [acks].
 */
class DeviceRepository(
    @Suppress("UNUSED_PARAMETER") context: Context,
    private val scope: CoroutineScope = CoroutineScope(SupervisorJob() + Dispatchers.Default),
    private val client: TcpClient = TcpClient(scope),
) {
    private val _state = MutableStateFlow(DeviceState())
    val state: StateFlow<DeviceState> = _state.asStateFlow()

    private val _catalog = MutableStateFlow<List<PatternEntry>>(emptyList())
    /** UI-facing mapped catalog (builtins + user), folded from catBegin/pat/catEnd. */
    val catalog: StateFlow<List<PatternEntry>> = _catalog.asStateFlow()

    /** Ack stream for correlating command outcomes (compile errors, queue-full). */
    private val _acks = MutableSharedFlow<NdjsonProtocol.Ack>(replay = 0, extraBufferCapacity = 64)
    val acks: SharedFlow<NdjsonProtocol.Ack> = _acks.asSharedFlow()

    /**
     * scanRes stream for the provisioning screen network picker. replay=0 so
     * [scan] only ever observes a fresh response to its own request (never a stale
     * cached one).
     */
    private val _scanResults = MutableSharedFlow<NdjsonProtocol.ScanRes>(replay = 0, extraBufferCapacity = 4)
    val scanResults: SharedFlow<NdjsonProtocol.ScanRes> = _scanResults.asSharedFlow()

    private val seq = AtomicInteger(1)
    private fun nextId(): Int = seq.getAndIncrement()

    /** Accumulates catalog rows between catBegin and catEnd before publishing. */
    private val catalogBuf = mutableListOf<NdjsonProtocol.PatRow>()
    private var catalogOpen = false

    init {
        // Fold inbound frames into DeviceState / catalog / ack / scan streams.
        client.inbound
            .onEach { frame -> reduce(frame) }
            .launchIn(scope)

        // Mirror TCP connection state into DeviceState.connected.
        client.state
            .onEach { s ->
                _state.value = _state.value.copy(connected = (s == TcpClient.State.CONNECTED))
            }
            .launchIn(scope)
    }

    // ---------------------------------------------------------------------------
    // Connection lifecycle
    // ---------------------------------------------------------------------------

    /**
     * Open the TCP socket to [host]:[port]. Returns true once the connection reaches
     * [TcpClient.State.CONNECTED] within a short window, false on timeout. The read
     * loop keeps retrying in the background regardless.
     */
    suspend fun connect(host: String, port: Int = 3333): Boolean {
        client.connect(host, port)
        val connected = withTimeoutOrNull(CONNECT_AWAIT_MS) {
            client.state.first { it == TcpClient.State.CONNECTED }
            true
        }
        return connected == true
    }

    suspend fun disconnect() {
        client.disconnect()
    }

    // ---------------------------------------------------------------------------
    // Inbound reducer
    // ---------------------------------------------------------------------------

    private suspend fun reduce(frame: NdjsonProtocol.Inbound) {
        when (frame) {
            is NdjsonProtocol.Tel -> _state.value = _state.value.copy(
                rpm = frame.rpm,
                baseRpm = frame.baseRpm,
                run = frame.run,
                pattern = frame.pat ?: "",
                degrees = frame.deg,
                mask = frame.mask,
                inv = frame.inv,
                edge = frame.edge,
                cycleUs = frame.cycUs,
                drop = frame.drop,
            )

            is NdjsonProtocol.Link -> _state.value = _state.value.copy(
                link = LinkInfo(
                    mode = frame.mode,
                    ssid = frame.ssid ?: "",
                    ip = frame.ip ?: "",
                    rssi = frame.rssi,
                    mdns = frame.mdns ?: "",
                    appw = frame.appw ?: "",
                ),
            )

            is NdjsonProtocol.Ack -> _acks.emit(frame)

            is NdjsonProtocol.CatBegin -> {
                catalogBuf.clear()
                catalogOpen = true
            }

            is NdjsonProtocol.PatRow -> {
                if (catalogOpen) catalogBuf.add(frame)
            }

            is NdjsonProtocol.CatEnd -> {
                if (catalogOpen) {
                    val rows = catalogBuf.toList()
                    _state.value = _state.value.copy(catalog = rows)
                    _catalog.value = rows.map { PatternEntry.from(it) }
                    catalogOpen = false
                }
            }

            is NdjsonProtocol.ScanRes -> _scanResults.emit(frame)
        }
    }

    // ---------------------------------------------------------------------------
    // Control actions (phone -> board). Each maps to a §4.0 CONTROL frame.
    // ---------------------------------------------------------------------------

    /** Coalesced RPM (D10) — only the latest within ~60ms reaches the board. */
    fun setRpm(rpm: Int) {
        client.sendRpmCoalesced(NdjsonProtocol.encode(NdjsonProtocol.RpmFrame(v = rpm, id = nextId())))
    }

    fun start() {
        client.send(NdjsonProtocol.encode(NdjsonProtocol.StartFrame(id = nextId())))
    }

    fun stop() {
        client.send(NdjsonProtocol.encode(NdjsonProtocol.StopFrame(id = nextId())))
    }

    fun setInvert(inverted: Boolean) {
        client.send(NdjsonProtocol.encode(NdjsonProtocol.InvertFrame(v = if (inverted) 1 else 0, id = nextId())))
    }

    fun selectBuiltin(index: Int) {
        client.send(NdjsonProtocol.encode(NdjsonProtocol.SelBuiltinFrame(i = index, id = nextId())))
    }

    fun selectNamed(key: String) {
        client.send(NdjsonProtocol.encode(NdjsonProtocol.SelNamedFrame(key = key, id = nextId())))
    }

    fun loadDsl(src: String) {
        client.send(NdjsonProtocol.encode(NdjsonProtocol.LoadDslFrame(src = src, id = nextId())))
    }

    /** Coalesced sweep (D10). mode: 0=OFF,1=LINEAR,2=LOG,3=WAYPOINT. */
    fun setSweep(low: Int, high: Int, mode: Int, intervalUs: Long) {
        client.sendSweepCoalesced(
            NdjsonProtocol.encode(
                NdjsonProtocol.SweepFrame(lo = low, hi = high, mode = mode, iv = intervalUs.toInt(), id = nextId())
            )
        )
    }

    fun setCompression(enabled: Boolean, cyl: Int, thresh: Int, peak: Int, dynamic: Boolean) {
        client.send(
            NdjsonProtocol.encode(
                NdjsonProtocol.CompFrame(on = enabled, cyl = cyl, thr = thresh, peak = peak, dyn = dynamic, id = nextId())
            )
        )
    }

    /** revs absent/0 -> firmware defaults to 2 (main.cpp:518). */
    fun captureStart(revs: Int? = null) {
        client.send(NdjsonProtocol.encode(NdjsonProtocol.CapStartFrame(revs = revs, id = nextId())))
    }

    fun captureStop() {
        client.send(NdjsonProtocol.encode(NdjsonProtocol.CapStopFrame(id = nextId())))
    }

    fun saveUser(key: String, src: String) {
        client.send(NdjsonProtocol.encode(NdjsonProtocol.SaveFrame(key = key, src = src, id = nextId())))
    }

    // ---------------------------------------------------------------------------
    // Link / query actions (handled inside wifi_link, not queued on the board).
    // ---------------------------------------------------------------------------

    /** hello -> board replies link + tel + (optionally) full catalog. */
    fun hello() {
        client.send(NdjsonProtocol.encode(NdjsonProtocol.HelloFrame(id = nextId())))
    }

    /** list -> board replies catBegin / pat... / catEnd (lands in [catalog]). */
    fun requestCatalog() {
        client.send(NdjsonProtocol.encode(NdjsonProtocol.ListFrame(id = nextId())))
    }

    /**
     * scan -> board replies scanRes. Suspends until the next scanRes arrives (or a
     * timeout) and returns the mapped network list for the provisioning picker.
     */
    suspend fun scan(): List<ScanResultEntry> {
        // Subscribe BEFORE sending so we never miss (or stale-read) the response.
        val deferred = scope.async {
            withTimeoutOrNull(SCAN_AWAIT_MS) { _scanResults.first() }
        }
        client.send(NdjsonProtocol.encode(NdjsonProtocol.ScanFrame(id = nextId())))
        val res = deferred.await()
        return res?.nets?.map { ScanResultEntry.from(it) } ?: emptyList()
    }

    /**
     * provision -> board persists creds, ACKs immediately, then WiFi.begin(STA).
     * The AP socket is EXPECTED to drop after the ack (§2.4); callers should hand
     * off to home-network rediscovery. Returned id lets the caller match the ack.
     */
    fun provision(ssid: String, pass: String): Int {
        val id = nextId()
        client.send(NdjsonProtocol.encode(NdjsonProtocol.ProvisionFrame(ssid = ssid, pass = pass, id = id)))
        return id
    }

    /** forgetWifi -> board clears creds and reboots to SoftAP. */
    fun forgetWifi() {
        client.send(NdjsonProtocol.encode(NdjsonProtocol.ForgetWifiFrame(id = nextId())))
    }

    companion object {
        private const val CONNECT_AWAIT_MS = 7000L
        private const val SCAN_AWAIT_MS = 8000L
    }
}
