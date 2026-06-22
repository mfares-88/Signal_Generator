package com.signalgen.companion.ui

import android.app.Application
import androidx.lifecycle.AndroidViewModel
import androidx.lifecycle.viewModelScope
import com.signalgen.companion.data.DeviceRepository
import com.signalgen.companion.data.DeviceState
import com.signalgen.companion.data.DiscoveryManager
import com.signalgen.companion.data.PatternEntry
import com.signalgen.companion.data.ProvisioningManager
import com.signalgen.companion.data.ScanResultEntry
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.SharingStarted
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.stateIn
import kotlinx.coroutines.launch

/**
 * Lightweight service locator so the ViewModel can be constructed by the
 * default [androidx.lifecycle.ViewModelProvider] factory (Application ctor)
 * while still sharing ONE [DeviceRepository] / [TcpClient] across the app.
 *
 * No Hilt by design (plan §4.6: "keep DI lightweight, no Hilt").
 */
object ServiceLocator {
    @Volatile
    private var repo: DeviceRepository? = null

    @Volatile
    private var prov: ProvisioningManager? = null

    @Volatile
    private var disc: DiscoveryManager? = null

    fun repository(app: Application): DeviceRepository =
        repo ?: synchronized(this) {
            repo ?: DeviceRepository(app).also { repo = it }
        }

    fun provisioning(app: Application): ProvisioningManager =
        prov ?: synchronized(this) {
            prov ?: ProvisioningManager(app, repository(app)).also { prov = it }
        }

    fun discovery(app: Application): DiscoveryManager =
        disc ?: synchronized(this) {
            disc ?: DiscoveryManager(app).also { disc = it }
        }
}

/** UI-side connection phase (separate from the board's link mode). */
enum class ConnPhase { DISCONNECTED, DISCOVERING, CONNECTING, CONNECTED, PROVISIONING, ERROR }

/** Aggregated connection / provisioning UI state surfaced to the screens. */
data class ConnectionUiState(
    val phase: ConnPhase = ConnPhase.DISCONNECTED,
    val message: String? = null,
    val manualIp: String = "",
    val discovered: List<DiscoveredDevice> = emptyList(),
    val scanResults: List<ScanResultEntry> = emptyList(),
    val scanning: Boolean = false,
)

/** A device found via NSD discovery (host + port + service name). */
data class DiscoveredDevice(
    val name: String,
    val host: String,
    val port: Int,
)

/**
 * Bridges [DeviceRepository]'s reactive state to Compose and owns the
 * connection / provisioning / discovery UI state. Every screen action funnels
 * through here and delegates to the repository — screens never touch the
 * networking layer directly.
 */
class ConnectionViewModel(app: Application) : AndroidViewModel(app) {

    private val repo: DeviceRepository = ServiceLocator.repository(app)
    private val provisioning: ProvisioningManager = ServiceLocator.provisioning(app)
    private val discovery: DiscoveryManager = ServiceLocator.discovery(app)

    /** Live device telemetry/link state, folded by the repository from tel/link/ack frames. */
    val deviceState: StateFlow<DeviceState> = repo.state.stateIn(
        scope = viewModelScope,
        started = SharingStarted.WhileSubscribed(5_000),
        initialValue = repo.state.value,
    )

    /** Cached pattern catalog (builtins + user) from the last `list`/`hello`. */
    val catalog: StateFlow<List<PatternEntry>> = repo.catalog.stateIn(
        scope = viewModelScope,
        started = SharingStarted.WhileSubscribed(5_000),
        initialValue = repo.catalog.value,
    )

    private val _conn = MutableStateFlow(ConnectionUiState())
    val connUi: StateFlow<ConnectionUiState> = _conn.asStateFlow()

    // ------------------------------------------------------------------
    // Connection lifecycle
    // ------------------------------------------------------------------

    fun setManualIp(ip: String) {
        _conn.value = _conn.value.copy(manualIp = ip)
    }

    /** Connect to an explicit host:port (manual-IP fallback, plan D13). */
    fun connect(host: String, port: Int = 3333) {
        _conn.value = _conn.value.copy(phase = ConnPhase.CONNECTING, message = "Connecting to $host…")
        viewModelScope.launch {
            val ok = runCatching { repo.connect(host, port) }.getOrElse { false }
            _conn.value = if (ok) {
                repo.hello()
                _conn.value.copy(phase = ConnPhase.CONNECTED, message = null)
            } else {
                _conn.value.copy(phase = ConnPhase.ERROR, message = "Could not reach $host:$port")
            }
        }
    }

    /** Connect using the manual-IP text field. */
    fun connectManual() {
        val ip = _conn.value.manualIp.trim()
        if (ip.isNotEmpty()) connect(ip)
    }

    fun disconnect() {
        viewModelScope.launch { repo.disconnect() }
        _conn.value = _conn.value.copy(phase = ConnPhase.DISCONNECTED, message = null)
    }

    // ------------------------------------------------------------------
    // Discovery (NSD)
    // ------------------------------------------------------------------

    fun startDiscovery() {
        _conn.value = _conn.value.copy(phase = ConnPhase.DISCOVERING, message = "Searching for devices…")
        viewModelScope.launch {
            runCatching {
                discovery.discover().collect { d ->
                    val device = DiscoveredDevice(d.name, d.host, d.port)
                    val merged = (_conn.value.discovered.filterNot { it.host == device.host } + device)
                    _conn.value = _conn.value.copy(discovered = merged)
                }
            }
        }
    }

    fun stopDiscovery() {
        discovery.stop()
        if (_conn.value.phase == ConnPhase.DISCOVERING) {
            _conn.value = _conn.value.copy(phase = ConnPhase.DISCONNECTED)
        }
    }

    // ------------------------------------------------------------------
    // Provisioning (SoftAP → home WiFi handoff, plan §2.4)
    // ------------------------------------------------------------------

    /** Join the board SoftAP, bind the process network, connect TCP, optionally scan. */
    fun joinSoftAp(apSsid: String, apPassphrase: String) {
        _conn.value = _conn.value.copy(phase = ConnPhase.PROVISIONING, message = "Joining $apSsid…")
        viewModelScope.launch {
            val ok = runCatching { provisioning.joinSoftAp(apSsid, apPassphrase) }.getOrElse { false }
            _conn.value = if (ok) {
                _conn.value.copy(phase = ConnPhase.CONNECTED, message = "Connected to board AP")
            } else {
                _conn.value.copy(phase = ConnPhase.ERROR, message = "Could not join $apSsid")
            }
        }
    }

    /** Ask the board to scan home networks; results land in [ConnectionUiState.scanResults]. */
    fun scanNetworks() {
        _conn.value = _conn.value.copy(scanning = true)
        viewModelScope.launch {
            val nets = runCatching { repo.scan() }.getOrElse { emptyList() }
            _conn.value = _conn.value.copy(scanning = false, scanResults = nets)
        }
    }

    /**
     * Push home credentials. The board ACKs immediately then begins STA join;
     * the SoftAP socket is EXPECTED to drop (plan §2.4). We then unbind and move
     * to rediscovery.
     */
    fun provision(ssid: String, pass: String) {
        _conn.value = _conn.value.copy(phase = ConnPhase.PROVISIONING, message = "Provisioning $ssid…")
        viewModelScope.launch {
            runCatching { provisioning.provision(ssid, pass) }
            // Socket drop is normal — transition to rediscovery on home WiFi.
            provisioning.unbind()
            _conn.value = _conn.value.copy(
                phase = ConnPhase.DISCOVERING,
                message = "Board joining $ssid — rediscovering on home WiFi…",
            )
            startDiscovery()
        }
    }

    fun forgetWifi() {
        viewModelScope.launch { runCatching { repo.forgetWifi() } }
        _conn.value = _conn.value.copy(phase = ConnPhase.DISCONNECTED, message = "WiFi forgotten — board rebooting to AP")
    }

    // ------------------------------------------------------------------
    // Control actions — all delegate to DeviceRepository (plan §4.5).
    // ------------------------------------------------------------------

    fun setRpm(rpm: Int) = viewModelScope.launch { repo.setRpm(rpm) }
    fun start() = viewModelScope.launch { repo.start() }
    fun stop() = viewModelScope.launch { repo.stop() }
    fun setInvert(on: Boolean) = viewModelScope.launch { repo.setInvert(on) }
    fun selectBuiltin(index: Int) = viewModelScope.launch { repo.selectBuiltin(index) }
    fun selectNamed(key: String) = viewModelScope.launch { repo.selectNamed(key) }
    fun loadDsl(src: String) = viewModelScope.launch { repo.loadDsl(src) }
    fun saveUser(key: String, src: String) = viewModelScope.launch { repo.saveUser(key, src) }

    fun setSweep(low: Int, high: Int, mode: Int, intervalUs: Long) =
        viewModelScope.launch { repo.setSweep(low, high, mode, intervalUs) }

    fun setCompression(enabled: Boolean, cyl: Int, thresh: Int, peak: Int, dynamic: Boolean) =
        viewModelScope.launch { repo.setCompression(enabled, cyl, thresh, peak, dynamic) }

    fun captureStart(revs: Int) = viewModelScope.launch { repo.captureStart(revs) }
    fun captureStop() = viewModelScope.launch { repo.captureStop() }

    fun refreshCatalog() = viewModelScope.launch { repo.requestCatalog() }
}
