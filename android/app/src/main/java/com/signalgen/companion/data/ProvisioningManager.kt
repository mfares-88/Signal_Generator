package com.signalgen.companion.data

import android.content.Context
import android.net.ConnectivityManager
import android.net.Network
import android.net.NetworkCapabilities
import android.net.NetworkRequest
import android.net.wifi.WifiNetworkSpecifier
import android.os.Build
import androidx.annotation.RequiresApi
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.suspendCancellableCoroutine
import kotlinx.coroutines.withTimeoutOrNull
import kotlin.coroutines.resume

/**
 * Drives the first-run SoftAP provisioning handshake (plan §2.4 / §4.5).
 *
 * Flow used by [com.signalgen.companion.ui.ConnectionViewModel]:
 *  1. [joinSoftAp] builds a [WifiNetworkSpecifier] for the board's SoftAP
 *     `SignalGen-XXXX` with the D14-derived WPA2 passphrase, requests it via
 *     [ConnectivityManager.requestNetwork], binds the process to that network
 *     ([ConnectivityManager.bindProcessToNetwork]) so the socket to 192.168.4.1
 *     actually traverses the AP, then connects the shared [DeviceRepository]'s
 *     TCP client to 192.168.4.1:3333. Returns true once connected.
 *  2. The UI may [DeviceRepository.scan] for home networks, then calls [provision].
 *  3. The board ACKs immediately then joins STA — the AP socket DROPS (expected,
 *     §2.2). The UI then calls [unbind] to release the AP network and hands off to
 *     home-network rediscovery ([DiscoveryManager]).
 *
 * minSdk 29 guarantees [WifiNetworkSpecifier] (plan D5).
 *
 * The passphrase derivation MUST match firmware D14 exactly:
 *   "siggen-" + last 4 MAC bytes as 8 lowercase hex digits  (15 chars, WPA2-valid).
 */
class ProvisioningManager(
    context: Context,
    private val repository: DeviceRepository,
) {
    private val appContext = context.applicationContext
    private val cm = appContext.getSystemService(Context.CONNECTIVITY_SERVICE) as ConnectivityManager

    enum class ApState { IDLE, REQUESTING, BOUND, UNAVAILABLE, LOST }

    private val _apState = MutableStateFlow(ApState.IDLE)
    val apState: StateFlow<ApState> = _apState.asStateFlow()

    private var callback: ConnectivityManager.NetworkCallback? = null
    private var boundNetwork: Network? = null

    /**
     * Request + bind the board SoftAP network, then connect the shared repository's
     * TCP client to [SOFTAP_IP]:[SOFTAP_PORT]. Suspends until the AP network is
     * available (or a timeout) and the socket reaches CONNECTED.
     *
     * @param apSsid e.g. "SignalGen-1A2B".
     * @param apPassphrase the D14-derived WPA2 passphrase shown on the LCD
     *        (see [derivePassphrase] when the full MAC is known).
     * @return true when bound + TCP-connected; false on timeout/unavailable.
     */
    @RequiresApi(Build.VERSION_CODES.Q)
    suspend fun joinSoftAp(apSsid: String, apPassphrase: String): Boolean {
        unbind()
        _apState.value = ApState.REQUESTING

        val bound = withTimeoutOrNull(AP_AWAIT_MS) {
            requestApNetwork(apSsid, apPassphrase)
        } ?: false
        if (!bound) {
            _apState.value = ApState.UNAVAILABLE
            return false
        }
        return repository.connect(SOFTAP_IP, SOFTAP_PORT)
    }

    /**
     * Push home credentials to the board over the already-bound AP socket. The board
     * ACKs immediately then begins STA join (§2.4); the AP socket is EXPECTED to drop
     * afterwards. Returns the provision frame id (echoed in the board's ack).
     */
    fun provision(ssid: String, pass: String): Int = repository.provision(ssid, pass)

    /**
     * Unbind the process from the AP network and release the request. Call this after
     * the provision ack (the AP socket is expected to die) to hand off to home-network
     * rediscovery.
     */
    fun unbind() {
        try {
            cm.bindProcessToNetwork(null)
        } catch (_: Exception) {
        }
        callback?.let {
            try {
                cm.unregisterNetworkCallback(it)
            } catch (_: Exception) {
            }
        }
        callback = null
        boundNetwork = null
        if (_apState.value != ApState.IDLE) _apState.value = ApState.IDLE
    }

    @RequiresApi(Build.VERSION_CODES.Q)
    private suspend fun requestApNetwork(ssid: String, passphrase: String): Boolean =
        suspendCancellableCoroutine { cont ->
            val specifier = WifiNetworkSpecifier.Builder()
                .setSsid(ssid)
                .setWpa2Passphrase(passphrase)
                .build()

            val request = NetworkRequest.Builder()
                .addTransportType(NetworkCapabilities.TRANSPORT_WIFI)
                // The SoftAP has no internet — do NOT require it, or the request never resolves.
                .removeCapability(NetworkCapabilities.NET_CAPABILITY_INTERNET)
                .setNetworkSpecifier(specifier)
                .build()

            val cb = object : ConnectivityManager.NetworkCallback() {
                @Volatile
                private var resumed = false

                override fun onAvailable(network: Network) {
                    boundNetwork = network
                    cm.bindProcessToNetwork(network)
                    _apState.value = ApState.BOUND
                    if (!resumed) {
                        resumed = true
                        if (cont.isActive) cont.resume(true)
                    }
                }

                override fun onUnavailable() {
                    _apState.value = ApState.UNAVAILABLE
                    if (!resumed) {
                        resumed = true
                        if (cont.isActive) cont.resume(false)
                    }
                }

                override fun onLost(network: Network) {
                    // Expected after the board switches to STA (§2.4 step 5).
                    if (boundNetwork == network) {
                        cm.bindProcessToNetwork(null)
                        boundNetwork = null
                    }
                    _apState.value = ApState.LOST
                }
            }
            callback = cb
            cm.requestNetwork(request, cb)

            cont.invokeOnCancellation {
                try { cm.unregisterNetworkCallback(cb) } catch (_: Exception) {}
            }
        }

    companion object {
        const val SOFTAP_IP = "192.168.4.1"
        const val SOFTAP_PORT = 3333
        private const val AP_AWAIT_MS = 20_000L

        /**
         * Firmware D14 SoftAP passphrase: "siggen-" + last 4 MAC bytes as 8 lowercase
         * hex digits. [mac] is the 6-byte board base MAC (big-endian as printed).
         * Produces a 15-char WPA2-valid passphrase, e.g. ..AA:BB:CC:DD -> "siggen-aabbccdd".
         */
        fun derivePassphrase(mac: ByteArray): String {
            require(mac.size == 6) { "MAC must be 6 bytes" }
            val last4 = mac.copyOfRange(2, 6)
            val hex = last4.joinToString("") { "%02x".format(it.toInt() and 0xFF) }
            return "siggen-$hex"
        }

        /**
         * Firmware SoftAP SSID suffix: last 2 MAC bytes as 4 uppercase hex (§2.3),
         * e.g. "SignalGen-CCDD".
         */
        fun deriveSsid(mac: ByteArray): String {
            require(mac.size == 6) { "MAC must be 6 bytes" }
            val last2 = mac.copyOfRange(4, 6)
            val hex = last2.joinToString("") { "%02X".format(it.toInt() and 0xFF) }
            return "SignalGen-$hex"
        }
    }
}
