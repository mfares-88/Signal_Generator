package com.signalgen.companion.data

import android.content.Context
import android.net.nsd.NsdManager
import android.net.nsd.NsdServiceInfo
import android.net.wifi.WifiManager
import kotlinx.coroutines.channels.awaitClose
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.callbackFlow

/**
 * Discovers the board on the home network via NSD (`_siggen._tcp`, plan §4.1/D13)
 * and provides a manual-IP fallback (the IP shown on the LCD link label, §4.2).
 *
 * NSD is the primary path; on resolve the [DiscoveredDevice] carries the host/port to
 * feed [DeviceRepository.connect]. A Wi-Fi multicast lock is held during discovery so
 * mDNS/NSD packets are not filtered on networks that drop multicast by default.
 *
 * One board at a time (single phone, D13) — collectors typically take the first
 * resolved service.
 *
 * Lifecycle: [discover] returns a cold flow that runs discovery for the lifetime of
 * collection. [stop] tears down the currently-active discovery listener (the
 * ViewModel calls it directly rather than cancelling the collect job).
 */
class DiscoveryManager(context: Context) {

    private val appContext = context.applicationContext
    private val nsdManager = appContext.getSystemService(Context.NSD_SERVICE) as NsdManager
    private val wifiManager = appContext.getSystemService(Context.WIFI_SERVICE) as WifiManager

    /** A device found via NSD discovery (or manual entry). */
    data class DiscoveredDevice(
        val name: String,
        val host: String,
        val port: Int,
    )

    @Volatile
    private var activeListener: NsdManager.DiscoveryListener? = null

    @Volatile
    private var activeMulticastLock: WifiManager.MulticastLock? = null

    /**
     * Cold flow that runs NSD discovery for the lifetime of collection and emits each
     * resolved board. Stops discovery + releases the multicast lock on cancellation or
     * when [stop] is called.
     */
    fun discover(): Flow<DiscoveredDevice> = callbackFlow {
        val multicastLock = wifiManager.createMulticastLock(MULTICAST_TAG).apply {
            setReferenceCounted(true)
            try { acquire() } catch (_: Exception) {}
        }
        activeMulticastLock = multicastLock

        // resolveService is deprecated on API 34+; the legacy callback path still works
        // across minSdk 29..34 and avoids the new ServiceInfoCallback (API 34-only).
        fun resolve(serviceInfo: NsdServiceInfo) {
            val resolveListener = object : NsdManager.ResolveListener {
                override fun onResolveFailed(si: NsdServiceInfo, errorCode: Int) {
                    // Ignore; another advertisement/resolve may succeed.
                }

                override fun onServiceResolved(si: NsdServiceInfo) {
                    val host = hostAddressOf(si) ?: return
                    trySend(
                        DiscoveredDevice(
                            name = si.serviceName ?: SERVICE_TYPE,
                            host = host,
                            port = si.port,
                        )
                    )
                }
            }
            @Suppress("DEPRECATION")
            try {
                nsdManager.resolveService(serviceInfo, resolveListener)
            } catch (_: Exception) {
            }
        }

        val discoveryListener = object : NsdManager.DiscoveryListener {
            override fun onStartDiscoveryFailed(serviceType: String, errorCode: Int) {
                try { nsdManager.stopServiceDiscovery(this) } catch (_: Exception) {}
            }

            override fun onStopDiscoveryFailed(serviceType: String, errorCode: Int) {}

            override fun onDiscoveryStarted(serviceType: String) {}

            override fun onDiscoveryStopped(serviceType: String) {}

            override fun onServiceFound(serviceInfo: NsdServiceInfo) {
                if (serviceInfo.serviceType.contains(SERVICE_TYPE_BARE)) {
                    resolve(serviceInfo)
                }
            }

            override fun onServiceLost(serviceInfo: NsdServiceInfo) {}
        }

        try {
            nsdManager.discoverServices(SERVICE_TYPE, NsdManager.PROTOCOL_DNS_SD, discoveryListener)
            activeListener = discoveryListener
        } catch (e: Exception) {
            close(e)
        }

        awaitClose {
            teardown(discoveryListener, multicastLock)
        }
    }

    /**
     * Manual-IP fallback (§4.2): the user types the IP shown on the LCD link label.
     * Returns a [DiscoveredDevice] pointing at port 3333 with no NSD round-trip.
     */
    fun manualEntry(ip: String, port: Int = DEFAULT_PORT): DiscoveredDevice =
        DiscoveredDevice(name = "manual", host = ip.trim(), port = port)

    /** Stop the active NSD discovery and release the multicast lock, if any. */
    fun stop() {
        val listener = activeListener
        val lock = activeMulticastLock
        teardown(listener, lock)
    }

    private fun teardown(
        listener: NsdManager.DiscoveryListener?,
        lock: WifiManager.MulticastLock?,
    ) {
        if (listener != null) {
            try { nsdManager.stopServiceDiscovery(listener) } catch (_: Exception) {}
            if (activeListener === listener) activeListener = null
        }
        if (lock != null) {
            try { if (lock.isHeld) lock.release() } catch (_: Exception) {}
            if (activeMulticastLock === lock) activeMulticastLock = null
        }
    }

    private fun hostAddressOf(si: NsdServiceInfo): String? {
        @Suppress("DEPRECATION")
        return si.host?.hostAddress
    }

    companion object {
        const val SERVICE_TYPE = "_siggen._tcp."
        const val SERVICE_TYPE_BARE = "_siggen._tcp"
        const val DEFAULT_PORT = 3333
        private const val MULTICAST_TAG = "signalgen-nsd"
    }
}
