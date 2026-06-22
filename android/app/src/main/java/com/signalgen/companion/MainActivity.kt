package com.signalgen.companion

import android.Manifest
import android.content.pm.PackageManager
import android.os.Build
import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.result.contract.ActivityResultContracts
import androidx.core.content.ContextCompat
import com.signalgen.companion.ui.nav.AppNav
import com.signalgen.companion.ui.theme.SignalGenTheme

/**
 * Single-activity entry point for the Signal Generator companion app.
 *
 * Requests runtime permissions needed for NSD discovery and Wi-Fi provisioning,
 * then hands control to the Compose navigation graph via [AppNav].
 *
 * Permissions requested at runtime:
 * - NEARBY_WIFI_DEVICES (API 33+) — Wi-Fi scanning without location
 * - ACCESS_FINE_LOCATION (API 29-32) — required for NSD/mDNS resolution on older APIs
 *
 * INTERNET, ACCESS_NETWORK_STATE, CHANGE_NETWORK_STATE, ACCESS_WIFI_STATE,
 * CHANGE_WIFI_STATE, and CHANGE_WIFI_MULTICAST_STATE are install-time permissions
 * and do not require runtime requests.
 */
class MainActivity : ComponentActivity() {

    private val permissionLauncher = registerForActivityResult(
        ActivityResultContracts.RequestMultiplePermissions()
    ) { _ ->
        // Permission results are consumed by DiscoveryManager / ProvisioningManager
        // via ConnectivityManager callbacks. We do not gate the UI on grant status —
        // the user can still enter a manual IP if Wi-Fi permissions are denied.
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        requestNecessaryPermissions()

        setContent {
            SignalGenTheme {
                AppNav()
            }
        }
    }

    private fun requestNecessaryPermissions() {
        val needed = mutableListOf<String>()

        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
            // API 33+: NEARBY_WIFI_DEVICES for Wi-Fi scanning / NSD
            if (!isGranted(Manifest.permission.NEARBY_WIFI_DEVICES)) {
                needed.add(Manifest.permission.NEARBY_WIFI_DEVICES)
            }
        } else {
            // API 29-32: ACCESS_FINE_LOCATION required for NSD / Wi-Fi scan results
            if (!isGranted(Manifest.permission.ACCESS_FINE_LOCATION)) {
                needed.add(Manifest.permission.ACCESS_FINE_LOCATION)
            }
        }

        if (needed.isNotEmpty()) {
            permissionLauncher.launch(needed.toTypedArray())
        }
    }

    private fun isGranted(permission: String): Boolean =
        ContextCompat.checkSelfPermission(this, permission) == PackageManager.PERMISSION_GRANTED
}
