package com.signalgen.companion.data

import android.content.Context
import androidx.datastore.core.DataStore
import androidx.datastore.preferences.core.Preferences
import androidx.datastore.preferences.core.edit
import androidx.datastore.preferences.core.stringPreferencesKey
import androidx.datastore.preferences.preferencesDataStore
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.map

/**
 * Persists user connection preferences via Jetpack DataStore (plan §4.5):
 * last successful board IP, last home SSID, and the mDNS instance name.
 *
 * These seed the reconnect / manual-IP-fallback flow so the app can re-find the
 * board after the post-provisioning SoftAP drop (§2.4) without re-discovery.
 */
private val Context.dataStore: DataStore<Preferences> by preferencesDataStore(name = "signalgen_settings")

class SettingsStore(private val context: Context) {

    private object Keys {
        val LAST_IP = stringPreferencesKey("last_ip")
        val LAST_SSID = stringPreferencesKey("last_ssid")
        val MDNS_NAME = stringPreferencesKey("mdns_name")
    }

    val lastIp: Flow<String?> = context.dataStore.data.map { it[Keys.LAST_IP] }
    val lastSsid: Flow<String?> = context.dataStore.data.map { it[Keys.LAST_SSID] }
    val mdnsName: Flow<String> = context.dataStore.data.map { it[Keys.MDNS_NAME] ?: DEFAULT_MDNS }

    suspend fun setLastIp(ip: String) {
        context.dataStore.edit { it[Keys.LAST_IP] = ip }
    }

    suspend fun setLastSsid(ssid: String) {
        context.dataStore.edit { it[Keys.LAST_SSID] = ssid }
    }

    suspend fun setMdnsName(name: String) {
        context.dataStore.edit { it[Keys.MDNS_NAME] = name }
    }

    companion object {
        /** Matches firmware MDNS.begin("siggen") (plan §4.1). */
        const val DEFAULT_MDNS = "siggen"
    }
}
