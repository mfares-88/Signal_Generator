package com.signalgen.companion.ui

import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.Card
import androidx.compose.material3.CardDefaults
import androidx.compose.material3.CircularProgressIndicator
import androidx.compose.material3.Divider
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.OutlinedTextField
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.input.PasswordVisualTransformation
import androidx.compose.ui.unit.dp
import com.signalgen.companion.data.DeviceState
import com.signalgen.companion.data.ScanResultEntry

@Composable
fun ConnectionScreen(
    state: DeviceState,
    conn: ConnectionUiState,
    onManualIp: (String) -> Unit,
    onConnectManual: () -> Unit,
    onConnectDiscovered: (host: String, port: Int) -> Unit,
    onStartDiscovery: () -> Unit,
    onStopDiscovery: () -> Unit,
    onDisconnect: () -> Unit,
    onJoinSoftAp: (ssid: String, passphrase: String) -> Unit,
    onScan: () -> Unit,
    onProvision: (ssid: String, pass: String) -> Unit,
    onForget: () -> Unit,
    modifier: Modifier = Modifier,
) {
    Column(
        modifier = modifier
            .fillMaxWidth()
            .verticalScroll(rememberScrollState())
            .padding(16.dp),
        verticalArrangement = Arrangement.spacedBy(16.dp),
    ) {
        StatusCard(state = state, conn = conn)
        DiscoveryCard(
            conn = conn,
            onStartDiscovery = onStartDiscovery,
            onStopDiscovery = onStopDiscovery,
            onConnectDiscovered = onConnectDiscovered,
        )
        ManualIpCard(
            conn = conn,
            onManualIp = onManualIp,
            onConnectManual = onConnectManual,
        )
        ProvisioningCard(
            conn = conn,
            onJoinSoftAp = onJoinSoftAp,
            onScan = onScan,
            onProvision = onProvision,
        )
        Button(
            onClick = onForget,
            modifier = Modifier.fillMaxWidth(),
            colors = ButtonDefaults.buttonColors(
                containerColor = MaterialTheme.colorScheme.error,
                contentColor = MaterialTheme.colorScheme.onError,
            ),
        ) { Text("FORGET WIFI (reboot to AP)", fontWeight = FontWeight.Bold) }

        if (conn.phase == ConnPhase.CONNECTED) {
            OutlinedButton(onClick = onDisconnect, modifier = Modifier.fillMaxWidth()) {
                Text("DISCONNECT")
            }
        }
    }
}

@Composable
private fun StatusCard(state: DeviceState, conn: ConnectionUiState) {
    Card(
        colors = CardDefaults.cardColors(containerColor = MaterialTheme.colorScheme.surfaceVariant),
        shape = RoundedCornerShape(12.dp),
        modifier = Modifier.fillMaxWidth(),
    ) {
        Column(Modifier.padding(16.dp), verticalArrangement = Arrangement.spacedBy(6.dp)) {
            Text("Status", color = MaterialTheme.colorScheme.primary, style = MaterialTheme.typography.titleLarge)
            Row(verticalAlignment = Alignment.CenterVertically) {
                if (conn.phase == ConnPhase.CONNECTING ||
                    conn.phase == ConnPhase.DISCOVERING ||
                    conn.phase == ConnPhase.PROVISIONING
                ) {
                    CircularProgressIndicator(
                        modifier = Modifier.padding(end = 8.dp),
                        strokeWidth = 2.dp,
                        color = MaterialTheme.colorScheme.primary,
                    )
                }
                Text(conn.phase.name, color = MaterialTheme.colorScheme.onSurface)
            }
            conn.message?.let {
                Text(it, color = MaterialTheme.colorScheme.onSurfaceVariant, style = MaterialTheme.typography.labelLarge)
            }
            Divider(color = MaterialTheme.colorScheme.outlineVariant)
            MonoRow("mode", state.link.mode)
            MonoRow("ssid", state.link.ssid.ifBlank { "—" })
            MonoRow("ip", state.link.ip.ifBlank { "—" })
            MonoRow("rssi", if (state.link.rssi != 0) "${state.link.rssi} dBm" else "—")
            MonoRow("mdns", state.link.mdns.ifBlank { "siggen" })
            if (state.link.appw.isNotBlank()) MonoRow("AP pw", state.link.appw)
            MonoRow("drops", "${state.drop}")
        }
    }
}

@Composable
private fun MonoRow(label: String, value: String) {
    Row(Modifier.fillMaxWidth()) {
        Text(
            label,
            color = MaterialTheme.colorScheme.onSurfaceVariant,
            fontFamily = FontFamily.Monospace,
            modifier = Modifier.weight(0.4f),
        )
        Text(
            value,
            color = MaterialTheme.colorScheme.onSurface,
            fontFamily = FontFamily.Monospace,
            modifier = Modifier.weight(0.6f),
        )
    }
}

@Composable
private fun DiscoveryCard(
    conn: ConnectionUiState,
    onStartDiscovery: () -> Unit,
    onStopDiscovery: () -> Unit,
    onConnectDiscovered: (String, Int) -> Unit,
) {
    Card(
        colors = CardDefaults.cardColors(containerColor = MaterialTheme.colorScheme.surfaceVariant),
        shape = RoundedCornerShape(12.dp),
        modifier = Modifier.fillMaxWidth(),
    ) {
        Column(Modifier.padding(16.dp), verticalArrangement = Arrangement.spacedBy(8.dp)) {
            Text("Discover (mDNS)", color = MaterialTheme.colorScheme.primary, style = MaterialTheme.typography.titleLarge)
            Row(horizontalArrangement = Arrangement.spacedBy(12.dp)) {
                Button(
                    onClick = onStartDiscovery,
                    modifier = Modifier.weight(1f),
                    colors = ButtonDefaults.buttonColors(
                        containerColor = MaterialTheme.colorScheme.primary,
                        contentColor = MaterialTheme.colorScheme.onPrimary,
                    ),
                ) { Text("SCAN _siggen._tcp") }
                OutlinedButton(onClick = onStopDiscovery, modifier = Modifier.weight(1f)) {
                    Text("STOP")
                }
            }
            if (conn.discovered.isEmpty()) {
                Text("No devices found yet.", color = MaterialTheme.colorScheme.onSurfaceVariant)
            } else {
                conn.discovered.forEach { d ->
                    Row(
                        Modifier.fillMaxWidth(),
                        verticalAlignment = Alignment.CenterVertically,
                    ) {
                        Column(Modifier.weight(1f)) {
                            Text(d.name, color = MaterialTheme.colorScheme.onSurface)
                            Text(
                                "${d.host}:${d.port}",
                                color = MaterialTheme.colorScheme.onSurfaceVariant,
                                fontFamily = FontFamily.Monospace,
                                style = MaterialTheme.typography.labelLarge,
                            )
                        }
                        Button(
                            onClick = { onConnectDiscovered(d.host, d.port) },
                            colors = ButtonDefaults.buttonColors(
                                containerColor = MaterialTheme.colorScheme.primary,
                                contentColor = MaterialTheme.colorScheme.onPrimary,
                            ),
                        ) { Text("CONNECT") }
                    }
                }
            }
        }
    }
}

@Composable
private fun ManualIpCard(
    conn: ConnectionUiState,
    onManualIp: (String) -> Unit,
    onConnectManual: () -> Unit,
) {
    Card(
        colors = CardDefaults.cardColors(containerColor = MaterialTheme.colorScheme.surfaceVariant),
        shape = RoundedCornerShape(12.dp),
        modifier = Modifier.fillMaxWidth(),
    ) {
        Column(Modifier.padding(16.dp), verticalArrangement = Arrangement.spacedBy(8.dp)) {
            Text("Manual IP", color = MaterialTheme.colorScheme.primary, style = MaterialTheme.typography.titleLarge)
            Text(
                "Read the IP off the device's on-screen link label.",
                color = MaterialTheme.colorScheme.onSurfaceVariant,
                style = MaterialTheme.typography.labelLarge,
            )
            OutlinedTextField(
                value = conn.manualIp,
                onValueChange = onManualIp,
                singleLine = true,
                label = { Text("192.168.x.x") },
                modifier = Modifier.fillMaxWidth(),
            )
            Button(
                onClick = onConnectManual,
                enabled = conn.manualIp.isNotBlank(),
                modifier = Modifier.fillMaxWidth(),
                colors = ButtonDefaults.buttonColors(
                    containerColor = MaterialTheme.colorScheme.primary,
                    contentColor = MaterialTheme.colorScheme.onPrimary,
                ),
            ) { Text("CONNECT TO IP", fontWeight = FontWeight.Bold) }
        }
    }
}

@Composable
private fun ProvisioningCard(
    conn: ConnectionUiState,
    onJoinSoftAp: (String, String) -> Unit,
    onScan: () -> Unit,
    onProvision: (String, String) -> Unit,
) {
    var apSsid by remember { mutableStateOf("") }
    var apPass by remember { mutableStateOf("") }
    var homeSsid by remember { mutableStateOf("") }
    var homePass by remember { mutableStateOf("") }

    Card(
        colors = CardDefaults.cardColors(containerColor = MaterialTheme.colorScheme.surfaceVariant),
        shape = RoundedCornerShape(12.dp),
        modifier = Modifier.fillMaxWidth(),
    ) {
        Column(Modifier.padding(16.dp), verticalArrangement = Arrangement.spacedBy(8.dp)) {
            Text("Provision home WiFi", color = MaterialTheme.colorScheme.primary, style = MaterialTheme.typography.titleLarge)
            Text(
                "1) Join the board AP. 2) Scan/pick home WiFi. 3) Send credentials. " +
                    "The AP link will drop after the board ACKs — this is expected; the app rediscovers on home WiFi.",
                color = MaterialTheme.colorScheme.onSurfaceVariant,
                style = MaterialTheme.typography.labelLarge,
            )

            OutlinedTextField(
                value = apSsid,
                onValueChange = { apSsid = it },
                singleLine = true,
                label = { Text("Board AP SSID (SignalGen-XXXX)") },
                modifier = Modifier.fillMaxWidth(),
            )
            OutlinedTextField(
                value = apPass,
                onValueChange = { apPass = it },
                singleLine = true,
                label = { Text("Board AP passphrase (shown on device)") },
                modifier = Modifier.fillMaxWidth(),
            )
            Button(
                onClick = { onJoinSoftAp(apSsid.trim(), apPass.trim()) },
                enabled = apSsid.isNotBlank() && apPass.length >= 8,
                modifier = Modifier.fillMaxWidth(),
                colors = ButtonDefaults.buttonColors(
                    containerColor = MaterialTheme.colorScheme.primary,
                    contentColor = MaterialTheme.colorScheme.onPrimary,
                ),
            ) { Text("1 · JOIN BOARD AP") }

            Divider(color = MaterialTheme.colorScheme.outlineVariant)

            OutlinedButton(
                onClick = onScan,
                enabled = conn.phase == ConnPhase.CONNECTED && !conn.scanning,
                modifier = Modifier.fillMaxWidth(),
            ) {
                if (conn.scanning) {
                    CircularProgressIndicator(
                        modifier = Modifier.padding(end = 8.dp),
                        strokeWidth = 2.dp,
                        color = MaterialTheme.colorScheme.primary,
                    )
                }
                Text("2 · SCAN HOME NETWORKS")
            }
            conn.scanResults.forEach { net: ScanResultEntry ->
                Row(
                    Modifier
                        .fillMaxWidth()
                        .padding(vertical = 2.dp),
                    verticalAlignment = Alignment.CenterVertically,
                ) {
                    Column(Modifier.weight(1f)) {
                        Text(net.ssid, color = MaterialTheme.colorScheme.onSurface)
                        Text(
                            "rssi ${net.rssi} · ch ${net.ch} · sec ${net.sec}",
                            color = MaterialTheme.colorScheme.onSurfaceVariant,
                            style = MaterialTheme.typography.labelLarge,
                        )
                    }
                    OutlinedButton(onClick = { homeSsid = net.ssid }) { Text("PICK") }
                }
            }

            Divider(color = MaterialTheme.colorScheme.outlineVariant)

            OutlinedTextField(
                value = homeSsid,
                onValueChange = { homeSsid = it },
                singleLine = true,
                label = { Text("Home SSID") },
                modifier = Modifier.fillMaxWidth(),
            )
            OutlinedTextField(
                value = homePass,
                onValueChange = { homePass = it },
                singleLine = true,
                visualTransformation = PasswordVisualTransformation(),
                label = { Text("Home password") },
                modifier = Modifier.fillMaxWidth(),
            )
            Button(
                onClick = { onProvision(homeSsid.trim(), homePass) },
                enabled = homeSsid.isNotBlank(),
                modifier = Modifier.fillMaxWidth(),
                colors = ButtonDefaults.buttonColors(
                    containerColor = MaterialTheme.colorScheme.primary,
                    contentColor = MaterialTheme.colorScheme.onPrimary,
                ),
            ) { Text("3 · SEND CREDENTIALS", fontWeight = FontWeight.Bold) }
        }
    }
}
