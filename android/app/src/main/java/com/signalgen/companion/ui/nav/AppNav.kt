package com.signalgen.companion.ui.nav

import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.background
import androidx.compose.foundation.clickable
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.Build
import androidx.compose.material.icons.filled.Home
import androidx.compose.material.icons.filled.Settings
import androidx.compose.material.icons.filled.Wifi
import androidx.compose.material3.Icon
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.NavigationBar
import androidx.compose.material3.NavigationBarItem
import androidx.compose.material3.Scaffold
import androidx.compose.material3.Surface
import androidx.compose.material3.Text
import androidx.compose.material3.TopAppBar
import androidx.compose.material3.TopAppBarDefaults
import androidx.compose.material3.ExperimentalMaterial3Api
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.vector.ImageVector
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.unit.dp
import androidx.lifecycle.compose.collectAsStateWithLifecycle
import androidx.lifecycle.viewmodel.compose.viewModel
import androidx.navigation.NavDestination.Companion.hierarchy
import androidx.navigation.NavGraph.Companion.findStartDestination
import androidx.navigation.NavHostController
import androidx.navigation.compose.NavHost
import androidx.navigation.compose.composable
import androidx.navigation.compose.currentBackStackEntryAsState
import androidx.navigation.compose.rememberNavController
import com.signalgen.companion.data.DeviceState
import com.signalgen.companion.ui.AdvancedScreen
import com.signalgen.companion.ui.ConnPhase
import com.signalgen.companion.ui.ConnectionScreen
import com.signalgen.companion.ui.ConnectionViewModel
import com.signalgen.companion.ui.HomeScreen
import com.signalgen.companion.ui.CustomBuilderScreen

private sealed class Dest(val route: String, val label: String, val icon: ImageVector) {
    data object Home : Dest("home", "HOME", Icons.Filled.Home)
    data object Custom : Dest("custom", "CUSTOM", Icons.Filled.Build)
    data object Advanced : Dest("advanced", "ADVANCED", Icons.Filled.Settings)
    data object Connection : Dest("connection", "LINK", Icons.Filled.Wifi)
}

private val BOTTOM_DESTS = listOf(Dest.Home, Dest.Custom, Dest.Advanced, Dest.Connection)

@OptIn(ExperimentalMaterial3Api::class)
@Composable
fun AppNav() {
    val nav = rememberNavController()
    val vm: ConnectionViewModel = viewModel()
    val state by vm.deviceState.collectAsStateWithLifecycle()
    val catalog by vm.catalog.collectAsStateWithLifecycle()
    val conn by vm.connUi.collectAsStateWithLifecycle()

    Scaffold(
        topBar = {
            TopAppBar(
                title = { Text("Signal Generator", color = MaterialTheme.colorScheme.onBackground) },
                actions = { ConnectionChip(state = state, connected = conn.phase == ConnPhase.CONNECTED) },
                colors = TopAppBarDefaults.topAppBarColors(
                    containerColor = MaterialTheme.colorScheme.background,
                ),
            )
        },
        bottomBar = { BottomBar(nav) },
        containerColor = MaterialTheme.colorScheme.background,
    ) { inner ->
        NavHost(
            navController = nav,
            startDestination = Dest.Home.route,
            modifier = Modifier.padding(inner),
        ) {
            composable(Dest.Home.route) {
                HomeScreen(
                    state = state,
                    catalog = catalog,
                    onRpm = vm::setRpm,
                    onStart = vm::start,
                    onStop = vm::stop,
                    onInvert = vm::setInvert,
                    onSelectBuiltin = vm::selectBuiltin,
                    onSelectNamed = vm::selectNamed,
                )
            }
            composable(Dest.Custom.route) {
                // Agent N3 owns this composable; it receives suspend lambdas
                // bridged to the shared ConnectionViewModel (loadDsl / loadDsl+start).
                CustomBuilderScreen(
                    onApply = { dsl -> vm.loadDsl(dsl) },
                    onApplyStart = { dsl ->
                        vm.loadDsl(dsl)
                        vm.start()
                    },
                )
            }
            composable(Dest.Advanced.route) {
                AdvancedScreen(
                    state = state,
                    catalog = catalog,
                    onSweep = vm::setSweep,
                    onCompression = vm::setCompression,
                    onSelectBuiltin = vm::selectBuiltin,
                    onSelectNamed = vm::selectNamed,
                    onCaptureStart = vm::captureStart,
                    onCaptureStop = vm::captureStop,
                    onOpenWaveform = { nav.navigate(Dest.Custom.route) { launchSingleTop = true } },
                )
            }
            composable(Dest.Connection.route) {
                ConnectionScreen(
                    state = state,
                    conn = conn,
                    onManualIp = vm::setManualIp,
                    onConnectManual = vm::connectManual,
                    onConnectDiscovered = { host, port -> vm.connect(host, port) },
                    onStartDiscovery = vm::startDiscovery,
                    onStopDiscovery = vm::stopDiscovery,
                    onDisconnect = vm::disconnect,
                    onJoinSoftAp = vm::joinSoftAp,
                    onScan = vm::scanNetworks,
                    onProvision = vm::provision,
                    onForget = vm::forgetWifi,
                )
            }
        }
    }
}

@Composable
private fun BottomBar(nav: NavHostController) {
    val backStack by nav.currentBackStackEntryAsState()
    val current = backStack?.destination
    NavigationBar(containerColor = MaterialTheme.colorScheme.surface) {
        BOTTOM_DESTS.forEach { dest ->
            val selected = current?.hierarchy?.any { it.route == dest.route } == true
            NavigationBarItem(
                selected = selected,
                onClick = {
                    nav.navigate(dest.route) {
                        popUpTo(nav.graph.findStartDestination().id) { saveState = true }
                        launchSingleTop = true
                        restoreState = true
                    }
                },
                icon = { Icon(dest.icon, contentDescription = dest.label) },
                label = { Text(dest.label) },
            )
        }
    }
}

@Composable
private fun ConnectionChip(state: DeviceState, connected: Boolean) {
    val color = if (connected) MaterialTheme.colorScheme.primary else MaterialTheme.colorScheme.outlineVariant
    Surface(
        color = MaterialTheme.colorScheme.surfaceVariant,
        shape = RoundedCornerShape(16.dp),
        modifier = Modifier.padding(end = 12.dp),
    ) {
        Row(
            modifier = Modifier.padding(horizontal = 10.dp, vertical = 6.dp),
            verticalAlignment = Alignment.CenterVertically,
        ) {
            Box(
                modifier = Modifier
                    .size(10.dp)
                    .clip(CircleShape)
                    .background(color),
            )
            Text(
                text = "  " + (state.link.ip.ifBlank { if (connected) "connected" else "offline" }),
                color = MaterialTheme.colorScheme.onSurface,
                fontFamily = FontFamily.Monospace,
                style = MaterialTheme.typography.labelLarge,
            )
        }
    }
}
