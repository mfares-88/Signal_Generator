package com.signalgen.companion.data

import com.signalgen.companion.data.net.NdjsonProtocol

/**
 * UI-facing catalog + scan model types (plan §4.5/§4.6). These are the stable,
 * decoupled shapes the Compose screens (Agents N2/N3) bind to — separate from the
 * raw wire frames in [NdjsonProtocol] so the UI never depends on serialization
 * details.
 */

/**
 * One pattern in the board's catalog.
 *
 *  - [key]   : stable name_key; select user patterns via selNamed(key).
 *  - [index] : builtinByIndex order for builtin rows (select via selBuiltin(index));
 *              -1 for user rows.
 *  - [tier]  : "builtin" | "user".
 *  - [name]  : display name, already null-guarded to the key.
 *  - [deg]   : 360 or 720 crank degrees.
 *  - [mask]  : channel mask (bit0 crank, bit1 cam1, bit2 cam2).
 *  - [slots] : compiled slot count.
 */
data class PatternEntry(
    val key: String,
    val index: Int,
    val tier: String,
    val name: String,
    val deg: Int,
    val mask: Int,
    val slots: Int,
) {
    val isBuiltin: Boolean get() = tier == "builtin"

    companion object {
        fun from(row: NdjsonProtocol.PatRow): PatternEntry = PatternEntry(
            key = row.key,
            index = row.i,
            tier = row.tier,
            name = row.displayName,
            deg = row.deg,
            mask = row.mask,
            slots = row.slots,
        )
    }
}

/**
 * One Wi-Fi network from the board's scan (provisioning picker).
 *  - [ssid] : network name.
 *  - [rssi] : signal strength (dBm).
 *  - [sec]  : security type code (firmware-defined; 0 = open).
 *  - [ch]   : channel (used to narrow the SoftAP-drop window, plan §2.4 step 4).
 */
data class ScanResultEntry(
    val ssid: String,
    val rssi: Int,
    val sec: Int,
    val ch: Int,
) {
    companion object {
        fun from(net: NdjsonProtocol.ScanNet): ScanResultEntry =
            ScanResultEntry(ssid = net.ssid, rssi = net.rssi, sec = net.sec, ch = net.ch)
    }
}
