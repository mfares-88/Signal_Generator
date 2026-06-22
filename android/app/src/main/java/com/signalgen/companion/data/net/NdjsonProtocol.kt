package com.signalgen.companion.data.net

import kotlinx.serialization.SerialName
import kotlinx.serialization.Serializable
import kotlinx.serialization.encodeToString
import kotlinx.serialization.json.Json

/**
 * Canonical Android side of the SHARED NDJSON protocol (plan §4.0).
 *
 * One compact JSON object per line, '\n'-terminated, UTF-8. Every frame carries a
 * "t" type discriminator. Field names are EXACTLY as specified in §4.0 so the
 * firmware (Agent F) hand-rolled flat-frame parser interoperates verbatim.
 *
 * Outbound frames (phone -> board) are CONTROL frames (become sendCtrlMsg() calls)
 * and LINK/QUERY frames (handled inside wifi_link, not queued).
 *
 * Inbound frames (board -> phone) are ack / catalog (catBegin,pat,catEnd) /
 * tel / link / scanRes.
 *
 * NOTE on serialization: kotlinx-serialization only serializes CONSTRUCTOR
 * properties. The "t" discriminator is therefore a constructor parameter with a
 * fixed default; with [json] `encodeDefaults=true` it is always emitted on the wire.
 *
 * The flat-only design (sweep/comp fields are flat, NOT nested objects) is kept here
 * too so both ends share the identical wire shape.
 */
object NdjsonProtocol {

    /**
     * Shared Json configuration.
     * - ignoreUnknownKeys: forward/backward compatible with firmware additions.
     * - encodeDefaults: emit default-valued fields (incl. the "t" discriminator) so
     *   the firmware flat parser always sees the keys it scans for.
     * - explicitNulls=false: omit nulls (firmware never sends/expects JSON null).
     */
    val json: Json = Json {
        ignoreUnknownKeys = true
        encodeDefaults = true
        explicitNulls = false
        isLenient = true
    }

    // ---------------------------------------------------------------------------
    // Outbound: CONTROL frames (phone -> board -> sendCtrlMsg())
    // ---------------------------------------------------------------------------

    @Serializable
    data class RpmFrame(
        @SerialName("v") val v: Int,
        @SerialName("id") val id: Int? = null,
        @SerialName("t") val t: String = "rpm",
    )

    @Serializable
    data class StartFrame(
        @SerialName("id") val id: Int? = null,
        @SerialName("t") val t: String = "start",
    )

    @Serializable
    data class StopFrame(
        @SerialName("id") val id: Int? = null,
        @SerialName("t") val t: String = "stop",
    )

    @Serializable
    data class InvertFrame(
        @SerialName("v") val v: Int,
        @SerialName("id") val id: Int? = null,
        @SerialName("t") val t: String = "invert",
    )

    @Serializable
    data class SelBuiltinFrame(
        @SerialName("i") val i: Int,
        @SerialName("id") val id: Int? = null,
        @SerialName("t") val t: String = "selBuiltin",
    )

    @Serializable
    data class SelNamedFrame(
        @SerialName("key") val key: String,
        @SerialName("id") val id: Int? = null,
        @SerialName("t") val t: String = "selNamed",
    )

    @Serializable
    data class LoadDslFrame(
        @SerialName("src") val src: String,
        @SerialName("id") val id: Int? = null,
        @SerialName("t") val t: String = "loadDsl",
    )

    @Serializable
    data class SweepFrame(
        @SerialName("lo") val lo: Int,
        @SerialName("hi") val hi: Int,
        @SerialName("mode") val mode: Int,
        @SerialName("iv") val iv: Int,
        @SerialName("id") val id: Int? = null,
        @SerialName("t") val t: String = "sweep",
    )

    @Serializable
    data class CompFrame(
        @SerialName("on") val on: Boolean,
        @SerialName("cyl") val cyl: Int,
        @SerialName("thr") val thr: Int,
        @SerialName("peak") val peak: Int,
        @SerialName("dyn") val dyn: Boolean,
        @SerialName("id") val id: Int? = null,
        @SerialName("t") val t: String = "comp",
    )

    @Serializable
    data class CapStartFrame(
        @SerialName("revs") val revs: Int? = null,
        @SerialName("id") val id: Int? = null,
        @SerialName("t") val t: String = "capStart",
    )

    @Serializable
    data class CapStopFrame(
        @SerialName("id") val id: Int? = null,
        @SerialName("t") val t: String = "capStop",
    )

    @Serializable
    data class SaveFrame(
        @SerialName("key") val key: String,
        @SerialName("src") val src: String,
        @SerialName("id") val id: Int? = null,
        @SerialName("t") val t: String = "save",
    )

    // ---------------------------------------------------------------------------
    // Outbound: LINK / QUERY frames (phone -> board, handled inside wifi_link)
    // ---------------------------------------------------------------------------

    @Serializable
    data class HelloFrame(
        @SerialName("id") val id: Int? = null,
        @SerialName("t") val t: String = "hello",
    )

    @Serializable
    data class ListFrame(
        @SerialName("id") val id: Int? = null,
        @SerialName("t") val t: String = "list",
    )

    @Serializable
    data class ProvisionFrame(
        @SerialName("ssid") val ssid: String,
        @SerialName("pass") val pass: String,
        @SerialName("id") val id: Int? = null,
        @SerialName("t") val t: String = "provision",
    )

    @Serializable
    data class ScanFrame(
        @SerialName("id") val id: Int? = null,
        @SerialName("t") val t: String = "scan",
    )

    @Serializable
    data class ForgetWifiFrame(
        @SerialName("id") val id: Int? = null,
        @SerialName("t") val t: String = "forgetWifi",
    )

    // ---------------------------------------------------------------------------
    // Inbound: board -> phone frames
    // ---------------------------------------------------------------------------

    /**
     * Sealed type for every inbound frame the board can push. Decoded by inspecting
     * the "t" discriminator (manual dispatch — see [decode]) so we can tolerate the
     * firmware's flat, hand-serialized wire format without a polymorphic class
     * discriminator wrapper.
     */
    sealed interface Inbound

    @Serializable
    data class Ack(
        @SerialName("id") val id: Int? = null,
        @SerialName("ok") val ok: Boolean = false,
        @SerialName("err") val err: String? = null,
        // off only present for DSL compile errors (character offset into the source).
        @SerialName("off") val off: Int? = null,
    ) : Inbound

    @Serializable
    data class CatBegin(
        @SerialName("n") val n: Int = 0,
    ) : Inbound

    /**
     * One catalog row. builtin rows: tier="builtin", i = builtinByIndex order
     * (matches selBuiltin). user rows: tier="user", i=-1, select by key (selNamed).
     */
    @Serializable
    data class PatRow(
        @SerialName("tier") val tier: String = "builtin",
        @SerialName("i") val i: Int = -1,
        @SerialName("key") val key: String = "",
        @SerialName("name") val name: String? = null,
        @SerialName("deg") val deg: Int = 360,
        @SerialName("mask") val mask: Int = 0,
        @SerialName("slots") val slots: Int = 0,
    ) : Inbound {
        val isBuiltin: Boolean get() = tier == "builtin"
        /** Display name, null-guarded to the key (firmware falls back to name_key). */
        val displayName: String get() = name ?: key
    }

    @Serializable
    class CatEnd : Inbound

    @Serializable
    data class Tel(
        @SerialName("rpm") val rpm: Int = 0,        // sweepCurrentRpm (effective live RPM)
        @SerialName("baseRpm") val baseRpm: Int = 0, // g_rpm (slider/base RPM)
        @SerialName("run") val run: Boolean = false,
        @SerialName("pat") val pat: String? = null,  // active name_key
        @SerialName("deg") val deg: Int = 360,
        @SerialName("mask") val mask: Int = 0,
        @SerialName("inv") val inv: Boolean = false,
        @SerialName("edge") val edge: Int = 0,       // edgeCounter (wrapping u16)
        @SerialName("cycUs") val cycUs: Long = 0,    // full-table cycle_duration_us
        @SerialName("drop") val drop: Int = 0,       // gUiMsgDropCount
    ) : Inbound

    @Serializable
    data class Link(
        // mode: "AP" | "STA" | "CONNECTING" | "DISCONNECTED"
        @SerialName("mode") val mode: String = "DISCONNECTED",
        @SerialName("ssid") val ssid: String? = null,
        @SerialName("ip") val ip: String? = null,
        @SerialName("rssi") val rssi: Int = 0,
        @SerialName("mdns") val mdns: String? = null,
        // appw: softAP passphrase, AP mode only.
        @SerialName("appw") val appw: String? = null,
    ) : Inbound

    @Serializable
    data class ScanNet(
        @SerialName("ssid") val ssid: String = "",
        @SerialName("rssi") val rssi: Int = 0,
        @SerialName("sec") val sec: Int = 0,
        @SerialName("ch") val ch: Int = 0,
    )

    @Serializable
    data class ScanRes(
        @SerialName("nets") val nets: List<ScanNet> = emptyList(),
    ) : Inbound

    // ---------------------------------------------------------------------------
    // Encoding (outbound)
    // ---------------------------------------------------------------------------

    fun encode(frame: RpmFrame): String = json.encodeToString(frame)
    fun encode(frame: StartFrame): String = json.encodeToString(frame)
    fun encode(frame: StopFrame): String = json.encodeToString(frame)
    fun encode(frame: InvertFrame): String = json.encodeToString(frame)
    fun encode(frame: SelBuiltinFrame): String = json.encodeToString(frame)
    fun encode(frame: SelNamedFrame): String = json.encodeToString(frame)
    fun encode(frame: LoadDslFrame): String = json.encodeToString(frame)
    fun encode(frame: SweepFrame): String = json.encodeToString(frame)
    fun encode(frame: CompFrame): String = json.encodeToString(frame)
    fun encode(frame: CapStartFrame): String = json.encodeToString(frame)
    fun encode(frame: CapStopFrame): String = json.encodeToString(frame)
    fun encode(frame: SaveFrame): String = json.encodeToString(frame)
    fun encode(frame: HelloFrame): String = json.encodeToString(frame)
    fun encode(frame: ListFrame): String = json.encodeToString(frame)
    fun encode(frame: ProvisionFrame): String = json.encodeToString(frame)
    fun encode(frame: ScanFrame): String = json.encodeToString(frame)
    fun encode(frame: ForgetWifiFrame): String = json.encodeToString(frame)

    // ---------------------------------------------------------------------------
    // Decoding (inbound) — dispatch on the "t" discriminator.
    // ---------------------------------------------------------------------------

    /**
     * Decode a single NDJSON line (without trailing newline) into an [Inbound] frame.
     * Returns null for blank lines, unknown "t" values, or malformed JSON — callers
     * simply ignore nulls (robust against partial/garbage lines).
     */
    fun decode(line: String): Inbound? {
        val trimmed = line.trim()
        if (trimmed.isEmpty()) return null
        return try {
            val type = extractType(trimmed) ?: return null
            when (type) {
                "ack" -> json.decodeFromString<Ack>(trimmed)
                "catBegin" -> json.decodeFromString<CatBegin>(trimmed)
                "pat" -> json.decodeFromString<PatRow>(trimmed)
                "catEnd" -> json.decodeFromString<CatEnd>(trimmed)
                "tel" -> json.decodeFromString<Tel>(trimmed)
                "link" -> json.decodeFromString<Link>(trimmed)
                "scanRes" -> json.decodeFromString<ScanRes>(trimmed)
                else -> null
            }
        } catch (e: Exception) {
            null
        }
    }

    /**
     * Pull the value of the top-level "t" string field without a full typed parse, so
     * we can route to the right @Serializable class. The frames are flat single-line
     * objects, so a lightweight scan is sufficient and avoids a polymorphic wrapper.
     */
    private fun extractType(line: String): String? {
        val element = try {
            json.parseToJsonElement(line)
        } catch (e: Exception) {
            return null
        }
        val obj = element as? kotlinx.serialization.json.JsonObject ?: return null
        val tField = obj["t"] ?: return null
        val prim = tField as? kotlinx.serialization.json.JsonPrimitive ?: return null
        if (!prim.isString) return null
        return prim.content
    }
}
