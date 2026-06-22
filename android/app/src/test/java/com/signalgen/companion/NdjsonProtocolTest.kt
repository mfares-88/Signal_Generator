package com.signalgen.companion

import com.signalgen.companion.data.net.NdjsonProtocol
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * Round-trips every §4.0 frame and pins the exact wire field names so the firmware
 * (Agent F) hand-rolled flat parser interoperates. Each outbound test asserts the
 * serialized line contains the contract keys; each inbound test asserts a sample
 * firmware-shaped line decodes to the right typed frame.
 */
class NdjsonProtocolTest {

    // ----------------------- Outbound CONTROL frames -----------------------

    @Test
    fun rpm_serializes_with_t_and_v() {
        val line = NdjsonProtocol.encode(NdjsonProtocol.RpmFrame(v = 2500, id = 7))
        assertContainsKv(line, "\"t\":\"rpm\"")
        assertContainsKv(line, "\"v\":2500")
        assertContainsKv(line, "\"id\":7")
    }

    @Test
    fun start_stop_serialize() {
        assertContainsKv(NdjsonProtocol.encode(NdjsonProtocol.StartFrame()), "\"t\":\"start\"")
        assertContainsKv(NdjsonProtocol.encode(NdjsonProtocol.StopFrame()), "\"t\":\"stop\"")
    }

    @Test
    fun invert_serializes() {
        val line = NdjsonProtocol.encode(NdjsonProtocol.InvertFrame(v = 1))
        assertContainsKv(line, "\"t\":\"invert\"")
        assertContainsKv(line, "\"v\":1")
    }

    @Test
    fun selBuiltin_serializes_with_i() {
        val line = NdjsonProtocol.encode(NdjsonProtocol.SelBuiltinFrame(i = 3))
        assertContainsKv(line, "\"t\":\"selBuiltin\"")
        assertContainsKv(line, "\"i\":3")
    }

    @Test
    fun selNamed_serializes_with_key() {
        val line = NdjsonProtocol.encode(NdjsonProtocol.SelNamedFrame(key = "missing36_1"))
        assertContainsKv(line, "\"t\":\"selNamed\"")
        assertContainsKv(line, "\"key\":\"missing36_1\"")
    }

    @Test
    fun loadDsl_serializes_with_src() {
        val line = NdjsonProtocol.encode(NdjsonProtocol.LoadDslFrame(src = "C:S:1/2:60"))
        assertContainsKv(line, "\"t\":\"loadDsl\"")
        assertContainsKv(line, "\"src\":\"C:S:1/2:60\"")
    }

    @Test
    fun sweep_serializes_flat_fields() {
        val line = NdjsonProtocol.encode(NdjsonProtocol.SweepFrame(lo = 600, hi = 6000, mode = 1, iv = 2000))
        assertContainsKv(line, "\"t\":\"sweep\"")
        assertContainsKv(line, "\"lo\":600")
        assertContainsKv(line, "\"hi\":6000")
        assertContainsKv(line, "\"mode\":1")
        assertContainsKv(line, "\"iv\":2000")
    }

    @Test
    fun comp_serializes_flat_fields() {
        val line = NdjsonProtocol.encode(
            NdjsonProtocol.CompFrame(on = true, cyl = 4, thr = 3000, peak = 100, dyn = false)
        )
        assertContainsKv(line, "\"t\":\"comp\"")
        assertContainsKv(line, "\"on\":true")
        assertContainsKv(line, "\"cyl\":4")
        assertContainsKv(line, "\"thr\":3000")
        assertContainsKv(line, "\"peak\":100")
        assertContainsKv(line, "\"dyn\":false")
    }

    @Test
    fun capStart_with_and_without_revs() {
        val withRevs = NdjsonProtocol.encode(NdjsonProtocol.CapStartFrame(revs = 2))
        assertContainsKv(withRevs, "\"t\":\"capStart\"")
        assertContainsKv(withRevs, "\"revs\":2")
        // revs null -> omitted (explicitNulls=false) so firmware defaults to 2.
        val noRevs = NdjsonProtocol.encode(NdjsonProtocol.CapStartFrame())
        assertContainsKv(noRevs, "\"t\":\"capStart\"")
        assertFalse(noRevs.contains("revs"))
    }

    @Test
    fun capStop_serializes() {
        assertContainsKv(NdjsonProtocol.encode(NdjsonProtocol.CapStopFrame()), "\"t\":\"capStop\"")
    }

    @Test
    fun save_serializes_with_key_and_src() {
        val line = NdjsonProtocol.encode(NdjsonProtocol.SaveFrame(key = "myPat", src = "C:S:1/2:60"))
        assertContainsKv(line, "\"t\":\"save\"")
        assertContainsKv(line, "\"key\":\"myPat\"")
        assertContainsKv(line, "\"src\":\"C:S:1/2:60\"")
    }

    // ----------------------- Outbound LINK/QUERY frames -----------------------

    @Test
    fun hello_list_scan_forget_serialize() {
        assertContainsKv(NdjsonProtocol.encode(NdjsonProtocol.HelloFrame()), "\"t\":\"hello\"")
        assertContainsKv(NdjsonProtocol.encode(NdjsonProtocol.ListFrame()), "\"t\":\"list\"")
        assertContainsKv(NdjsonProtocol.encode(NdjsonProtocol.ScanFrame()), "\"t\":\"scan\"")
        assertContainsKv(NdjsonProtocol.encode(NdjsonProtocol.ForgetWifiFrame()), "\"t\":\"forgetWifi\"")
    }

    @Test
    fun provision_serializes_with_ssid_pass() {
        val line = NdjsonProtocol.encode(NdjsonProtocol.ProvisionFrame(ssid = "HomeNet", pass = "secret123"))
        assertContainsKv(line, "\"t\":\"provision\"")
        assertContainsKv(line, "\"ssid\":\"HomeNet\"")
        assertContainsKv(line, "\"pass\":\"secret123\"")
    }

    // ----------------------- Inbound frames -----------------------

    @Test
    fun ack_ok_decodes() {
        val frame = NdjsonProtocol.decode("""{"t":"ack","id":7,"ok":true}""")
        assertTrue(frame is NdjsonProtocol.Ack)
        frame as NdjsonProtocol.Ack
        assertEquals(7, frame.id)
        assertTrue(frame.ok)
        assertNull(frame.err)
    }

    @Test
    fun ack_error_with_offset_decodes() {
        val frame = NdjsonProtocol.decode("""{"t":"ack","id":9,"ok":false,"err":"bad duty","off":12}""")
        assertTrue(frame is NdjsonProtocol.Ack)
        frame as NdjsonProtocol.Ack
        assertFalse(frame.ok)
        assertEquals("bad duty", frame.err)
        assertEquals(12, frame.off)
    }

    @Test
    fun catalog_sequence_decodes() {
        val begin = NdjsonProtocol.decode("""{"t":"catBegin","n":2}""")
        assertTrue(begin is NdjsonProtocol.CatBegin)
        assertEquals(2, (begin as NdjsonProtocol.CatBegin).n)

        val builtin = NdjsonProtocol.decode(
            """{"t":"pat","tier":"builtin","i":0,"key":"missing60_2","name":"60-2","deg":360,"mask":1,"slots":120}"""
        )
        assertTrue(builtin is NdjsonProtocol.PatRow)
        builtin as NdjsonProtocol.PatRow
        assertTrue(builtin.isBuiltin)
        assertEquals(0, builtin.i)
        assertEquals("missing60_2", builtin.key)
        assertEquals("60-2", builtin.displayName)
        assertEquals(360, builtin.deg)
        assertEquals(1, builtin.mask)
        assertEquals(120, builtin.slots)

        // user row: i=-1, name absent -> displayName falls back to key.
        val user = NdjsonProtocol.decode("""{"t":"pat","tier":"user","i":-1,"key":"myPat","deg":720,"mask":3,"slots":240}""")
        user as NdjsonProtocol.PatRow
        assertFalse(user.isBuiltin)
        assertEquals(-1, user.i)
        assertEquals("myPat", user.displayName)

        val end = NdjsonProtocol.decode("""{"t":"catEnd"}""")
        assertTrue(end is NdjsonProtocol.CatEnd)
    }

    @Test
    fun tel_decodes_all_fields() {
        val frame = NdjsonProtocol.decode(
            """{"t":"tel","rpm":2500,"baseRpm":2400,"run":true,"pat":"missing60_2","deg":360,"mask":1,"inv":false,"edge":1234,"cycUs":48000,"drop":0}"""
        )
        assertTrue(frame is NdjsonProtocol.Tel)
        frame as NdjsonProtocol.Tel
        assertEquals(2500, frame.rpm)
        assertEquals(2400, frame.baseRpm)
        assertTrue(frame.run)
        assertEquals("missing60_2", frame.pat)
        assertEquals(360, frame.deg)
        assertEquals(1, frame.mask)
        assertFalse(frame.inv)
        assertEquals(1234, frame.edge)
        assertEquals(48000L, frame.cycUs)
        assertEquals(0, frame.drop)
    }

    @Test
    fun link_ap_mode_decodes() {
        val frame = NdjsonProtocol.decode(
            """{"t":"link","mode":"AP","ssid":"SignalGen-1A2B","ip":"192.168.4.1","rssi":0,"mdns":"siggen","appw":"siggen-aabbccdd"}"""
        )
        assertTrue(frame is NdjsonProtocol.Link)
        frame as NdjsonProtocol.Link
        assertEquals("AP", frame.mode)
        assertEquals("SignalGen-1A2B", frame.ssid)
        assertEquals("192.168.4.1", frame.ip)
        assertEquals("siggen", frame.mdns)
        assertEquals("siggen-aabbccdd", frame.appw)
    }

    @Test
    fun link_sta_mode_decodes() {
        val frame = NdjsonProtocol.decode("""{"t":"link","mode":"STA","ssid":"HomeNet","ip":"192.168.1.42","rssi":-55,"mdns":"siggen"}""")
        frame as NdjsonProtocol.Link
        assertEquals("STA", frame.mode)
        assertEquals("192.168.1.42", frame.ip)
        assertEquals(-55, frame.rssi)
        assertNull(frame.appw)
    }

    @Test
    fun scanRes_decodes_nets() {
        val frame = NdjsonProtocol.decode(
            """{"t":"scanRes","nets":[{"ssid":"HomeNet","rssi":-50,"sec":3,"ch":6},{"ssid":"Other","rssi":-70,"sec":0,"ch":11}]}"""
        )
        assertTrue(frame is NdjsonProtocol.ScanRes)
        frame as NdjsonProtocol.ScanRes
        assertEquals(2, frame.nets.size)
        assertEquals("HomeNet", frame.nets[0].ssid)
        assertEquals(6, frame.nets[0].ch)
        assertEquals(11, frame.nets[1].ch)
    }

    // ----------------------- Robustness -----------------------

    @Test
    fun unknown_type_decodes_to_null() {
        assertNull(NdjsonProtocol.decode("""{"t":"bogus","x":1}"""))
    }

    @Test
    fun blank_and_malformed_decode_to_null() {
        assertNull(NdjsonProtocol.decode(""))
        assertNull(NdjsonProtocol.decode("   "))
        assertNull(NdjsonProtocol.decode("not json"))
        assertNull(NdjsonProtocol.decode("""{"no_type":1}"""))
    }

    @Test
    fun ignores_unknown_fields_in_known_frame() {
        // Firmware may add fields later — must not break decoding.
        val frame = NdjsonProtocol.decode("""{"t":"tel","rpm":1000,"future":42}""")
        assertTrue(frame is NdjsonProtocol.Tel)
        assertEquals(1000, (frame as NdjsonProtocol.Tel).rpm)
    }

    @Test
    fun passphrase_derivation_matches_firmware_d14() {
        val mac = byteArrayOf(0x24, 0x6F, 0x28.toByte(), 0xAA.toByte(), 0xBB.toByte(), 0xCC.toByte())
        assertEquals(
            "siggen-28aabbcc",
            com.signalgen.companion.data.ProvisioningManager.derivePassphrase(mac)
        )
        assertEquals(
            "SignalGen-BBCC",
            com.signalgen.companion.data.ProvisioningManager.deriveSsid(mac)
        )
    }

    private fun assertContainsKv(line: String, kv: String) {
        assertTrue("expected `$kv` in: $line", line.contains(kv))
    }
}
