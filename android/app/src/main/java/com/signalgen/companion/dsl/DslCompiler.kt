package com.signalgen.companion.dsl

/**
 * DslCompiler — a FAITHFUL Kotlin port of the firmware DSL pipeline so the phone
 * can compile/validate/preview a pattern LOCALLY (plan D4) and render a
 * BIT-IDENTICAL waveform without reading the compiled table back from the board.
 *
 * Ported verbatim from:
 *   - lib/dsl/Compiler.cpp   (expandWheel, CCW reversal, CW cam-doubling,
 *                             LCM stretch-merge, canonicalization, bit-pack)
 *   - lib/dsl/Validator.cpp  (the 12 semantic rules, incl. the 512-char limit)
 *   - lib/ui_lvgl/ui_lvgl.cpp (cb_emit_dsl / cb_emit_wheel for the DSL string)
 *
 * IMPORTANT fidelity notes (these were the adversarial-review catches, plan §9):
 *   1. The PREVIEW must match the REAL compiler, NOT the LVGL preview shortcut.
 *      The LVGL `cb_expand_wheel` doubles ONLY the crank and OMITS the CCW
 *      reversal (it is a phase-only approximation). The real Compiler.cpp:
 *        - reverses EVERY CCW wheel's slot vector (Compiler.cpp:207-217), and
 *        - doubles EVERY CW wheel in a 720 deg group (Compiler.cpp:220-224).
 *      We port the REAL compiler so the on-phone preview is bit-identical to
 *      what the board actually generates after APPLY.
 *   2. The 512-char DSL-source limit (Validator.cpp:71-74, rule #11) is enforced
 *      against the EMITTED DSL string so the UI char-counter blocks oversize sends.
 */
object DslCompiler {

    // ---------------------------------------------------------------------------
    // Validation
    // ---------------------------------------------------------------------------

    /** A single validation failure: which channel (or -1 for the whole group) and why. */
    data class ValidationError(val channel: Int, val rule: Int, val message: String)

    /** Result of validating a full [BuilderState]. */
    data class ValidationResult(
        val ok: Boolean,
        val errors: List<ValidationError>,
        /** Length of the emitted DSL string (always computed for the char-counter). */
        val dslLength: Int,
    ) {
        val firstError: ValidationError? get() = errors.firstOrNull()
    }

    /**
     * Native (pre-doubling) slot count for one channel.
     *   S/M -> teeth * dutyDen ; A -> sum(degrees).
     * Mirrors Validator.cpp expandedSlotCount + ui_lvgl cb_native_slots.
     */
    fun nativeSlots(spec: ChannelSpec): Int = when (spec.kind) {
        WheelKind.ANG -> spec.angular.sumOf { if (it > 0) it else 0 }
        else -> spec.teeth * spec.dutyDen
    }

    /**
     * Validate the whole builder state against the firmware rules.
     *
     * Per-channel rules (Validator.cpp + ui_lvgl cb_validate):
     *   - duty: 0 < n < d, d <= 32 (rule #4)
     *   - teeth >= 1 (and <= TEETH_MAX)
     *   - Missing: run-list sums to teeth AND has >= 1 'm' gap (rule #6)
     *   - Angular: degrees all > 0 (rule #8) AND sum == span (360 CW / 720 CCW) (rule #7)
     *   - per-wheel native slots in 2..4096 (rules #5, #9)
     * Group rules:
     *   - LCM of all enabled wheels in 2..4096 (rule #9)
     *   - emitted DSL source length <= 512 (rule #11)
     */
    fun validate(state: BuilderState): ValidationResult {
        val errors = mutableListOf<ValidationError>()

        for (ch in 0 until DslLimits.N_CHANNELS) {
            if (!state.isEnabled(ch)) continue
            validateChannel(ch, state, errors)
        }

        // Group LCM (rule #9) — only meaningful if every channel passed so far.
        if (errors.isEmpty()) {
            val lcm = computeLcm(state)
            when {
                lcm < 2 -> errors.add(ValidationError(-1, 9, "compiled to < 2 slots"))
                lcm > DslLimits.SLOT_MAX ->
                    errors.add(ValidationError(-1, 9, "LCM exceeds ${DslLimits.SLOT_MAX}-slot limit"))
            }
        }

        // Rule #11 — emitted DSL source length. Always compute the length so the
        // UI can show a live char counter even when other rules also fail.
        val dsl = emitDsl(state)
        if (dsl.length > DslLimits.SRC_MAX) {
            errors.add(ValidationError(-1, 11, "DSL source exceeds ${DslLimits.SRC_MAX} characters"))
        }

        return ValidationResult(errors.isEmpty(), errors, dsl.length)
    }

    private fun validateChannel(ch: Int, state: BuilderState, errors: MutableList<ValidationError>) {
        val spec = state.channels[ch]
        when (spec.kind) {
            WheelKind.SYM, WheelKind.MISS -> {
                if (spec.dutyNum < 1) errors.add(ValidationError(ch, 4, "duty numerator n >= 1"))
                if (spec.dutyDen < 2) errors.add(ValidationError(ch, 4, "duty denom d >= 2"))
                if (spec.dutyDen > DslLimits.DUTY_DEN_MAX)
                    errors.add(ValidationError(ch, 4, "duty denom d <= ${DslLimits.DUTY_DEN_MAX}"))
                if (spec.dutyNum >= spec.dutyDen) errors.add(ValidationError(ch, 4, "need n < d"))
                if (spec.teeth < 1) errors.add(ValidationError(ch, 5, "teeth >= 1"))
                if (spec.teeth > DslLimits.TEETH_MAX)
                    errors.add(ValidationError(ch, 5, "teeth <= ${DslLimits.TEETH_MAX}"))
            }
            WheelKind.ANG -> { /* duty not applicable */ }
        }

        if (spec.kind == WheelKind.MISS) {
            var present = 0
            var miss = 0
            spec.runs.forEachIndexed { i, r ->
                val c = if (r.count > 0) r.count else 0
                if (r.isTeeth) present += c else miss += c
                if (r.count < 1) errors.add(ValidationError(ch, 6, "run #${i + 1} count >= 1"))
            }
            if (present + miss != spec.teeth)
                errors.add(ValidationError(ch, 6, "run rows sum ${present + miss} != teeth ${spec.teeth}"))
            if (miss < 1) errors.add(ValidationError(ch, 6, "need >= 1 missing (m) row"))
        }

        if (spec.kind == WheelKind.ANG) {
            val want = state.spanFor(ch)
            var sum = 0
            spec.angular.forEachIndexed { i, deg ->
                if (deg <= 0) errors.add(ValidationError(ch, 8, "degree #${i + 1} > 0"))
                sum += deg
            }
            if (sum != want) errors.add(ValidationError(ch, 7, "degrees sum $sum != $want"))
        }

        val slots = nativeSlots(spec)
        if (slots < 2) errors.add(ValidationError(ch, 5, "slots >= 2"))
        if (slots > DslLimits.SLOT_MAX) errors.add(ValidationError(ch, 9, "slots <= ${DslLimits.SLOT_MAX}"))
    }

    // ---------------------------------------------------------------------------
    // Per-wheel expansion — Compiler.cpp expandWheel + rotation/doubling (step 1-3)
    // ---------------------------------------------------------------------------

    /**
     * Expand one channel into its 0/1 slot vector, applying:
     *   - S/M/A base expansion (Compiler.cpp expandWheel, lines 87-125),
     *   - CCW reversal for cam wheels (Compiler.cpp:207-217) — done BEFORE doubling,
     *   - CW per-wheel x2 doubling when the group is 720 deg (Compiler.cpp:220-224).
     *
     * This is BIT-IDENTICAL to the real firmware compiler's per-wheel output.
     *
     * @param ch          channel index (0 crank / 1 cam1 / 2 cam2).
     * @param groupIs720  whether ANY cam is enabled (the group span is 720 deg).
     */
    fun expandWheel(ch: Int, state: BuilderState, groupIs720: Boolean): IntArray {
        val spec = state.channels[ch]
        val out = ArrayList<Int>()

        when (spec.kind) {
            WheelKind.SYM -> {
                // total_teeth slots of: dutyNum HIGH then (dutyDen - dutyNum) LOW.
                for (i in 0 until spec.teeth) {
                    repeat(spec.dutyNum) { out.add(1) }
                    repeat(spec.dutyDen - spec.dutyNum) { out.add(0) }
                }
            }
            WheelKind.MISS -> {
                // Walk the run-list: 't' runs emit present teeth, 'm' runs emit
                // (count * dutyDen) LOW slots (Compiler.cpp:103-113).
                for (r in spec.runs) {
                    if (r.isTeeth) {
                        repeat(r.count) {
                            repeat(spec.dutyNum) { out.add(1) }
                            repeat(spec.dutyDen - spec.dutyNum) { out.add(0) }
                        }
                    } else {
                        val gapSlots = r.count * spec.dutyDen
                        repeat(gapSlots) { out.add(0) }
                    }
                }
            }
            WheelKind.ANG -> {
                // 1 slot/degree, alternating HIGH/LOW starting HIGH (Compiler.cpp:116-122).
                var level = 1
                for (deg in spec.angular) {
                    repeat(deg) { out.add(level) }
                    level = if (level == 1) 0 else 1
                }
            }
        }

        // CCW reversal (Compiler.cpp:207-217). Cams are CCW; crank is CW.
        if (BuilderState.rotationOf(ch) == Rotation.CCW) {
            out.reverse()
        }

        // CW cam-doubling in a 720 deg group (Compiler.cpp:220-224): repeat x2 so
        // every CW wheel spans the same angular period as the 720 deg cams.
        if (groupIs720 && BuilderState.rotationOf(ch) == Rotation.CW) {
            val base = out.size
            for (k in 0 until base) out.add(out[k])
        }

        return out.toIntArray()
    }

    // ---------------------------------------------------------------------------
    // LCM helpers — Compiler.cpp gcd_u32 / lcm_u32
    // ---------------------------------------------------------------------------

    private fun gcd(a: Int, b: Int): Int {
        var x = a
        var y = b
        while (y != 0) { val r = x % y; x = y; y = r }
        return x
    }

    /** LCM with the firmware's overflow sentinel: returns SLOT_MAX+1 if it exceeds the cap. */
    private fun lcm(a: Int, b: Int): Int {
        if (a == 0 || b == 0) return 0
        val g = gcd(a, b)
        val l = (a.toLong() / g) * b.toLong()
        return if (l > DslLimits.SLOT_MAX.toLong()) DslLimits.SLOT_MAX + 1 else l.toInt()
    }

    /**
     * Compute the merged LCM slot count over all enabled channels (post doubling),
     * mirroring Compiler.cpp step 4. Returns SLOT_MAX+1 on overflow.
     */
    fun computeLcm(state: BuilderState): Int {
        val is720 = state.hasCam()
        var l = 0
        for (ch in 0 until DslLimits.N_CHANNELS) {
            if (!state.isEnabled(ch)) continue
            val n = expandWheel(ch, state, is720).size
            if (n <= 0) return DslLimits.SLOT_MAX + 1
            l = if (l == 0) n else lcm(l, n)
            if (l > DslLimits.SLOT_MAX) return DslLimits.SLOT_MAX + 1
        }
        return l
    }

    // ---------------------------------------------------------------------------
    // compiledStats — full LCM stretch-merge to a bit-packed table (Compiler.cpp step 4-6)
    // ---------------------------------------------------------------------------

    /**
     * The result of a local compile: the bit-packed slot table plus metadata,
     * matching the firmware [PatternRef] fields the phone cares about.
     *
     * @property table        L bytes; bit0=crank, bit1=cam1, bit2=cam2 (Compiler.cpp:257-263).
     * @property slotCount    L = LCM of all enabled per-wheel lengths.
     * @property degrees      360 (no cam) or 720 (any cam) — the FULL table span.
     * @property channelMask  OR of bitOf(ch) for every enabled channel.
     */
    data class CompiledStats(
        val ok: Boolean,
        val table: ByteArray,
        val slotCount: Int,
        val degrees: Int,
        val channelMask: Int,
        val error: String? = null,
    ) {
        override fun equals(other: Any?): Boolean {
            if (this === other) return true
            if (other !is CompiledStats) return false
            return ok == other.ok && slotCount == other.slotCount &&
                degrees == other.degrees && channelMask == other.channelMask &&
                table.contentEquals(other.table)
        }

        override fun hashCode(): Int {
            var result = ok.hashCode()
            result = 31 * result + slotCount
            result = 31 * result + degrees
            result = 31 * result + channelMask
            result = 31 * result + table.contentHashCode()
            return result
        }
    }

    /**
     * Compile the builder state to a bit-packed slot table — a faithful port of
     * Compiler.cpp::dslCompileAst steps 4-6 (LCM stretch-merge, canonicalize, pack).
     *
     * The stretch rule (Compiler.cpp:261-263) is `merged[j] |= bit if v[(j*base)/L]`,
     * i.e. each native slot is stretched uniformly over L/base output slots — NOT
     * tile-repeated. Canonicalization then rotates to the first rising edge of the
     * lowest active pin (Compiler.cpp:146-169) to dedup phase-shifted duplicates.
     *
     * Returns ok=false (and an empty table) when the group fails to compile (no
     * enabled wheel, zero-length expansion, or LCM over the 4096 cap).
     */
    fun compiledStats(state: BuilderState): CompiledStats {
        val is720 = state.hasCam()
        val degrees = if (is720) 720 else 360

        // Expand every enabled wheel once.
        data class Wheel(val ch: Int, val vec: IntArray)
        val wheels = ArrayList<Wheel>()
        for (ch in 0 until DslLimits.N_CHANNELS) {
            if (!state.isEnabled(ch)) continue
            val vec = expandWheel(ch, state, is720)
            if (vec.isEmpty()) {
                return CompiledStats(false, ByteArray(0), 0, degrees, 0, "wheel expanded to zero slots")
            }
            if (vec.size > DslLimits.SLOT_MAX) {
                return CompiledStats(false, ByteArray(0), 0, degrees, 0, "per-wheel slot count exceeds ${DslLimits.SLOT_MAX}")
            }
            wheels.add(Wheel(ch, vec))
        }
        if (wheels.isEmpty()) {
            return CompiledStats(false, ByteArray(0), 0, degrees, 0, "no enabled channels")
        }

        // Step 4: LCM merge length.
        var l = wheels[0].vec.size
        for (i in 1 until wheels.size) {
            l = lcm(l, wheels[i].vec.size)
            if (l == 0 || l > DslLimits.SLOT_MAX) {
                return CompiledStats(false, ByteArray(0), 0, degrees, 0,
                    "LCM exceeds ${DslLimits.SLOT_MAX}-byte limit (rule #9)")
            }
        }

        // OR-combine with the uniform stretch: out[j] |= bit if v[(j*base)/L].
        val merged = IntArray(l)
        var channelMask = 0
        for (w in wheels) {
            val bit = 1 shl (BuilderState.pinOf(w.ch) - 1)
            channelMask = channelMask or bit
            val base = w.vec.size.toLong()
            for (j in 0 until l) {
                val idx = ((j.toLong() * base) / l).toInt()
                if (w.vec[idx] != 0) merged[j] = merged[j] or bit
            }
        }

        // Step 5: canonicalize — rotate to the first rising edge of the lowest pin.
        canonicalize(merged, channelMask)

        // Step 6: pack to a ByteArray.
        val table = ByteArray(l) { merged[it].toByte() }
        return CompiledStats(true, table, l, degrees, channelMask)
    }

    /**
     * Rotate `buf` in place so slot 0 is the first rising edge of the lowest active
     * pin (lowest set bit in `channelMask`). Port of Compiler.cpp canonicalizeBuffer
     * (lines 146-169). A rising edge is a slot whose bit is set while the previous
     * (modulo-wrap) slot's bit is clear.
     */
    private fun canonicalize(buf: IntArray, channelMask: Int) {
        if (buf.isEmpty() || channelMask == 0) return
        var lowestBit = 0
        for (b in 0 until 8) {
            if (channelMask and (1 shl b) != 0) { lowestBit = b; break }
        }
        val mask = 1 shl lowestBit
        val n = buf.size
        var rotateTo = 0
        for (i in 0 until n) {
            val prev = (i + n - 1) % n
            if (buf[i] and mask != 0 && buf[prev] and mask == 0) {
                rotateTo = i
                break
            }
        }
        if (rotateTo == 0) return
        val tmp = IntArray(n) { buf[(it + rotateTo) % n] }
        System.arraycopy(tmp, 0, buf, 0, n)
    }

    // ---------------------------------------------------------------------------
    // emitDsl — the ':'-joined DSL string (ui_lvgl cb_emit_dsl / cb_emit_wheel)
    // ---------------------------------------------------------------------------

    /** Emit ONE channel's compact DSL token (mirror cb_emit_wheel, ui_lvgl.cpp:2609). */
    fun emitWheel(ch: Int, spec: ChannelSpec): String {
        val pin = BuilderState.pinOf(ch)
        val rot = if (BuilderState.rotationOf(ch) == Rotation.CW) 'C' else 'c'
        return when (spec.kind) {
            WheelKind.SYM ->
                "$pin,$rot,S,${spec.dutyNum}/${spec.dutyDen},${spec.teeth}"
            WheelKind.MISS -> buildString {
                append("$pin,$rot,M,${spec.dutyNum}/${spec.dutyDen},${spec.teeth}")
                for (r in spec.runs) append(",${r.count}${r.suffix}")
            }
            WheelKind.ANG -> buildString {
                append("$pin,$rot,A")
                for (deg in spec.angular) append(",$deg")
            }
        }
    }

    /** Emit the full ':'-joined DSL for all enabled channels (mirror cb_emit_dsl). */
    fun emitDsl(state: BuilderState): String {
        val tokens = ArrayList<String>()
        for (ch in 0 until DslLimits.N_CHANNELS) {
            if (!state.isEnabled(ch)) continue
            tokens.add(emitWheel(ch, state.channels[ch]))
        }
        return tokens.joinToString(":")
    }
}
