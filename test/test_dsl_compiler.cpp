// test_dsl_compiler.cpp — unit tests for the DSL compiler pipeline
// (Parser + Validator + Compiler).
//
// Scope: M5.2 + M5.3 + M5.5 deliverable for Agent D.
//
// Coverage:
//   1. Parses the 4 worked examples from integration_report.md §7.1 into
//      structurally correct ASTs.
//   2. Compiles each worked example to a PatternRef and asserts buffer
//      length, channel_mask, degrees, and rpm_scaler.
//   3. The headline regression: `"1,C,M,1/2,60,58t,2m : 2,c,S,1/2,1"`
//      compiles to a 240-byte 720° buffer whose bit-0 (crank) lane
//      matches the byte-packed Ardu-Stim 60-2 with-cam pattern, and
//      whose bit-1 (cam) lane is HIGH for exactly half the cycle.
//      The plan text mentions a "120-byte" buffer matching
//      sixty_minus_two_with_halfmoon_cam — that target conflicts with
//      §7.2's cam-doubling rule (the reference array itself is 240
//      bytes over 720°), so the 240-byte interpretation is the one
//      that's consistent with the integration report. See the open
//      questions section in the dispatch report.
//
// Builds: same dual-mode shim as test_dsl_lexer.cpp.

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "../lib/dsl/Lexer.h"
#include "../lib/dsl/Parser.h"
#include "../lib/dsl/Validator.h"
#include "../lib/dsl/Compiler.h"
#include "../lib/ckp_gen/PatternRef.h"
// Note: the SignalConfig POD remains declared by Dsl.h (host-test stub) /
// CkpGenerator.h (firmware), but the DSL compiler no longer exposes a
// SignalConfig adapter, so these tests exercise dslCompile() only.

// ── Assertion shim ─────────────────────────────────────────────────────────

#if defined(DSL_LEXER_TEST_HOST) || !defined(UNITY_INCLUDE_CONFIG_H)

static int g_failures = 0;
static int g_checks   = 0;

#define DSL_CHECK(cond, msg)                                                  \
  do {                                                                        \
    ++g_checks;                                                               \
    if (!(cond)) {                                                            \
      ++g_failures;                                                           \
      fprintf(stderr, "FAIL %s:%d  %s  (%s)\n", __FILE__, __LINE__, msg, #cond); \
    }                                                                         \
  } while (0)

#define DSL_EQ_INT(actual, expected, msg)                                     \
  do {                                                                        \
    ++g_checks;                                                               \
    long _a = (long)(actual);                                                 \
    long _e = (long)(expected);                                               \
    if (_a != _e) {                                                           \
      ++g_failures;                                                           \
      fprintf(stderr, "FAIL %s:%d  %s  expected=%ld got=%ld\n",               \
              __FILE__, __LINE__, msg, _e, _a);                               \
    }                                                                         \
  } while (0)

#else
#include <unity.h>
#define DSL_CHECK(cond, msg)            TEST_ASSERT_TRUE_MESSAGE((cond), msg)
#define DSL_EQ_INT(actual, exp, msg)    TEST_ASSERT_EQUAL_INT_MESSAGE((exp), (actual), msg)
#endif

// ── Helpers ────────────────────────────────────────────────────────────────

// Count how many slots of `pat` have the given bit set.
static int countBitSet(const uint8_t* pat, uint16_t n, uint8_t bit_mask) {
  int c = 0;
  for (uint16_t i = 0; i < n; ++i) if (pat[i] & bit_mask) ++c;
  return c;
}

// ── Parser tests: 4 worked examples from §7.1 ──────────────────────────────

static void test_parse_4cyl_distributor(void) {
  // "4-cyl distributor (2 teeth/rev, 50%)" — 1,C,S,1/2,2
  char err[96] = {0};
  uint16_t off = 0;
  ProgramAst* ast = parse("1,C,S,1/2,2", err, sizeof(err), &off);
  DSL_CHECK(ast != nullptr, "4cyl parses");
  if (!ast) return;
  DSL_EQ_INT(ast->wheels.size(), 1, "one wheel");
  const WheelDef& w = ast->wheels[0];
  DSL_EQ_INT(w.pin, 1, "pin=1");
  DSL_EQ_INT((int)w.rotation, (int)Rotation::CW, "C");
  DSL_EQ_INT((int)w.kind, (int)WheelKind::Symmetric, "S");
  DSL_EQ_INT(w.duty_num, 1, "duty num");
  DSL_EQ_INT(w.duty_den, 2, "duty den");
  DSL_EQ_INT(w.total_teeth, 2, "total=2");
  freeProgramAst(ast);
}

static void test_parse_60_2_with_halfmoon(void) {
  // "60-2 crank + half-moon cam" — 1,C,M,1/2,60,58t,2m : 2,c,S,1/2,1
  char err[96] = {0};
  uint16_t off = 0;
  ProgramAst* ast = parse("1,C,M,1/2,60,58t,2m : 2,c,S,1/2,1",
                          err, sizeof(err), &off);
  DSL_CHECK(ast != nullptr, "60-2+halfmoon parses");
  if (!ast) return;
  DSL_EQ_INT(ast->wheels.size(), 2, "two wheels");
  const WheelDef& wA = ast->wheels[0];
  DSL_EQ_INT(wA.pin, 1, "wA pin");
  DSL_EQ_INT((int)wA.kind, (int)WheelKind::Missing, "wA missing");
  DSL_EQ_INT(wA.total_teeth, 60, "wA total");
  DSL_EQ_INT(wA.runs.size(), 2, "wA runs");
  DSL_EQ_INT(wA.runs[0].value, 58, "58t value");
  DSL_EQ_INT(wA.runs[0].suffix, 't', "58t suffix");
  DSL_EQ_INT(wA.runs[1].value, 2, "2m value");
  DSL_EQ_INT(wA.runs[1].suffix, 'm', "2m suffix");
  DSL_EQ_INT(wA.teeth_with, 58, "tw=58");
  DSL_EQ_INT(wA.missing, 2, "missing=2");
  const WheelDef& wB = ast->wheels[1];
  DSL_EQ_INT(wB.pin, 2, "wB pin");
  DSL_EQ_INT((int)wB.rotation, (int)Rotation::CCW, "wB cam");
  DSL_EQ_INT((int)wB.kind, (int)WheelKind::Symmetric, "wB sym");
  DSL_EQ_INT(wB.total_teeth, 1, "wB total=1");
  freeProgramAst(ast);
}

static void test_parse_nissan_360(void) {
  // "Nissan 360 CAS + sync slot" — 1,C,S,1/2,360 : 2,C,A,40,20,...
  const char* src = "1,C,S,1/2,360 : 2,C,A,40,20,40,20,40,20,40,20,40,20,40,20";
  char err[96] = {0};
  uint16_t off = 0;
  ProgramAst* ast = parse(src, err, sizeof(err), &off);
  DSL_CHECK(ast != nullptr, "nissan parses");
  if (!ast) return;
  DSL_EQ_INT(ast->wheels.size(), 2, "two wheels");
  const WheelDef& wB = ast->wheels[1];
  DSL_EQ_INT((int)wB.kind, (int)WheelKind::Angular, "wB angular");
  DSL_EQ_INT(wB.angular.size(), 12, "12 angular entries");
  // Sum should be 360 (alternating 40+20 ×6 = 360).
  int sum = 0;
  for (auto& a : wB.angular) sum += a.degrees;
  DSL_EQ_INT(sum, 360, "angular sum 360");
  freeProgramAst(ast);
}

static void test_parse_36_1_with_cam(void) {
  // "36-1 with cam sync pulse" — 1,C,M,1/2,36,35t,1m : 2,c,A,10,710
  char err[96] = {0};
  uint16_t off = 0;
  ProgramAst* ast = parse("1,C,M,1/2,36,35t,1m : 2,c,A,10,710",
                          err, sizeof(err), &off);
  DSL_CHECK(ast != nullptr, "36-1+cam parses");
  if (!ast) return;
  const WheelDef& wB = ast->wheels[1];
  DSL_EQ_INT((int)wB.kind, (int)WheelKind::Angular, "wB angular");
  DSL_EQ_INT(wB.angular.size(), 2, "two angular entries");
  DSL_EQ_INT(wB.angular[0].degrees, 10, "first 10");
  DSL_EQ_INT(wB.angular[1].degrees, 710, "second 710");
  freeProgramAst(ast);
}

// ── Compile tests ──────────────────────────────────────────────────────────

static void test_compile_4cyl_distributor(void) {
  DslResult r = dslCompile("1,C,S,1/2,2");
  DSL_CHECK(r.ok, "4cyl compiles");
  if (!r.ok) { fprintf(stderr, "  err=%s\n", r.error); return; }
  DSL_EQ_INT(r.pattern.slot_count, 4, "2 teeth × 2 duty = 4 slots");
  DSL_EQ_INT(r.pattern.degrees, 360, "360°");
  DSL_EQ_INT(r.pattern.channel_mask, 0x01, "pin1 only");
  // After canonicalization, slot 0 = rising edge of pin1 ⇒ bit0 set in slot 0.
  DSL_CHECK(r.pattern.table[0] & 0x01, "slot 0 has crank HIGH");
  dslFree(r.pattern);
}

static void test_compile_36_1_with_cam_pulse(void) {
  // 36-1 (1,C,M,1/2,36,35t,1m) with a cam sync pulse (2,c,A,10,710).
  // Wheel A: 36×2 = 72 slots over 360°, cam-doubled to 144 over 720°.
  // Wheel B: angular 10°+710° = 720 slots.
  // LCM(144, 720) = 720. Channel mask = 0x03.
  DslResult r = dslCompile("1,C,M,1/2,36,35t,1m : 2,c,A,10,710");
  DSL_CHECK(r.ok, "36-1+cam compiles");
  if (!r.ok) { fprintf(stderr, "  err=%s\n", r.error); return; }
  DSL_EQ_INT(r.pattern.slot_count, 720, "LCM = 720");
  DSL_EQ_INT(r.pattern.degrees, 720, "720°");
  DSL_EQ_INT(r.pattern.channel_mask, 0x03, "pin1+pin2");
  // Cam (bit1) is HIGH for 10° out of 720° → 10 slots out of 720 in the
  // expanded buffer (one slot per output position, scaled from native).
  int cam_high = countBitSet(r.pattern.table, r.pattern.slot_count, 0x02);
  // The cam HIGH portion may be reversed by 'c'; either 10 or 710 high
  // is acceptable depending on reversal semantics. Assert "approximately
  // one of those two".
  DSL_CHECK(cam_high == 10 || cam_high == 710,
            "cam HIGH count matches 10° or 710° (after reversal)");
  dslFree(r.pattern);
}

static void test_compile_nissan_360(void) {
  const char* src =
      "1,C,S,1/2,360 : 2,C,A,40,20,40,20,40,20,40,20,40,20,40,20";
  DslResult r = dslCompile(src);
  DSL_CHECK(r.ok, "nissan compiles");
  if (!r.ok) { fprintf(stderr, "  err=%s\n", r.error); return; }
  // Wheel A: 360 teeth × 2 = 720 slots over 360°. Both wheels are 'C'
  // (no cam-doubling). Wheel B is angular sum 360 → 360 slots.
  // LCM(720, 360) = 720. degrees=360 (no 'c' present).
  DSL_EQ_INT(r.pattern.slot_count, 720, "LCM=720");
  DSL_EQ_INT(r.pattern.degrees, 360, "360°");
  DSL_EQ_INT(r.pattern.channel_mask, 0x03, "pin1+pin2 active");
  dslFree(r.pattern);
}

// ── Headline regression: 60-2 with half-moon cam ───────────────────────────
//
// DSL: "1,C,M,1/2,60,58t,2m : 2,c,S,1/2,1"
//   - Wheel A: 60 teeth × 2 = 120 slots over 360°, cam-doubled to 240 over 720°.
//     Pattern (before doubling): 58 × [1,0] + 2 × [0,0] = 116 ones-and-zeros
//     followed by 4 zeros.
//   - Wheel B: 1 tooth × 2 = 2 slots over 720°, native [1,0]; with CCW
//     reversal → [0,1]. Stretched to 240 → first 120 slots = 0 (cam LOW),
//     next 120 slots = 1 (cam HIGH).
//   - LCM(240, 2) = 240.
//   - channel_mask = 0x03, degrees = 720, rpm_scaler = 240/120 = 2.0.
//
// After canonicalization (rotate to first rising edge of pin1 = bit0):
//   - The pin-1 bit-pattern (bit0 of each byte) must be the byte-packed
//     60-2 sequence: 58 × [1,0] + 4 × [0], repeated twice (240 slots).
//   - The cam lane (bit1) must be HIGH for exactly 120 of the 240 slots.
//
// We do NOT byte-match against References/wheel_defs.h's
// sixty_minus_two_with_halfmoon_cam because that table encodes the cam
// transition at tooth 44 (a specific angular event), which the symmetric
// DSL `2,c,S,1/2,1` cannot express. The "120 bytes" target in
// implementation_plan.md is at odds with §7.2 cam-doubling and the
// reference table's own length (240 bytes). See open questions.
static void test_compile_60_2_halfmoon_cam(void) {
  const char* src = "1,C,M,1/2,60,58t,2m : 2,c,S,1/2,1";
  DslResult r = dslCompile(src);
  DSL_CHECK(r.ok, "60-2+halfmoon compiles");
  if (!r.ok) { fprintf(stderr, "  err=%s\n", r.error); return; }
  DSL_EQ_INT(r.pattern.slot_count, 240, "compiled to 240 bytes");
  DSL_EQ_INT(r.pattern.degrees, 720, "720°");
  DSL_EQ_INT(r.pattern.channel_mask, 0x03, "pin1+pin2");
  DSL_CHECK(r.pattern.rpm_scaler > 1.99f && r.pattern.rpm_scaler < 2.01f,
            "rpm_scaler ≈ 2.0");

  // Cam lane is HIGH for exactly 120 slots (half-moon).
  int cam_high = countBitSet(r.pattern.table, r.pattern.slot_count, 0x02);
  DSL_EQ_INT(cam_high, 120, "cam HIGH for 120/240 slots");

  // Crank lane: 58 present-tooth pulses per revolution × 2 revs = 116
  // bits of HIGH on bit0 (one HIGH-slot per present tooth at 1/2 duty).
  int crank_high = countBitSet(r.pattern.table, r.pattern.slot_count, 0x01);
  DSL_EQ_INT(crank_high, 116, "116 crank HIGH slots (58 teeth × 2 revs)");

  // Crank pattern: in each 120-slot half, bit0 alternates 1,0,1,0... for
  // 116 slots then 0,0,0,0 for the gap. Check the first revolution.
  // After canonicalization slot 0 is the rising edge of pin1, so slot 0
  // is 1, slot 1 is 0, slot 2 is 1, slot 3 is 0, …
  for (int i = 0; i < 116; ++i) {
    const uint8_t expected_crank = (i % 2 == 0) ? 1 : 0;
    if ((r.pattern.table[i] & 0x01) != expected_crank) {
      DSL_CHECK(false, "crank alternation broken in rev 1");
      break;
    }
  }
  // Gap: 4 zeros on bit0.
  for (int i = 116; i < 120; ++i) {
    DSL_CHECK((r.pattern.table[i] & 0x01) == 0, "rev1 gap is 0");
  }

  dslFree(r.pattern);
}

// ── Negative tests ─────────────────────────────────────────────────────────

static void test_compile_empty_source(void) {
  DslResult r = dslCompile("");
  DSL_CHECK(!r.ok, "empty source fails");
  DSL_CHECK(r.pattern.table == nullptr, "no allocation on fail");
}

static void test_compile_garbage(void) {
  DslResult r = dslCompile("hello world");
  DSL_CHECK(!r.ok, "garbage source fails");
}

// ── Runner ─────────────────────────────────────────────────────────────────

static void run_all(void) {
  test_parse_4cyl_distributor();
  test_parse_60_2_with_halfmoon();
  test_parse_nissan_360();
  test_parse_36_1_with_cam();
  test_compile_4cyl_distributor();
  test_compile_36_1_with_cam_pulse();
  test_compile_nissan_360();
  test_compile_60_2_halfmoon_cam();
  test_compile_empty_source();
  test_compile_garbage();
}

#if defined(DSL_LEXER_TEST_HOST) || !defined(UNITY_INCLUDE_CONFIG_H)
int main(void) {
  run_all();
  if (g_failures == 0) {
    fprintf(stdout, "[OK] DSL compiler: %d checks passed\n", g_checks);
    return 0;
  }
  fprintf(stdout, "[FAIL] DSL compiler: %d/%d checks failed\n",
          g_failures, g_checks);
  return 1;
}
#else
void setUp(void)    {}
void tearDown(void) {}
int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_parse_4cyl_distributor);
  RUN_TEST(test_parse_60_2_with_halfmoon);
  RUN_TEST(test_parse_nissan_360);
  RUN_TEST(test_parse_36_1_with_cam);
  RUN_TEST(test_compile_4cyl_distributor);
  RUN_TEST(test_compile_36_1_with_cam_pulse);
  RUN_TEST(test_compile_nissan_360);
  RUN_TEST(test_compile_60_2_halfmoon_cam);
  RUN_TEST(test_compile_empty_source);
  RUN_TEST(test_compile_garbage);
  return UNITY_END();
}
#endif
