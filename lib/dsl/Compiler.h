// Compiler.h — DSL → PatternRef pipeline (M5.3).
//
// The compiler runs the full pipeline:
//
//   source ──Lexer──> tokens ──Parser──> AST ──Validator──> AST
//                                              ──Compiler──> byte table
//
// On success the returned DslResult.pattern owns a PSRAM-allocated byte
// table (heap_caps_malloc(MALLOC_CAP_SPIRAM)). dslFree() releases it.
//
// This header re-declares the public API from Dsl.h; the implementation
// lives in Compiler.cpp.

#pragma once

#include "Dsl.h"

// Compiler-internal entry point that the public dslCompile() wraps. Exposed
// for unit tests so they can drive the compiler with an already-parsed AST.
struct ProgramAst;
DslResult dslCompileAst(const ProgramAst* ast);
