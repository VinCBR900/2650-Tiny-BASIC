# docs/Makefile v1.1 - 2026-09-20
# Builds the GitHub Pages browser simulator: assembles each Tiny BASIC
# variant to .hex, generates assets/variants.json (CHIN/COUT per variant,
# pulled from each .LST by gen-variants.sh), and builds the pipbug_wrap
# wasm/JS runtime. Run via GitHub Actions on every push, or locally with
# `make -C docs serve`.
#
# History:
#   v1.1 - added pBASIC2650/4kBASIC2650 variants; added variants.json
#          generation (gen-variants.sh) so CHIN/COUT are pulled from each
#          variant's .LST automatically instead of being hand-edited into
#          index.html after an offline grep
#   v1.0 - single-variant (uBASIC2650) hex + wasm build

EMCC ?= emcc
MAKE ?= make
ASM2650 := ../tools/asm2650

# ---------------------------------------------------------------------------
# BASIC variants offered by the browser simulator. Each variant's source
# lives at the repo root as ../<name>.asm; asm2650 writes its .hex here and
# its .LST sidecar next to the source (../<name>.LST), from which CHIN/COUT
# are pulled automatically (see gen-variants.sh) instead of being grepped
# and hand-edited into the page offline.
# ---------------------------------------------------------------------------
VARIANTS := uBASIC2650 pBASIC2650 4kBASIC2650

uBASIC2650_SRC  := ../uBASIC2650.asm
pBASIC2650_SRC  := ../pBASIC2650.asm
4kBASIC2650_SRC := ../4kBASIC2650.asm

HEX_OUTS      := $(patsubst %,assets/%.hex,$(VARIANTS))
LST_OUTS      := $(patsubst %,../%.LST,$(VARIANTS))
VARIANTS_JSON := assets/variants.json

PIPBUG_SRC := ../tools/pipbug_wrap.c
PIPBUG_DEPS := $(PIPBUG_SRC) ../tools/2650.c ../tools/aa.h ../tools/aa_strings.h
PIPBUG_JS := assets/pipbug_wrap.js
PIPBUG_WASM := assets/pipbug_wrap.wasm

EMCCFLAGS ?= -Wall -O2 -DGAMER
EMSCRIPTEN_FLAGS := \
	-s MODULARIZE=1 \
	-s EXPORT_NAME=createPipbugWrapModule \
	-s INVOKE_RUN=0 \
	-s EXIT_RUNTIME=0 \
	-s FORCE_FILESYSTEM=1 \
	-s EXPORTED_FUNCTIONS='["_main","_pipbug_run_chunk"]' \
	-s EXPORTED_RUNTIME_METHODS='["FS","callMain"]'

.PHONY: all clean hex wasm variants serve

all: $(HEX_OUTS) $(VARIANTS_JSON) $(PIPBUG_JS)

hex: $(HEX_OUTS)

variants: $(VARIANTS_JSON)

wasm: $(PIPBUG_JS)

$(ASM2650):
	$(MAKE) -C ../tools asm2650

# One assembler invocation produces both a variant's .hex (here) and its
# .LST (next to the source). Listed as a plain multi-target rule rather
# than GNU Make's `&:` grouped-target syntax so this still works under
# older Make (e.g. macOS's bundled 3.81); that only matters if this
# Makefile is ever run with `-j` in parallel, which it isn't today.
define VARIANT_RULE
assets/$(1).hex ../$(1).LST: $$($(1)_SRC) $(ASM2650)
	mkdir -p assets
	$(ASM2650) $$($(1)_SRC) assets/$(1).hex
endef
$(foreach v,$(VARIANTS),$(eval $(call VARIANT_RULE,$(v))))

# CHIN/COUT shift with every code change on any variant, so pull them
# fresh from each .LST on every build rather than hand-editing the page.
$(VARIANTS_JSON): $(LST_OUTS) gen-variants.sh
	mkdir -p assets
	sh gen-variants.sh $(VARIANTS) > $(VARIANTS_JSON)

$(PIPBUG_JS) $(PIPBUG_WASM): $(PIPBUG_DEPS)
	mkdir -p assets
	$(EMCC) $(EMCCFLAGS) $(EMSCRIPTEN_FLAGS) -o $(PIPBUG_JS) $(PIPBUG_SRC)

serve: all
	python3 -m http.server 8000

clean:
	rm -f $(HEX_OUTS) $(LST_OUTS) $(VARIANTS_JSON) $(PIPBUG_JS) $(PIPBUG_WASM)
