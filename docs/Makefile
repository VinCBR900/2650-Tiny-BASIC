EMCC ?= emcc
MAKE ?= make
ASM2650 := ../tools/asm2650
ASM_SRC := ../uBASIC2650.asm
HEX_OUT := assets/uBASIC2650.hex
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

.PHONY: all clean hex wasm serve

all: $(HEX_OUT) $(PIPBUG_JS)

hex: $(HEX_OUT)

wasm: $(PIPBUG_JS)

$(ASM2650):
	$(MAKE) -C ../tools asm2650

$(HEX_OUT): $(ASM_SRC) $(ASM2650)
	mkdir -p assets
	$(ASM2650) $(ASM_SRC) $@

$(PIPBUG_JS) $(PIPBUG_WASM): $(PIPBUG_DEPS)
	mkdir -p assets
	$(EMCC) $(EMCCFLAGS) $(EMSCRIPTEN_FLAGS) -o $(PIPBUG_JS) $(PIPBUG_SRC)

serve: all
	python3 -m http.server 8000

clean:
	rm -f $(HEX_OUT) $(PIPBUG_JS) $(PIPBUG_WASM)
