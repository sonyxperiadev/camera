SDCLANG_FLAGS := -Ofast -ffp-contract=fast -mcpu=cortex-a53 -mfpu=crypto-neon-fp-armv8 -fno-fast-math
SDCLANG_LTO   := -flto
SDCLANG_LINK  := -fuse-ld=qcld

# Turn off LTO for libs which do not build with LTO.
ifeq ($(LOCAL_SDCLANG_LTO),)
SDCLANG_LTO :=
endif

# Note: LOCAL_SDCLANG_EXTRA_FLAGS can be set in the individual module's .mk
# file in order to override the default SDCLANG_FLAGS.

SDCLANG_CFLAGS   := $(SDCLANG_FLAGS) $(SDCLANG_LTO) $(SDCLANG_LINK)
SDCLANG_LDFLAGS  := $(SDCLANG_FLAGS) $(SDCLANG_LTO)

LOCAL_CFLAGS     += $(SDCLANG_CFLAGS)
LOCAL_CXX_FLAGS  += $(SDCLANG_CFLAGS)
LOCAL_LDFLAGS    += $(SDCLANG_LDFLAGS)
LOCAL_CFLAGS_32  += $(LOCAL_SDCLANG_EXTRA_FLAGS_32)
LOCAL_LDFLAGS_32 += $(LOCAL_SDCLANG_EXTRA_FLAGS_32)
