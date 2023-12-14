CHECK_INCLUDE_FILE_CXX(lsxintrin.h HAVE_C_LOONGARCH64_LSXINTRIN_H)

if (HAVE_C_LOONGARCH64_LSXINTRIN_H)
    set (INTRIN_INC_H "lsxintrin.h")
else()
    message (FATAL_ERROR "No intrinsics header found for LSX")
endif ()

set(ARCH_C_FLAGS "-mlsx")
set(ARCH_CXX_FLAGS "-mlsx")

set(CMAKE_REQUIRED_FLAGS "${ARCH_C_FLAGS}")
CHECK_C_SOURCE_COMPILES("#include <${INTRIN_INC_H}>
int main() {
    __m128i a = __lsx_vreplgr2vr_w(1);
    (void)a;
}" HAVE_LSX)

if (NOT HAVE_LSX)
    message(FATAL_ERROR "LSX support required for LoongArch support")
endif ()
