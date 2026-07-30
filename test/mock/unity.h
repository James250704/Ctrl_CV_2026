#pragma once
// Minimal Unity-compatible test macros for native testing
// Provides TEST_ASSERT_* macros that print pass/fail

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cstring>

static int _unity_tests = 0;
static int _unity_fails = 0;

#define UNITY_BEGIN() do { _unity_tests = 0; _unity_fails = 0; } while(0)
#define UNITY_END() ((_unity_fails == 0) ? 0 : 1)

#define RUN_TEST(func) do { \
    _unity_tests++; \
    int _fails_before = _unity_fails; \
    printf("  %-50s ", #func); \
    fflush(stdout); \
    func(); \
    if (_unity_fails > _fails_before) \
        printf("FAIL\n"); \
    else \
        printf("PASS\n"); \
} while(0)

// ---- Assertions ----
#define TEST_ASSERT_TRUE(cond) do { \
    if (!(cond)) { \
        printf("FAIL\n    Assert TRUE failed: %s (line %d)\n", #cond, __LINE__); \
        _unity_fails++; return; \
    } } while(0)

#define TEST_ASSERT_FALSE(cond) do { \
    if ((cond)) { \
        printf("FAIL\n    Assert FALSE failed: %s (line %d)\n", #cond, __LINE__); \
        _unity_fails++; return; \
    } } while(0)

#define TEST_ASSERT_EQUAL_HEX8(a, b) do { \
    if ((a) != (b)) { \
        printf("FAIL\n    Assert EQUAL_HEX8: 0x%02X != 0x%02X (line %d)\n", \
               (unsigned)(uint8_t)(a), (unsigned)(uint8_t)(b), __LINE__); \
        _unity_fails++; return; \
    } } while(0)

#define TEST_ASSERT_EQUAL_HEX16(a, b) do { \
    if ((a) != (b)) { \
        printf("FAIL\n    Assert EQUAL_HEX16: 0x%04X != 0x%04X (line %d)\n", \
               (unsigned)(uint16_t)(a), (unsigned)(uint16_t)(b), __LINE__); \
        _unity_fails++; return; \
    } } while(0)

#define TEST_ASSERT_EQUAL_HEX64(a, b) do { \
    if ((a) != (b)) { \
        printf("FAIL\n    Assert EQUAL_HEX64 (line %d)\n", __LINE__); \
        _unity_fails++; return; \
    } } while(0)

#define TEST_ASSERT_EQUAL_INT8(a, b) do { \
    if ((a) != (b)) { \
        printf("FAIL\n    Assert EQUAL_INT8: %d != %d (line %d)\n", \
               (int)(int8_t)(a), (int)(int8_t)(b), __LINE__); \
        _unity_fails++; return; \
    } } while(0)

#define TEST_ASSERT_EQUAL_UINT8(a, b) do { \
    if ((a) != (b)) { \
        printf("FAIL\n    Assert EQUAL_UINT8: 0x%02X != 0x%02X (line %d)\n", \
               (unsigned)(uint8_t)(a), (unsigned)(uint8_t)(b), __LINE__); \
        _unity_fails++; return; \
    } } while(0)

#define TEST_ASSERT_EQUAL_FLOAT(a, b) do { \
    if (std::fabs((double)(a) - (double)(b)) > 0.001) { \
        printf("FAIL\n    Assert EQUAL_FLOAT: %f != %f (line %d)\n", \
               (double)(a), (double)(b), __LINE__); \
        _unity_fails++; return; \
    } } while(0)

#define TEST_ASSERT_EQUAL_size_t(a, b) do { \
    if ((a) != (b)) { \
        printf("FAIL\n    Assert EQUAL_size_t: %zu != %zu (line %d)\n", \
               (size_t)(a), (size_t)(b), __LINE__); \
        _unity_fails++; return; \
    } } while(0)

#define TEST_ASSERT_NOT_EQUAL(a, b) do { \
    if ((a) == (b)) { \
        printf("FAIL\n    Assert NOT_EQUAL (line %d)\n", __LINE__); \
        _unity_fails++; return; \
    } } while(0)

#define TEST_ASSERT_TRUE_MESSAGE(cond, msg) TEST_ASSERT_TRUE(cond)

// uint8_t array comparison
inline void _unity_assert_uint8_array(const uint8_t* a, const uint8_t* b, size_t len, int line) {
    for (size_t i = 0; i < len; i++) {
        if (a[i] != b[i]) {
            printf("FAIL\n    Assert UINT8_ARRAY: [%zu] 0x%02X != 0x%02X (line %d)\n",
                   i, (unsigned)a[i], (unsigned)b[i], line);
            _unity_fails++;
            return;
        }
    }
}
#define TEST_ASSERT_EQUAL_UINT8_ARRAY(a, b, len) _unity_assert_uint8_array((const uint8_t*)(a), (const uint8_t*)(b), (size_t)(len), __LINE__)
