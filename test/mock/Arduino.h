#pragma once
// Mock Arduino.h for native testing
// Provides types, Stream base class, and stub functions

#include <cstdint>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <chrono>
#include <cmath>
#include <algorithm>

// ---- Time functions (mock using chrono) ----
inline uint64_t time_us_64() {
    static auto start = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(now - start).count());
}

inline void delayMicroseconds(uint32_t us) {
    (void)us; // no-op in test
}

inline unsigned long millis() {
    return static_cast<unsigned long>(time_us_64() / 1000ULL);
}

// ---- Arduino utility functions ----
inline int constrain(int val, int min_val, int max_val) {
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}

inline long constrain(long val, long min_val, long max_val) {
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}

inline float constrain(float val, float min_val, float max_val) {
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}

// ---- GPIO stubs ----
inline void gpio_init(uint) {}
inline void gpio_set_dir(uint, bool) {}
inline void gpio_put(uint, bool) {}
inline void gpio_pull_up(uint) {}
inline bool gpio_get(uint) { return false; }
inline void gpio_set_function(uint, int) {}
constexpr int GPIO_FUNC_PWM = 0;
constexpr int GPIO_FUNC_I2C = 0;
constexpr int GPIO_FUNC_UART = 0;
constexpr int GPIO_FUNC_PIO0 = 0;
constexpr int GPIO_OUT = 1;
constexpr int GPIO_IN = 0;
constexpr int GPIO_IRQ_EDGE_RISE = 1;
constexpr int GPIO_IRQ_EDGE_FALL = 2;
inline void gpio_set_irq_enabled(uint, uint32_t, bool) {}
inline void gpio_acknowledge_irq(uint, uint32_t) {}
inline uint32_t gpio_get_irq_event_mask(uint) { return 0; }

// ---- PWM stubs ----
typedef struct { } pwm_config;
inline pwm_config pwm_get_default_config() { return {}; }
inline void pwm_config_set_clkdiv(pwm_config*, float) {}
inline void pwm_config_set_wrap(pwm_config*, uint16_t) {}
inline void pwm_init(uint, const pwm_config*, bool) {}
inline uint pwm_gpio_to_slice_num(uint) { return 0; }
inline void pwm_set_clkdiv(uint, float) {}
inline void pwm_set_wrap(uint, uint16_t) {}
inline void pwm_set_gpio_level(uint, uint16_t) {}
inline void pwm_set_enabled(uint, bool) {}

// ---- I2C stubs ----
struct i2c_inst_t { };
inline i2c_inst_t* i2c0 = nullptr;
inline i2c_inst_t* i2c1 = nullptr;
inline void i2c_init(i2c_inst_t*, uint32_t) {}
inline int i2c_write_blocking(i2c_inst_t*, uint8_t, const uint8_t*, size_t, bool) { return 0; }
inline int i2c_read_blocking(i2c_inst_t*, uint8_t, uint8_t*, size_t, bool) { return 0; }

// ---- PIO stubs ----
struct pio_program_t { const uint16_t* instructions; uint8_t length; int origin; };
typedef struct { } PIO;
typedef struct { } pio_sm_config;
inline PIO pio0 = {};
inline PIO pio1 = {};
inline uint pio_add_program(PIO, const pio_program_t*) { return 0; }
inline pio_sm_config pio_get_default_sm_config() { return {}; }
inline void sm_config_set_wrap(pio_sm_config*, uint, uint) {}
inline void sm_config_set_in_pins(pio_sm_config*, uint) {}
inline void sm_config_set_jmp_pin(pio_sm_config*, uint) {}
inline void sm_config_set_in_pin_count(pio_sm_config*, uint) {}
inline void sm_config_set_clkdiv(pio_sm_config*, float) {}
inline void sm_config_set_in_shift(pio_sm_config*, bool, bool, uint) {}
inline void sm_config_set_out_shift(pio_sm_config*, bool, bool, uint) {}
inline pio_sm_config ultrasonic_echo_program_get_default_config(uint) { return {}; }
inline void pio_sm_init(PIO, uint, uint, const pio_sm_config*) {}
inline void pio_sm_set_enabled(PIO, uint, bool) {}
inline void pio_gpio_init(PIO, uint) {}
inline void pio_sm_claim(PIO, uint) {}
inline bool pio_sm_is_rx_fifo_empty(PIO, uint) { return true; }
inline uint32_t pio_sm_get(PIO, uint) { return 0; }
inline void pio_sm_put_blocking(PIO, uint, uint32_t) {}
inline void pio_sm_clear_fifos(PIO, uint) {}
inline void pio_sm_restart(PIO, uint) {}
inline void pio_sm_exec(PIO, uint, uint32_t) {}
inline uint32_t pio_encode_jmp(uint offset) { return offset; }

// ---- Timer stubs ----
inline void busy_wait_us(uint32_t) {}

// ---- IRQ stubs ----
inline void irq_set_exclusive_handler(uint, void(*)()) {}
inline void irq_set_enabled(uint, bool) {}
inline void uart_set_irq_enables(void*, bool, bool) {}
constexpr uint UART0_IRQ = 0;
constexpr uint UART1_IRQ = 1;
constexpr uint IO_IRQ_BANK0 = 0;

// ---- UART stubs ----
struct uart_inst_t { };
inline uart_inst_t* uart0 = nullptr;
inline uart_inst_t* uart1 = nullptr;
inline void uart_init(uart_inst_t*, uint32_t) {}
inline bool uart_is_readable(uart_inst_t*) { return false; }
inline uint8_t uart_getc(uart_inst_t*) { return 0; }
inline void uart_set_fifo_enabled(uart_inst_t*, bool) {}
inline void uart_set_format(uart_inst_t*, int, int, int) {}
constexpr int UART_PARITY_NONE = 0;

// ---- Multicore stubs ----
inline void multicore_reset_core1() {}

// ---- LED stub ----
constexpr uint LED_BUILTIN = 25;
inline void pinMode(uint, int) {}
inline void digitalWrite(uint, int) {}
inline bool digitalRead(uint) { return false; }

// ---- Stream base class (mock) ----
class Stream {
public:
    virtual ~Stream() = default;
    virtual int available() { return 0; }
    virtual int read() { return -1; }
    virtual size_t write(uint8_t) { return 1; }
    virtual size_t write(const uint8_t* buf, size_t len) {
        for (size_t i = 0; i < len; i++) write(buf[i]);
        return len;
    }
    virtual int availableForWrite() { return 256; }
    virtual void flush() {}
    int readBytes(char* buf, int len) {
        for (int i = 0; i < len; i++) {
            int c = read();
            if (c < 0) return i;
            buf[i] = static_cast<char>(c);
        }
        return len;
    }
};

// ---- Serial mock (for printf output) ----
class MockSerial : public Stream {
public:
    int available() override { return 0; }
    int read() override { return -1; }
    size_t write(uint8_t c) override {
        putchar(c);
        return 1;
    }
    size_t write(const uint8_t* buf, size_t len) override {
        for (size_t i = 0; i < len; i++) putchar(buf[i]);
        return len;
    }
    int availableForWrite() override { return 4096; }
    void flush() override { fflush(stdout); }

    // Arduino-compatible printf
    void printf(const char* fmt, ...) {
        va_list args;
        va_start(args, fmt);
        vprintf(fmt, args);
        va_end(args);
    }
};

static MockSerial Serial;

// ---- String mock ----
class String {
public:
    String() = default;
    String(const char* s) : data_(s) {}
    int length() const { return static_cast<int>(data_.size()); }
    void trim() {}
    const char* c_str() const { return data_.c_str(); }
    char operator[](int i) const { return data_[i]; }
private:
    std::string data_;
};

// ---- Faking/mock helper ----
// Disable Arduino functions that conflict with C++ standard library
#define bit(b) (1UL << (b))
#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLDOWN 2
