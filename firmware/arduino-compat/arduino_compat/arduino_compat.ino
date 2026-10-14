// Arduino-ESP32 compatibility suite for esp32sim.
//
// One sketch, built unchanged for every emulated chip, that runs a list of
// self-checking tests against the Arduino core and its bundled libraries and
// reports each result on Serial. tests/arduino_compat.py flashes it, plays the
// "outside world" (serial input, pins driven from outside), and checks the
// report. See README.md next to this folder.
//
// Report lines (all start with "@", so the harness can ignore boot noise):
//   @BEGIN <chip> <arduino version> <idf version>
//   @PASS <name>            a test passed
//   @FAIL <name> <detail>   a test failed
//   @ASK <what>             the sketch needs the harness; it waits for "OK\n"
//   @RESTART                the sketch is restarting itself on purpose
//   @END pass=<n> fail=<n>  all done

#include <Arduino.h>
#include <Preferences.h>
#include <LittleFS.h>
#include <Ticker.h>
#include <Wire.h>
#include <SPI.h>
#include <MD5Builder.h>
#include <esp_chip_info.h>
#include <esp_rom_gpio.h>
#include <soc/gpio_sig_map.h>
#include <esp_mac.h>
#include <esp_random.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <esp_idf_version.h>
#include <mbedtls/sha256.h>
#include <mbedtls/aes.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <atomic>
#include <functional>
#include <map>
#include <stdexcept>
#include <thread>
#include <vector>

// Pins that are plain GPIOs (no flash, USB, UART0 or strapping role) on every
// chip the suite targets. The harness uses the same numbers.
static const int PIN_OUT = 2;    // output, watched from outside
static const int PIN_IN = 3;     // input, driven from outside
static const int PIN_IRQ = 6;    // input with interrupt, driven from outside
static const int PIN_LOOP = 7;   // output and interrupt on the same pin
static const int PIN_PWM = 10;   // LEDC output
static const int PIN_UART_RX = 1, PIN_UART_TX = 0;  // Serial1, looped back on the chip
static const int PIN_SDA = 4, PIN_SCL = 5;           // Wire; the harness puts an EEPROM at 0x50
static const int PIN_SCK = 0, PIN_MOSI = 4, PIN_MISO = 5;  // SPI, MOSI looped back to MISO

static int g_pass = 0, g_fail = 0;
static String g_failure;

// Inside a test: record a failed check and leave the test.
#define CHECK(cond, ...)                                                   \
  do {                                                                     \
    if (!(cond)) {                                                         \
      char _b[160];                                                        \
      snprintf(_b, sizeof _b, __VA_ARGS__);                                \
      g_failure = String(#cond) + " (" + _b + ") line " + String(__LINE__); \
      return false;                                                        \
    }                                                                      \
  } while (0)

// Asks the harness to do something and waits for its "OK".
static bool ask(const char *what, uint32_t timeout_ms = 20000) {
  while (Serial.available()) Serial.read();
  Serial.printf("@ASK %s\n", what);
  Serial.setTimeout(timeout_ms);
  String r = Serial.readStringUntil('\n');
  r.trim();
  return r == "OK";
}

// Runs one test in its own task, so a driver that blocks forever on a
// peripheral the emulator lacks fails that test instead of the whole run.
struct TestCtx {
  bool (*fn)();
  bool result;
  SemaphoreHandle_t done;
};

static void test_task(void *arg) {
  TestCtx *t = static_cast<TestCtx *>(arg);
  t->result = t->fn();
  xSemaphoreGive(t->done);
  vTaskDelete(nullptr);
}

static void run(const char *name, bool (*fn)(), uint32_t timeout_ms = 10000) {
  TestCtx t{fn, false, xSemaphoreCreateBinary()};
  g_failure = "";
  TaskHandle_t h;
  xTaskCreate(test_task, name, 8192, &t, uxTaskPriorityGet(nullptr), &h);
  if (xSemaphoreTake(t.done, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
    vTaskDelete(h);
    t.result = false;
    g_failure = "timed out after " + String(timeout_ms) + " ms";
  }
  vSemaphoreDelete(t.done);
  if (t.result) {
    ++g_pass;
    Serial.printf("@PASS %s\n", name);
  } else {
    ++g_fail;
    Serial.printf("@FAIL %s %s\n", name, g_failure.c_str());
  }
  Serial.flush();
}

// ---------------------------------------------------------------- system

static bool t_chip_identity() {
  esp_chip_info_t info;
  esp_chip_info(&info);
  String want = CONFIG_IDF_TARGET;  // "esp32c3" -> "ESP32-C3"
  want.toUpperCase();
  if (want.length() > 5) want = want.substring(0, 5) + "-" + want.substring(5);
  CHECK(String(ESP.getChipModel()) == want, "model %s", ESP.getChipModel());
  CHECK(info.cores >= 1, "cores %d", info.cores);
  CHECK(ESP.getCpuFreqMHz() == 160, "cpu %u MHz", (unsigned)ESP.getCpuFreqMHz());
  CHECK(getXtalFrequencyMhz() == 40, "xtal %u", (unsigned)getXtalFrequencyMhz());
  CHECK(ESP.getFlashChipSize() >= 2 * 1024 * 1024, "flash %u", (unsigned)ESP.getFlashChipSize());
  uint64_t mac = ESP.getEfuseMac();
  CHECK(mac != 0 && mac != 0xFFFFFFFFFFFFull, "efuse mac");
  uint8_t m[6] = {};
  CHECK(esp_read_mac(m, ESP_MAC_BASE) == ESP_OK, "esp_read_mac");
  CHECK(m[0] || m[1] || m[2], "mac %02x:%02x:%02x", m[0], m[1], m[2]);
  CHECK(strlen(ESP.getSdkVersion()) > 0, "sdk version");
  return true;
}

static bool t_heap() {
  size_t free0 = ESP.getFreeHeap();
  CHECK(free0 > 100 * 1024, "free heap %u", (unsigned)free0);
  CHECK(ESP.getHeapSize() > free0, "heap size");
  void *p = malloc(64 * 1024);
  CHECK(p != nullptr, "malloc 64K");
  memset(p, 0xA5, 64 * 1024);
  CHECK(ESP.getFreeHeap() < free0 - 60 * 1024, "heap did not shrink");
  free(p);
  uint8_t *a = new uint8_t[1000];
  delete[] a;
  CHECK(ESP.getMaxAllocHeap() > 32 * 1024, "max alloc %u", (unsigned)ESP.getMaxAllocHeap());
  CHECK(heap_caps_check_integrity_all(true), "heap integrity");
  return true;
}

static bool t_cpp_runtime() {
  // Exceptions, RTTI-free polymorphism, STL containers, std::function.
  bool caught = false;
  try {
    throw std::runtime_error("boom");
  } catch (const std::exception &e) {
    caught = String(e.what()) == "boom";
  }
  CHECK(caught, "exception not caught");
  std::map<String, int> m{{"a", 1}, {"b", 2}};
  std::vector<int> v;
  for (int i = 0; i < 1000; ++i) v.push_back(i * i);
  long sum = 0;
  for (int x : v) sum += x;
  CHECK(sum == 332833500L, "sum %ld", sum);
  std::function<int(int)> f = [&](int x) { return x + m["b"]; };
  CHECK(f(40) == 42, "lambda");
  static int ctor_runs = 0;
  struct S { S() { ++ctor_runs; } };
  static S s1;
  CHECK(ctor_runs == 1, "static ctor");
  return true;
}

static bool t_string_and_math() {
  String s = "Hello";
  s += ' ';
  s += 42;
  s += F(" world ");
  s += 3.25f;
  CHECK(s == "Hello 42 world 3.25", "%s", s.c_str());
  CHECK(s.indexOf("world") == 9, "indexOf");
  CHECK(s.substring(0, 5).equalsIgnoreCase("HELLO"), "substring");
  CHECK(String("12345").toInt() == 12345, "toInt");
  CHECK(fabs(String("2.5").toFloat() - 2.5f) < 1e-6f, "toFloat");
  char buf[64];
  snprintf(buf, sizeof buf, "%.3f %lld %x %s", 3.14159, (long long)-1234567890123LL, 0xBEEF, "ok");
  CHECK(strcmp(buf, "3.142 -1234567890123 beef ok") == 0, "%s", buf);
  volatile double d = 2.0;
  CHECK(fabs(sqrt(d) - 1.41421356237) < 1e-9, "sqrt");
  CHECK(fabs(sin(M_PI / 6) - 0.5) < 1e-9, "sin");
  volatile float f = 10.0f;
  CHECK(fabsf(logf(f) - 2.302585f) < 1e-5f, "logf");
  volatile uint64_t a = 0xFFFFFFFFFFFFull, b = 12345;
  CHECK(a / b == 22800727153ull && a % b == 6870, "64-bit divide");
  CHECK(map(512, 0, 1023, 0, 255) == 127, "map");
  CHECK(constrain(300, 0, 255) == 255, "constrain");
  CHECK(bitRead(0x10, 4) == 1 && highByte(0x1234) == 0x12, "bit macros");
  return true;
}

static bool t_random() {
  randomSeed(1234);
  long a1 = random(1000000), a2 = random(1000000);
  randomSeed(1234);
  CHECK(random(1000000) == a1 && random(1000000) == a2, "randomSeed not repeatable");
  for (int i = 0; i < 200; ++i) {
    long r = random(10, 20);
    CHECK(r >= 10 && r < 20, "random(10,20) = %ld", r);
  }
  uint32_t r1 = esp_random(), r2 = esp_random(), r3 = esp_random();
  CHECK(!(r1 == r2 && r2 == r3), "esp_random constant %08lx", (unsigned long)r1);
  uint8_t buf[32] = {};
  esp_fill_random(buf, sizeof buf);
  int nz = 0;
  for (uint8_t x : buf) nz += x != 0;
  CHECK(nz > 16, "esp_fill_random");
  return true;
}

// ---------------------------------------------------------------- time

static bool t_millis_micros() {
  uint32_t m0 = millis(), u0 = micros();
  int64_t e0 = esp_timer_get_time();
  delay(200);
  uint32_t m1 = millis(), u1 = micros();
  int64_t e1 = esp_timer_get_time();
  CHECK(m1 - m0 >= 199 && m1 - m0 <= 202, "delay(200) took %lu ms", (unsigned long)(m1 - m0));
  CHECK(u1 - u0 >= 199000 && u1 - u0 <= 202000, "delay(200) took %lu us", (unsigned long)(u1 - u0));
  CHECK(e1 - e0 >= 199000 && e1 - e0 <= 202000, "esp_timer %lld", (long long)(e1 - e0));
  u0 = micros();
  delayMicroseconds(1500);
  u1 = micros();
  CHECK(u1 - u0 >= 1500 && u1 - u0 <= 1600, "delayMicroseconds(1500) took %lu", (unsigned long)(u1 - u0));
  // micros() must never step backwards.
  uint32_t last = micros();
  for (int i = 0; i < 20000; ++i) {
    uint32_t now = micros();
    CHECK(now - last < 1000000, "micros went backwards %lu -> %lu", (unsigned long)last, (unsigned long)now);
    last = now;
  }
  return true;
}

static bool t_cpu_frequency() {
  CHECK(setCpuFrequencyMhz(80), "set 80 MHz");
  CHECK(getCpuFrequencyMhz() == 80, "now %u", (unsigned)getCpuFrequencyMhz());
  uint32_t m0 = millis();
  delay(100);
  uint32_t dt = millis() - m0;
  // Busy work takes twice the cycles' worth of time at half the clock.
  uint32_t c0 = ESP.getCycleCount();
  delayMicroseconds(1000);
  uint32_t cyc80 = ESP.getCycleCount() - c0;
  CHECK(setCpuFrequencyMhz(160), "set 160 MHz");
  CHECK(getCpuFrequencyMhz() == 160, "back to %u", (unsigned)getCpuFrequencyMhz());
  c0 = ESP.getCycleCount();
  delayMicroseconds(1000);
  uint32_t cyc160 = ESP.getCycleCount() - c0;
  CHECK(dt >= 99 && dt <= 102, "delay(100) at 80 MHz took %lu", (unsigned long)dt);
  CHECK(cyc80 > 70000 && cyc80 < 90000, "1 ms at 80 MHz = %lu cycles", (unsigned long)cyc80);
  CHECK(cyc160 > 150000 && cyc160 < 170000, "1 ms at 160 MHz = %lu cycles", (unsigned long)cyc160);
  return true;
}

static volatile int g_ticks = 0;
static void on_tick() { ++g_ticks; }

static bool t_ticker() {
  Ticker t;
  g_ticks = 0;
  t.attach_ms(10, on_tick);
  delay(205);
  t.detach();
  int n = g_ticks;
  CHECK(n >= 19 && n <= 21, "%d ticks in 205 ms", n);
  delay(50);
  CHECK(g_ticks == n, "ticker still running after detach");
  Ticker once;
  g_ticks = 0;
  once.once_ms(30, on_tick);
  delay(60);
  CHECK(g_ticks == 1, "once_ms fired %d times", g_ticks);
  return true;
}

static volatile uint32_t g_hw_ticks = 0;
static void ARDUINO_ISR_ATTR on_hw_timer() { ++g_hw_ticks; }

static bool t_hw_timer() {
  hw_timer_t *tm = timerBegin(1000000);  // 1 MHz
  CHECK(tm != nullptr, "timerBegin");
  timerAttachInterrupt(tm, on_hw_timer);
  g_hw_ticks = 0;
  timerAlarm(tm, 1000, true, 0);  // every 1 ms
  delay(100);
  timerStop(tm);
  uint32_t n = g_hw_ticks;
  uint64_t t = timerRead(tm);
  timerEnd(tm);
  CHECK(n >= 98 && n <= 102, "%lu alarms in 100 ms", (unsigned long)n);
  CHECK(t < 1000, "counter %llu after auto-reload", (unsigned long long)t);
  return true;
}

// ---------------------------------------------------------------- FreeRTOS

static QueueHandle_t g_q;
static SemaphoreHandle_t g_mutex;
static volatile int g_shared = 0;

static void producer(void *) {
  for (int i = 1; i <= 50; ++i) {
    xQueueSend(g_q, &i, portMAX_DELAY);
    if (i % 10 == 0) vTaskDelay(1);
  }
  vTaskDelete(nullptr);
}

static void incrementer(void *arg) {
  for (int i = 0; i < 1000; ++i) {
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    int v = g_shared;
    if ((i & 63) == 0) taskYIELD();
    g_shared = v + 1;
    xSemaphoreGive(g_mutex);
  }
  xTaskNotifyGive(static_cast<TaskHandle_t>(arg));
  vTaskDelete(nullptr);
}

static bool t_freertos() {
  CHECK(configTICK_RATE_HZ == 1000, "tick rate %d", configTICK_RATE_HZ);
  TickType_t t0 = xTaskGetTickCount();
  vTaskDelay(pdMS_TO_TICKS(50));
  TickType_t dt = xTaskGetTickCount() - t0;
  CHECK(dt >= 50 && dt <= 51, "vTaskDelay(50) = %lu ticks", (unsigned long)dt);

  g_q = xQueueCreate(4, sizeof(int));
  xTaskCreate(producer, "prod", 2048, nullptr, 2, nullptr);
  int sum = 0, v;
  for (int i = 0; i < 50; ++i) {
    CHECK(xQueueReceive(g_q, &v, pdMS_TO_TICKS(1000)) == pdTRUE, "queue receive %d", i);
    sum += v;
  }
  vQueueDelete(g_q);
  CHECK(sum == 1275, "queue sum %d", sum);

  g_mutex = xSemaphoreCreateMutex();
  g_shared = 0;
  TaskHandle_t me = xTaskGetCurrentTaskHandle();
  xTaskCreate(incrementer, "inc1", 2048, me, 1, nullptr);
  xTaskCreate(incrementer, "inc2", 2048, me, 1, nullptr);
  for (int i = 0; i < 2; ++i)
    CHECK(ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5000)) == 1, "worker %d did not finish", i);
  vSemaphoreDelete(g_mutex);
  CHECK(g_shared == 2000, "mutex count %d", g_shared);
  return true;
}

static bool t_std_thread() {
  std::atomic<int> n{0};
  std::vector<std::thread> ts;
  for (int i = 0; i < 3; ++i)
    ts.emplace_back([&n] {
      for (int k = 0; k < 100; ++k) n++;
    });
  for (auto &t : ts) t.join();
  CHECK(n == 300, "n=%d", n.load());
  return true;
}

// ---------------------------------------------------------------- GPIO

static volatile int g_irq_count = 0;
static void ARDUINO_ISR_ATTR on_pin_irq() { ++g_irq_count; }

static bool t_gpio_output() {
  pinMode(PIN_OUT, OUTPUT);
  digitalWrite(PIN_OUT, HIGH);
  CHECK(digitalRead(PIN_OUT) == HIGH, "readback high");
  CHECK(ask("PIN 2 1"), "harness did not see GPIO2 high");
  digitalWrite(PIN_OUT, LOW);
  CHECK(digitalRead(PIN_OUT) == LOW, "readback low");
  CHECK(ask("PIN 2 0"), "harness did not see GPIO2 low");
  return true;
}

static bool t_gpio_input_pulls() {
  pinMode(PIN_IN, INPUT_PULLUP);
  delay(1);
  CHECK(digitalRead(PIN_IN) == HIGH, "pull-up reads low");
  pinMode(PIN_IN, INPUT_PULLDOWN);
  delay(1);
  CHECK(digitalRead(PIN_IN) == LOW, "pull-down reads high");
  CHECK(ask("DRIVE 3 1"), "drive");
  CHECK(digitalRead(PIN_IN) == HIGH, "driven high reads low");
  pinMode(PIN_IN, INPUT_PULLUP);
  CHECK(ask("DRIVE 3 0"), "drive");
  CHECK(digitalRead(PIN_IN) == LOW, "driven low reads high");
  CHECK(ask("DRIVE 3 z"), "release");
  CHECK(digitalRead(PIN_IN) == HIGH, "released: pull-up should win");
  return true;
}

static bool t_gpio_interrupt_external() {
  pinMode(PIN_IRQ, INPUT_PULLDOWN);
  g_irq_count = 0;
  attachInterrupt(digitalPinToInterrupt(PIN_IRQ), on_pin_irq, RISING);
  CHECK(ask("DRIVE 6 1"), "drive");
  delay(2);
  CHECK(g_irq_count == 1, "rising edge: %d irqs", g_irq_count);
  CHECK(ask("DRIVE 6 0"), "drive");
  delay(2);
  CHECK(g_irq_count == 1, "falling edge counted on RISING: %d", g_irq_count);
  detachInterrupt(digitalPinToInterrupt(PIN_IRQ));
  CHECK(ask("DRIVE 6 1"), "drive");
  delay(2);
  CHECK(g_irq_count == 1, "irq after detach: %d", g_irq_count);
  CHECK(ask("DRIVE 6 z"), "release");
  return true;
}

static bool t_gpio_interrupt_loopback() {
  // The pad's input path stays on in OUTPUT mode, so writing the pin
  // triggers its own CHANGE interrupt, as on real silicon.
  pinMode(PIN_LOOP, OUTPUT);
  digitalWrite(PIN_LOOP, LOW);
  g_irq_count = 0;
  attachInterrupt(PIN_LOOP, on_pin_irq, CHANGE);
  for (int i = 0; i < 10; ++i) {
    digitalWrite(PIN_LOOP, HIGH);
    delayMicroseconds(50);
    digitalWrite(PIN_LOOP, LOW);
    delayMicroseconds(50);
  }
  delay(1);
  detachInterrupt(PIN_LOOP);
  CHECK(g_irq_count == 20, "%d edges, want 20", g_irq_count);
  return true;
}

// ---------------------------------------------------------------- serial

static bool t_serial_echo() {
  // UART0 receive path: the harness answers with a line we check.
  while (Serial.available()) Serial.read();
  Serial.println("@ASK ECHO");
  Serial.setTimeout(20000);
  String line = Serial.readStringUntil('\n');
  line.trim();
  CHECK(line == "the quick brown fox 0123456789", "got '%s'", line.c_str());
  // A burst larger than the 128-byte hardware FIFO.
  Serial.println("@ASK BURST");
  size_t n = 0;
  uint32_t sum = 0;
  uint32_t t0 = millis();
  while (n < 1000 && millis() - t0 < 20000) {
    if (Serial.available()) {
      sum += (uint8_t)Serial.read();
      ++n;
    } else {
      delay(1);
    }
  }
  CHECK(n == 1000, "burst got %u bytes", (unsigned)n);
  uint32_t want = 0;
  for (int i = 0; i < 1000; ++i) want += 'A' + i % 26;
  CHECK(sum == want, "burst checksum %lu want %lu", (unsigned long)sum, (unsigned long)want);
  while (Serial.available()) Serial.read();  // trailing newline
  return true;
}

static bool t_serial1_loopback() {
  // Serial1 (UART1) with its TX signal also routed to the RX pad through the
  // GPIO matrix (uart_internal_loopback): what it sends comes straight back.
  Serial1.begin(115200, SERIAL_8N1, PIN_UART_RX, PIN_UART_TX);
  uart_internal_loopback(1, PIN_UART_RX);
  delay(5);
  while (Serial1.available()) Serial1.read();
  const char *msg = "uart1 loopback 0123456789";
  Serial1.print(msg);
  Serial1.flush();
  delay(10);
  String got;
  while (Serial1.available()) got += (char)Serial1.read();
  Serial1.end();
  CHECK(got == msg, "got '%s'", got.c_str());
  return true;
}

// ---------------------------------------------------------------- storage

static bool t_preferences() {
  Preferences p;
  CHECK(p.begin("compat", false), "begin");
  p.clear();
  CHECK(p.putInt("int", -12345) == 4, "putInt");
  CHECK(p.putString("str", "hello nvs") > 0, "putString");
  uint8_t blob[100];
  for (int i = 0; i < 100; ++i) blob[i] = i * 7;
  CHECK(p.putBytes("blob", blob, sizeof blob) == sizeof blob, "putBytes");
  CHECK(p.getInt("int", 0) == -12345, "getInt");
  CHECK(p.getString("str", "") == "hello nvs", "getString");
  uint8_t back[100] = {};
  CHECK(p.getBytes("blob", back, sizeof back) == sizeof back && !memcmp(blob, back, sizeof blob), "getBytes");
  CHECK(p.isKey("int") && !p.isKey("nope"), "isKey");
  CHECK(p.remove("int") && !p.isKey("int"), "remove");
  p.end();
  return true;
}

static bool t_littlefs() {
  CHECK(LittleFS.begin(true), "mount/format");
  File f = LittleFS.open("/compat.txt", "w");
  CHECK(f, "open for write");
  for (int i = 0; i < 200; ++i) f.printf("line %d\n", i);
  f.close();
  f = LittleFS.open("/compat.txt", "r");
  CHECK(f, "open for read");
  size_t size = f.size();
  String last;
  int lines = 0;
  while (f.available()) {
    last = f.readStringUntil('\n');
    ++lines;
  }
  f.close();
  CHECK(lines == 200 && last == "line 199", "%d lines, last '%s'", lines, last.c_str());
  CHECK(size == 1690, "size %u", (unsigned)size);
  CHECK(LittleFS.exists("/compat.txt"), "exists");
  CHECK(LittleFS.totalBytes() > 0 && LittleFS.usedBytes() > 0, "stats");
  LittleFS.end();
  return true;
}

static bool t_flash_raw() {
  // esp_flash / partition API straight onto the SPI flash model.
  const esp_partition_t *part =
      esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_COREDUMP, nullptr);
  CHECK(part != nullptr, "no coredump partition");
  CHECK(esp_partition_erase_range(part, 0, 4096) == ESP_OK, "erase");
  uint8_t buf[256], back[256];
  for (int i = 0; i < 256; ++i) buf[i] = 255 - i;
  CHECK(esp_partition_write(part, 0, buf, sizeof buf) == ESP_OK, "write");
  CHECK(esp_partition_read(part, 0, back, sizeof back) == ESP_OK, "read");
  CHECK(!memcmp(buf, back, sizeof buf), "readback differs");
  CHECK(esp_partition_erase_range(part, 0, 4096) == ESP_OK, "erase");
  CHECK(esp_partition_read(part, 0, back, 4) == ESP_OK && back[0] == 0xFF && back[3] == 0xFF, "erased");
  return true;
}

// ---------------------------------------------------------------- crypto

static bool t_crypto() {
  // SHA-256 runs on the SHA accelerator.
  uint8_t out[32];
  const char *abc = "abc";
  CHECK(mbedtls_sha256((const uint8_t *)abc, 3, out, 0) == 0, "sha256");
  static const uint8_t want[4] = {0xba, 0x78, 0x16, 0xbf};
  CHECK(!memcmp(out, want, 4), "sha256 %02x%02x%02x%02x", out[0], out[1], out[2], out[3]);
  // Long input: the SHA accelerator is fed through GDMA.
  static uint8_t big[5000];
  for (int i = 0; i < 5000; ++i) big[i] = i * 7;
  CHECK(mbedtls_sha256(big, sizeof big, out, 0) == 0, "sha256 5000");
  static const uint8_t want_big[4] = {0xb0, 0xab, 0xe1, 0xfc};
  CHECK(!memcmp(out, want_big, 4), "sha256(5000) %02x%02x%02x%02x", out[0], out[1], out[2], out[3]);
  MD5Builder md5;
  md5.begin();
  md5.add("abc");
  md5.calculate();
  CHECK(md5.toString() == "900150983cd24fb0d6963f7d28e17f72", "md5 %s", md5.toString().c_str());
  return true;
}

static bool t_aes() {
  // FIPS-197 C.3 AES-256 vector; the AES accelerator does the work.
  uint8_t key[32], pt[16], ct[16], back[16];
  for (int i = 0; i < 32; ++i) key[i] = i;
  for (int i = 0; i < 16; ++i) pt[i] = i * 0x11;
  static const uint8_t want[16] = {0x8e, 0xa2, 0xb7, 0xca, 0x51, 0x67, 0x45, 0xbf,
                                   0xea, 0xfc, 0x49, 0x90, 0x4b, 0x49, 0x60, 0x89};
  mbedtls_aes_context ctx;
  mbedtls_aes_init(&ctx);
  CHECK(mbedtls_aes_setkey_enc(&ctx, key, 256) == 0, "setkey");
  CHECK(mbedtls_aes_crypt_ecb(&ctx, MBEDTLS_AES_ENCRYPT, pt, ct) == 0, "encrypt");
  CHECK(!memcmp(ct, want, 16), "ciphertext %02x%02x..", ct[0], ct[1]);
  CHECK(mbedtls_aes_setkey_dec(&ctx, key, 256) == 0, "setkey dec");
  CHECK(mbedtls_aes_crypt_ecb(&ctx, MBEDTLS_AES_DECRYPT, ct, back) == 0, "decrypt");
  CHECK(!memcmp(back, pt, 16), "decrypt mismatch");

  // CBC over 4 KiB (GDMA, and the AES interrupt above 2000 bytes): the first
  // block must equal ECB(pt ^ iv), and decrypting must give the input back.
  static uint8_t data[4096], enc[4096], dec[4096];
  for (int i = 0; i < 4096; ++i) data[i] = i * 13 + 5;
  uint8_t iv[16], iv0[16], x[16], first[16];
  for (int i = 0; i < 16; ++i) iv0[i] = 0xF0 + i;
  for (int i = 0; i < 16; ++i) x[i] = data[i] ^ iv0[i];
  CHECK(mbedtls_aes_setkey_enc(&ctx, key, 256) == 0, "setkey");
  CHECK(mbedtls_aes_crypt_ecb(&ctx, MBEDTLS_AES_ENCRYPT, x, first) == 0, "ecb");
  memcpy(iv, iv0, 16);
  CHECK(mbedtls_aes_crypt_cbc(&ctx, MBEDTLS_AES_ENCRYPT, sizeof data, iv, data, enc) == 0, "cbc encrypt");
  CHECK(!memcmp(enc, first, 16), "cbc first block %02x%02x", enc[0], enc[1]);
  CHECK(!memcmp(iv, enc + 4080, 16), "cbc IV not chained");
  CHECK(mbedtls_aes_setkey_dec(&ctx, key, 256) == 0, "setkey dec");
  memcpy(iv, iv0, 16);
  CHECK(mbedtls_aes_crypt_cbc(&ctx, MBEDTLS_AES_DECRYPT, sizeof enc, iv, enc, dec) == 0, "cbc decrypt");
  CHECK(!memcmp(dec, data, sizeof data), "cbc roundtrip mismatch");

  // CTR, 100 bytes (not a whole number of blocks): keystream is ECB(counter).
  uint8_t nonce[16] = {}, stream[16], ks[16];
  size_t nc_off = 0;
  CHECK(mbedtls_aes_setkey_enc(&ctx, key, 128) == 0, "setkey 128");
  CHECK(mbedtls_aes_crypt_ctr(&ctx, 100, &nc_off, nonce, stream, data, enc) == 0, "ctr");
  uint8_t ctr[16] = {};
  ctr[15] = 6;  // block 6 holds bytes 96..99
  CHECK(mbedtls_aes_crypt_ecb(&ctx, MBEDTLS_AES_ENCRYPT, ctr, ks) == 0, "ecb ctr");
  CHECK((enc[97] ^ data[97]) == ks[1], "ctr keystream");
  CHECK(nonce[15] == 7, "ctr counter %u", nonce[15]);
  mbedtls_aes_free(&ctx);
  return true;
}

// ---------------------------------------------------------------- peripherals

static bool t_ledc_pwm() {
  CHECK(ledcAttach(PIN_PWM, 5000, 8), "ledcAttach");
  CHECK(ledcWrite(PIN_PWM, 128), "ledcWrite");
  CHECK(ledcRead(PIN_PWM) == 128, "ledcRead %lu", (unsigned long)ledcRead(PIN_PWM));
  CHECK(ledcReadFreq(PIN_PWM) == 5000, "freq %lu", (unsigned long)ledcReadFreq(PIN_PWM));
  analogWrite(PIN_PWM, 64);
  CHECK(ledcRead(PIN_PWM) == 64, "analogWrite duty %lu", (unsigned long)ledcRead(PIN_PWM));
  CHECK(ledcDetach(PIN_PWM), "ledcDetach");
  return true;
}

static bool t_i2c_eeprom() {
  // The harness starts the emulator with a 24C02-style EEPROM at 0x50
  // (--i2c-dev eeprom@0x50); nothing answers at 0x51.
  CHECK(Wire.begin(PIN_SDA, PIN_SCL, 100000), "Wire.begin");
  Wire.beginTransmission(0x51);
  uint8_t err = Wire.endTransmission();
  CHECK(err == 2, "probe of empty address = %u, want 2 (NACK)", err);
  Wire.beginTransmission(0x50);
  err = Wire.endTransmission();
  CHECK(err == 0, "probe of the EEPROM = %u", err);
  Wire.beginTransmission(0x50);
  Wire.write(0x10);  // memory address
  Wire.write((const uint8_t *)"hello i2c", 9);
  err = Wire.endTransmission();
  CHECK(err == 0, "write = %u", err);
  Wire.beginTransmission(0x50);
  Wire.write(0x10);
  err = Wire.endTransmission(false);  // repeated start
  CHECK(err == 0, "set address = %u", err);
  CHECK(Wire.requestFrom(0x50, 9) == 9, "requestFrom");
  char buf[10] = {};
  for (int i = 0; i < 9 && Wire.available(); ++i) buf[i] = Wire.read();
  CHECK(!strcmp(buf, "hello i2c"), "read back '%s'", buf);
  Wire.end();
  return true;
}

static bool t_spi_transfer() {
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, -1);
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  // Nothing on MISO yet: it reads all ones.
  uint8_t idle = SPI.transfer(0x00);
  // Route the MOSI signal to the MISO pad too: a loopback wire.
  esp_rom_gpio_connect_out_signal(PIN_MISO, FSPID_OUT_IDX, false, false);
  uint8_t r1 = SPI.transfer(0xA5);
  uint16_t r2 = SPI.transfer16(0x1234);
  uint32_t r3 = SPI.transfer32(0xDEADBEEF);
  uint8_t out[100], in[100];
  for (int i = 0; i < 100; ++i) out[i] = i * 3;
  SPI.transferBytes(out, in, sizeof out);
  SPI.endTransaction();
  SPI.end();
  CHECK(idle == 0xFF, "unconnected MISO read %02x", idle);
  CHECK(r1 == 0xA5 && r2 == 0x1234 && r3 == 0xDEADBEEF, "loopback got %02x %04x %08lx", r1, r2, (unsigned long)r3);
  CHECK(!memcmp(out, in, sizeof out), "transferBytes mismatch");
  return true;
}

static bool t_adc() {
  analogReadResolution(12);
  int v = analogRead(PIN_IN);
  CHECK(v >= 0 && v <= 4095, "analogRead %d", v);
  uint32_t mv = analogReadMilliVolts(PIN_IN);
  CHECK(mv <= 3400, "mV %lu", (unsigned long)mv);
  // The emulator reads a pad's logic level: 0 or full scale.
  CHECK(ask("DRIVE 3 1"), "drive");
  v = analogRead(PIN_IN);
  CHECK(v > 3900, "GPIO3 driven high reads %d", v);
  CHECK(ask("DRIVE 3 0"), "drive");
  v = analogRead(PIN_IN);
  CHECK(v < 200, "GPIO3 driven low reads %d", v);
  CHECK(ask("DRIVE 3 z"), "release");
  return true;
}

// ---------------------------------------------------------------- restart

static bool t_reset_reason_after_restart() {
  CHECK(esp_reset_reason() == ESP_RST_SW, "reset reason %d", (int)esp_reset_reason());
  Preferences p;
  p.begin("compat", true);
  CHECK(p.getString("persist", "") == "survives restart", "NVS value lost: '%s'", p.getString("persist", "").c_str());
  CHECK(p.getUInt("boots", 0) == 1, "boot count %u", (unsigned)p.getUInt("boots", 0));
  p.end();
  CHECK(LittleFS.begin(false), "LittleFS mount after restart");
  File f = LittleFS.open("/compat.txt", "r");
  CHECK(f && f.size() == 1690, "file lost across restart");
  f.close();
  LittleFS.end();
  return true;
}

// ---------------------------------------------------------------- main

void setup() {
  Serial.begin(115200);
  delay(50);
  Preferences p;
  p.begin("compat", false);
  uint32_t boots = p.getUInt("boots", 0);
  p.end();

  if (boots == 0) {
    Serial.printf("\n@BEGIN %s %d.%d.%d %s serial=%s\n", CONFIG_IDF_TARGET, ESP_ARDUINO_VERSION_MAJOR,
                  ESP_ARDUINO_VERSION_MINOR, ESP_ARDUINO_VERSION_PATCH, esp_get_idf_version(),
                  ARDUINO_USB_CDC_ON_BOOT ? "usb-cdc" : "uart0");
    if (esp_reset_reason() != ESP_RST_POWERON) Serial.printf("@NOTE reset reason %d\n", (int)esp_reset_reason());
    run("chip_identity", t_chip_identity);
    run("heap", t_heap);
    run("cpp_runtime", t_cpp_runtime);
    run("string_and_math", t_string_and_math);
    run("random", t_random);
    run("millis_micros", t_millis_micros);
    run("cpu_frequency", t_cpu_frequency);
    run("ticker", t_ticker);
    run("hw_timer", t_hw_timer);
    run("freertos", t_freertos);
    run("std_thread", t_std_thread);
    run("gpio_output", t_gpio_output, 60000);
    run("gpio_input_pulls", t_gpio_input_pulls, 60000);
    run("gpio_interrupt_external", t_gpio_interrupt_external, 60000);
    run("gpio_interrupt_loopback", t_gpio_interrupt_loopback);
    run("serial_echo", t_serial_echo, 60000);
    run("serial1_loopback", t_serial1_loopback);
    run("preferences", t_preferences);
    run("littlefs", t_littlefs, 60000);
    run("flash_raw", t_flash_raw);
    run("crypto", t_crypto);
    run("aes", t_aes);
    run("ledc_pwm", t_ledc_pwm);
    run("i2c_eeprom", t_i2c_eeprom);
    run("spi_transfer", t_spi_transfer);
    run("adc", t_adc);

    // Phase two runs after a software restart: NVS and LittleFS contents
    // must survive it, and the reset reason must say so.
    p.begin("compat", false);
    p.putString("persist", "survives restart");
    p.putUInt("boots", 1);
    p.putUInt("pass", g_pass);
    p.putUInt("fail", g_fail);
    p.end();
    Serial.println("@RESTART");
    Serial.flush();
    ESP.restart();
  }

  p.begin("compat", false);
  g_pass = p.getUInt("pass", 0);
  g_fail = p.getUInt("fail", 0);
  p.end();
  run("restart_persistence", t_reset_reason_after_restart);
  p.begin("compat", false);
  p.clear();  // the next power-on starts over
  p.end();
  Serial.printf("@END pass=%d fail=%d\n", g_pass, g_fail);
}

void loop() { delay(1000); }
