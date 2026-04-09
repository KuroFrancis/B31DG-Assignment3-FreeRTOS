#include "driver/pcnt.h"
#include <inttypes.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#define SYNC_PIN    33
#define IN_A_PIN    32
#define IN_B_PIN    27
#define IN_S_PIN    25
#define IN_MODE_PIN 26

#define ACK_A_PIN   19
#define ACK_B_PIN   2
#define ACK_AGG_PIN 4
#define ACK_C_PIN   17
#define ACK_D_PIN   5
#define ACK_S_PIN   16
#define PIN_ERR     13

#define PCNT_UNIT_A PCNT_UNIT_0
#define PCNT_UNIT_B PCNT_UNIT_1

#define DEBOUNCE_DELAY_US 50000
#define MODE_DEBOUNCE_DELAY_US 200000
#define FINAL_REPORT_AFTER_SECONDS 10

static inline uint32_t get_ccount() {
  uint32_t ccount;
  asm volatile("rsr %0, ccount" : "=a"(ccount));
  return ccount;
}

#define COMPILER_BARRIER() asm volatile("" ::: "memory")

static inline uint32_t mix32(uint32_t x) {
  x ^= x >> 16;
  x *= 0x7FEB352Du;
  x ^= x >> 15;
  x *= 0x846CA68Bu;
  x ^= x >> 16;
  return x;
}

static uint32_t WorkKernel(uint32_t budget_cycles, uint32_t seed) {
  uint32_t start = get_ccount();
  uint32_t acc = 0x12345678u ^ seed;
  uint32_t x   = 0x9E3779B9u ^ (seed * 0x85EBCA6Bu);

  while ((uint32_t)(get_ccount() - start) < budget_cycles) {
    x   = mix32(x + 0x9E3779B9u);
    acc ^= x;
    acc = (acc << 5) | (acc >> 27);
    acc += 0xA5A5A5A5u;
    COMPILER_BARRIER();
  }
  return mix32(acc);
}

class TimingMonitor {
 public:
  void setPeriodicReportEverySeconds(uint32_t seconds) {
    periodic_report_every_us_ = (uint64_t)seconds * 1000000ull;
    next_periodic_report_us_ = (periodic_report_every_us_ > 0u) ? (t0_ + periodic_report_every_us_) : 0u;
  }
  
  void setFinalReportAfterSeconds(uint32_t seconds) {
    final_report_after_us_ = (uint64_t)seconds * 1000000ull;
    final_report_deadline_us_ = (final_report_after_us_ > 0u) ? (t0_ + final_report_after_us_) : 0u;
    final_report_printed_ = false;
  }

  void synch() {
    t0_ = micros();
    resetTask(a_);
    resetTask(b_);
    resetTask(agg_);
    resetTask(c_);
    resetTask(d_);
    resetTask(s_);
    s_q_head_ = 0;
    s_q_tail_ = 0;
    s_q_count_ = 0;
    next_periodic_report_us_ = (periodic_report_every_us_ > 0u) ? (t0_ + periodic_report_every_us_) : 0u;
    final_report_deadline_us_ = (final_report_after_us_ > 0u) ? (t0_ + final_report_after_us_) : 0u;
    final_report_printed_ = false;
  }

  void notifySRelease() {
    const uint64_t t = micros();
    if (s_q_count_ < S_RELEASE_Q_MAX) {
      s_release_us_[s_q_tail_] = t;
      s_q_tail_ = (s_q_tail_ + 1u) % S_RELEASE_Q_MAX;
      s_q_count_++;
    }
  }

  void beginTaskA(uint32_t id) { beginTask(a_, id, t0_ + (uint64_t)id * 10000ull); }
  void endTaskA() { endTask(a_); }
  void beginTaskB(uint32_t id) { beginTask(b_, id, t0_ + (uint64_t)id * 20000ull); }
  void endTaskB() { endTask(b_); }
  void beginTaskAGG(uint32_t id) { beginTask(agg_, id, t0_ + (uint64_t)id * 20000ull); }
  void endTaskAGG() { endTask(agg_); }
  void beginTaskC(uint32_t id) { beginTask(c_, id, t0_ + (uint64_t)id * 50000ull); }
  void endTaskC() { endTask(c_); }
  void beginTaskD(uint32_t id) { beginTask(d_, id, t0_ + (uint64_t)id * 50000ull); }
  void endTaskD() { endTask(d_); }
  void beginTaskS(uint32_t id) { beginTask(s_, id, popSRelease()); }
  void endTaskS() { endTask(s_); }

  bool allDeadlinesMet() const {
    return a_.misses == 0 && b_.misses == 0 && agg_.misses == 0 && 
           c_.misses == 0 && d_.misses == 0 && s_.misses == 0;
  }

  bool pollReports() {
    const uint64_t now = micros();
    if (periodic_report_every_us_ > 0u && now >= next_periodic_report_us_) {
      const uint64_t periods_missed =
          (now - next_periodic_report_us_) / periodic_report_every_us_;
      next_periodic_report_us_ += (periods_missed + 1u) * periodic_report_every_us_;
      report();
    }
    if (!final_report_printed_ && final_report_deadline_us_ > 0u &&
        now >= final_report_deadline_us_) {
      printFinalReport();
      final_report_printed_ = true;
      return true;
    }
    return false;
  }

  void report() const {
    Serial.printf("[MON] T0=%llu us\n", t0_);
    reportOne("A", a_);
    reportOne("B", b_);
    reportOne("AGG", agg_);
    reportOne("C", c_);
    reportOne("D", d_);
    reportOne("S", s_);
  }

  void printFinalReport() const {
    Serial.println("FINAL_REPORT_BEGIN");
    report();
    Serial.println("FINAL_REPORT_END");
  }

 private:
  struct TaskStats {
    uint32_t jobs = 0;
    uint32_t misses = 0;
    uint32_t id = 0;
    bool active = false;
    uint64_t start_us = 0;
    uint64_t release_us = 0;
    uint64_t max_exec_us = 0;
    int64_t worst_lateness_us = 0;
    uint64_t deadline_us = 0;
  };

  TaskStats a_{0, 0, 0, false, 0, 0, 0, 0, 10000};
  TaskStats b_{0, 0, 0, false, 0, 0, 0, 0, 20000};
  TaskStats agg_{0, 0, 0, false, 0, 0, 0, 0, 20000};
  TaskStats c_{0, 0, 0, false, 0, 0, 0, 0, 50000};
  TaskStats d_{0, 0, 0, false, 0, 0, 0, 0, 50000};
  TaskStats s_{0, 0, 0, false, 0, 0, 0, 0, 30000};
  uint64_t t0_ = 0;
  uint64_t periodic_report_every_us_ = 0u;
  uint64_t final_report_after_us_ = 0u;
  uint64_t next_periodic_report_us_ = 0u;
  uint64_t final_report_deadline_us_ = 0u;
  bool final_report_printed_ = false;
  static const uint32_t S_RELEASE_Q_MAX = 32u;
  uint64_t s_release_us_[S_RELEASE_Q_MAX]{};
  uint32_t s_q_head_ = 0;
  uint32_t s_q_tail_ = 0;
  uint32_t s_q_count_ = 0;

  static void resetTask(TaskStats &t) {
    t.jobs = 0;
    t.misses = 0;
    t.id = 0;
    t.active = false;
    t.start_us = 0;
    t.release_us = 0;
    t.max_exec_us = 0;
    t.worst_lateness_us = 0;
  }

  static void beginTask(TaskStats &t, uint32_t id, uint64_t release_us) {
    t.active = true;
    t.id = id;
    t.release_us = release_us;
    t.start_us = micros();
  }

  static void endTask(TaskStats &t) {
    if (!t.active) return;
    
    const uint64_t end_us = micros();
    const uint64_t exec_us = end_us - t.start_us;
    const uint64_t abs_deadline_us = t.release_us + t.deadline_us;
    const int64_t lateness_us = (int64_t)end_us - (int64_t)abs_deadline_us;
    
    if (exec_us > t.max_exec_us) {
      t.max_exec_us = exec_us;
    }
    if (lateness_us > t.worst_lateness_us) {
      t.worst_lateness_us = lateness_us;
    }
    t.jobs++;
    if (lateness_us > 0) {
      t.misses++;
    }
    t.active = false;
  }

  uint64_t popSRelease() {
    if (s_q_count_ == 0u) {
      return micros();
    }
    const uint64_t t = s_release_us_[s_q_head_];
    s_q_head_ = (s_q_head_ + 1u) % S_RELEASE_Q_MAX;
    s_q_count_--;
    return t;
  }

  static void reportOne(const char *name, const TaskStats &t) {
    Serial.printf("[MON] %s jobs=%" PRIu32 " misses=%" PRIu32 " max_exec=%lluus worst_late=%" PRIi64 "us\n",
                  name, t.jobs, t.misses, t.max_exec_us, t.worst_lateness_us);
  }
};

static TimingMonitor g_monitor;

// ========== FREERTOS OBJECTS ==========
SemaphoreHandle_t xSemaphore_S = NULL;
SemaphoreHandle_t xMutex_Token = NULL;
volatile uint32_t last_INS_trigger_time = 0;
volatile uint32_t last_MODE_trigger_time = 0;
volatile bool mode_enabled = false;
volatile bool sync_received = false;

// ========== GLOBAL VARIABLES ==========
uint32_t IDA = 0, IDB = 0, IDAGG = 0, IDC = 0, IDD = 0, IDS = 0;
uint32_t latest_tokenA = 0, latest_tokenB = 0;
bool tokenA_ready = false, tokenB_ready = false;
int16_t prev_count_A = 0, prev_count_B = 0;

// ========== PCNT ==========
void setup_pcnt() {
  pcnt_config_t pcnt_config_A = {
    .pulse_gpio_num = IN_A_PIN,
    .ctrl_gpio_num = PCNT_PIN_NOT_USED,
    .lctrl_mode = PCNT_MODE_KEEP,
    .hctrl_mode = PCNT_MODE_KEEP,
    .pos_mode = PCNT_COUNT_INC,
    .neg_mode = PCNT_COUNT_DIS,
    .counter_h_lim = 32767,
    .counter_l_lim = -32768,
    .unit = PCNT_UNIT_A,
    .channel = PCNT_CHANNEL_0,
  };
  pcnt_unit_config(&pcnt_config_A);
  pcnt_counter_pause(PCNT_UNIT_A);
  pcnt_counter_clear(PCNT_UNIT_A);
  pcnt_counter_resume(PCNT_UNIT_A);

  pcnt_config_t pcnt_config_B = {
    .pulse_gpio_num = IN_B_PIN,
    .ctrl_gpio_num = PCNT_PIN_NOT_USED,
    .lctrl_mode = PCNT_MODE_KEEP,
    .hctrl_mode = PCNT_MODE_KEEP,
    .pos_mode = PCNT_COUNT_INC,
    .neg_mode = PCNT_COUNT_DIS,
    .counter_h_lim = 32767,
    .counter_l_lim = -32768,
    .unit = PCNT_UNIT_B,
    .channel = PCNT_CHANNEL_0,
  };
  pcnt_unit_config(&pcnt_config_B);
  pcnt_counter_pause(PCNT_UNIT_B);
  pcnt_counter_clear(PCNT_UNIT_B);
  pcnt_counter_resume(PCNT_UNIT_B);
}

uint32_t EdgesInLastWindow_A() {
  int16_t current_count;
  pcnt_get_counter_value(PCNT_UNIT_A, &current_count);
  int16_t delta = current_count - prev_count_A;
  prev_count_A = current_count;
  return (uint32_t)delta;
}

uint32_t EdgesInLastWindow_B() {
  int16_t current_count;
  pcnt_get_counter_value(PCNT_UNIT_B, &current_count);
  int16_t delta = current_count - prev_count_B;
  prev_count_B = current_count;
  return (uint32_t)delta;
}

// ========== ISR FOR SPORADIC ==========
void IRAM_ATTR INS_ISR() {
  uint32_t current_time = micros();
  
  if ((current_time - last_INS_trigger_time) >= DEBOUNCE_DELAY_US) {
    last_INS_trigger_time = current_time;
    
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    g_monitor.notifySRelease();
    xSemaphoreGiveFromISR(xSemaphore_S, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}

// ========== ISR FOR MODE TOGGLE ==========
void IRAM_ATTR MODE_ISR() {
  uint32_t current_time = micros();
  
  if ((current_time - last_MODE_trigger_time) >= MODE_DEBOUNCE_DELAY_US) {
    last_MODE_trigger_time = current_time;
    mode_enabled = !mode_enabled;
  }
}

// ========== TASK A (10ms period, priority 5) ==========
void TaskA(void *pvParameters) {
  // Wait for SYNC
  while (!sync_received) {
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod = pdMS_TO_TICKS(10);
  
  for (;;) {
    uint32_t countA = EdgesInLastWindow_A();
    uint32_t seed = ((uint32_t)IDA << 16) ^ countA ^ 0xA1;
    
    g_monitor.beginTaskA(IDA);
    digitalWrite(ACK_A_PIN, HIGH);
    uint32_t tokenA = WorkKernel(672000, seed);
    digitalWrite(ACK_A_PIN, LOW);
    g_monitor.endTaskA();
    
    Serial.printf("A,%u,%u,0x%08X\n", IDA, countA, tokenA);
    
    xSemaphoreTake(xMutex_Token, portMAX_DELAY);
    latest_tokenA = tokenA;
    tokenA_ready = true;
    xSemaphoreGive(xMutex_Token);
    
    IDA++;
    
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}

// ========== TASK B (20ms period, priority 4) ==========
void TaskB(void *pvParameters) {
  // Wait for SYNC
  while (!sync_received) {
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod = pdMS_TO_TICKS(20);
  
  for (;;) {
    uint32_t countB = EdgesInLastWindow_B();
    uint32_t seed = ((uint32_t)IDB << 16) ^ countB ^ 0xB2;
    
    g_monitor.beginTaskB(IDB);
    digitalWrite(ACK_B_PIN, HIGH);
    uint32_t tokenB = WorkKernel(960000, seed);
    digitalWrite(ACK_B_PIN, LOW);
    g_monitor.endTaskB();
    
    Serial.printf("B,%u,%u,0x%08X\n", IDB, countB, tokenB);
    
    xSemaphoreTake(xMutex_Token, portMAX_DELAY);
    latest_tokenB = tokenB;
    tokenB_ready = true;
    xSemaphoreGive(xMutex_Token);
    
    IDB++;
    
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}

// ========== TASK AGG (20ms period, priority 3) ==========
void TaskAGG(void *pvParameters) {
  // Wait for SYNC
  while (!sync_received) {
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod = pdMS_TO_TICKS(20);
  
  for (;;) {
    uint32_t agg;
    
    xSemaphoreTake(xMutex_Token, portMAX_DELAY);
    if (tokenA_ready && tokenB_ready) {
      agg = latest_tokenA ^ latest_tokenB;
    } else {
      agg = 0xDEADBEEF;
    }
    xSemaphoreGive(xMutex_Token);
    
    uint32_t seed = ((uint32_t)IDAGG << 16) ^ agg ^ 0xD4;
    
    g_monitor.beginTaskAGG(IDAGG);
    digitalWrite(ACK_AGG_PIN, HIGH);
    uint32_t tokenAGG = WorkKernel(480000, seed);
    digitalWrite(ACK_AGG_PIN, LOW);
    g_monitor.endTaskAGG();
    
    Serial.printf("AGG,%u,0x%08X,0x%08X\n", IDAGG, agg, tokenAGG);
    IDAGG++;
    
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}

// ========== TASK C (50ms period, priority 2) ==========
void TaskC(void *pvParameters) {
  // Wait for SYNC
  while (!sync_received) {
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod = pdMS_TO_TICKS(50);
  
  for (;;) {
    if (mode_enabled) {
      uint32_t seed = ((uint32_t)IDC << 16) ^ 0xC3;
      
      g_monitor.beginTaskC(IDC);
      digitalWrite(ACK_C_PIN, HIGH);
      uint32_t tokenC = WorkKernel(1680000, seed);
      digitalWrite(ACK_C_PIN, LOW);
      g_monitor.endTaskC();
      
      Serial.printf("C,%u,0x%08X\n", IDC, tokenC);
      IDC++;
    }
    
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}

// ========== TASK D (50ms period, priority 2) ==========
void TaskD(void *pvParameters) {
  // Wait for SYNC
  while (!sync_received) {
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod = pdMS_TO_TICKS(50);
  
  for (;;) {
    if (mode_enabled) {
      uint32_t seed = ((uint32_t)IDD << 16) ^ 0xD5;
      
      g_monitor.beginTaskD(IDD);
      digitalWrite(ACK_D_PIN, HIGH);
      uint32_t tokenD = WorkKernel(960000, seed);
      digitalWrite(ACK_D_PIN, LOW);
      g_monitor.endTaskD();
      
      Serial.printf("D,%u,0x%08X\n", IDD, tokenD);
      IDD++;
    }
    
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}

// ========== TASK S (sporadic, priority 1) ==========
void TaskS(void *pvParameters) {
  // Wait for SYNC
  while (!sync_received) {
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  
  for (;;) {
    if (xSemaphoreTake(xSemaphore_S, portMAX_DELAY) == pdTRUE) {
      uint32_t seed = ((uint32_t)IDS << 16) ^ 0x55;
      
      g_monitor.beginTaskS(IDS);
      digitalWrite(ACK_S_PIN, HIGH);
      uint32_t tokenS = WorkKernel(600000, seed);
      digitalWrite(ACK_S_PIN, LOW);
      g_monitor.endTaskS();
      
      Serial.printf("S,%u,0x%08X\n", IDS, tokenS);
      IDS++;
    }
  }
}

// ========== MONITOR TASK (priority 0) ==========
void MonitorTask(void *pvParameters) {
  // Wait for SYNC
  while (!sync_received) {
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  
  for (;;) {
    if (!g_monitor.allDeadlinesMet()) {
      digitalWrite(PIN_ERR, HIGH);
    }
    
    if (g_monitor.pollReports()) {
      while (true) {
        vTaskDelay(portMAX_DELAY);
      }
    }
    
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// ========== SETUP ==========
void setup() {
  setCpuFrequencyMhz(240);
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n=== B31DG Assignment 3 - FreeRTOS ===");
  Serial.printf("CPU: %d MHz\n", getCpuFrequencyMhz());
  
  g_monitor.setPeriodicReportEverySeconds(0);
  g_monitor.setFinalReportAfterSeconds(FINAL_REPORT_AFTER_SECONDS);
  
  pinMode(SYNC_PIN, INPUT_PULLUP);
  pinMode(IN_A_PIN, INPUT);
  pinMode(IN_B_PIN, INPUT);
  pinMode(IN_S_PIN, INPUT_PULLUP);
  pinMode(IN_MODE_PIN, INPUT_PULLUP);
  
  pinMode(ACK_A_PIN, OUTPUT);
  pinMode(ACK_B_PIN, OUTPUT);
  pinMode(ACK_AGG_PIN, OUTPUT);
  pinMode(ACK_C_PIN, OUTPUT);
  pinMode(ACK_D_PIN, OUTPUT);
  pinMode(ACK_S_PIN, OUTPUT);
  pinMode(PIN_ERR, OUTPUT);
  
  digitalWrite(ACK_A_PIN, LOW);
  digitalWrite(ACK_B_PIN, LOW);
  digitalWrite(ACK_AGG_PIN, LOW);
  digitalWrite(ACK_C_PIN, LOW);
  digitalWrite(ACK_D_PIN, LOW);
  digitalWrite(ACK_S_PIN, LOW);
  digitalWrite(PIN_ERR, LOW);
  
  setup_pcnt();
  
  xSemaphore_S = xSemaphoreCreateBinary();
  xMutex_Token = xSemaphoreCreateMutex();
  
  attachInterrupt(digitalPinToInterrupt(IN_S_PIN), INS_ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(IN_MODE_PIN), MODE_ISR, FALLING);
  
  // Create all tasks - they start immediately but wait for SYNC
  xTaskCreate(TaskA, "TaskA", 2048, NULL, 5, NULL);
  xTaskCreate(TaskB, "TaskB", 2048, NULL, 4, NULL);
  xTaskCreate(TaskAGG, "TaskAGG", 2048, NULL, 3, NULL);
  xTaskCreate(TaskC, "TaskC", 2048, NULL, 2, NULL);
  xTaskCreate(TaskD, "TaskD", 2048, NULL, 2, NULL);
  xTaskCreate(TaskS, "TaskS", 2048, NULL, 1, NULL);
  xTaskCreate(MonitorTask, "Monitor", 2048, NULL, 0, NULL);
  
  Serial.println("Waiting for SYNC...");
  while (digitalRead(SYNC_PIN) == HIGH) delay(1);
  while (digitalRead(SYNC_PIN) == LOW) delay(1);
  Serial.println("SYNC detected!");
  
  // Reset all counters
  IDA = IDB = IDAGG = IDC = IDD = IDS = 0;
  tokenA_ready = tokenB_ready = false;
  prev_count_A = prev_count_B = 0;
  pcnt_counter_clear(PCNT_UNIT_A);
  pcnt_counter_clear(PCNT_UNIT_B);
  last_INS_trigger_time = 0;
  last_MODE_trigger_time = 0;
  mode_enabled = false;
  
  // Set T0 and signal tasks to start
  g_monitor.synch();
  sync_received = true;
  
  Serial.println("FreeRTOS tasks started!");
  Serial.println("Press IN_MODE button to enable/disable C and D\n");
}

void loop() {
  vTaskDelete(NULL);
}