# B31DG Assignment 3 - FreeRTOS Implementation

## Project Overview
Real-time multi-task system implemented using FreeRTOS on ESP32 microcontroller.

## Author
- **Name**: Kuroekegha Francis-Epe
- **Student ID**: H00518884
- **Institution**: Heriot-Watt University 
- **Course**: MSc Robotics 
- **Module**: B31DG Embedded Software

## System Specifications

### Hardware
- **Microcontroller**: ESP32 @ 240MHz
- **Development Board**: [Your specific board]

### Pin Configuration
| Function | GPIO Pin |
|----------|----------|
| SYNC     | 33       |
| IN_A     | 32       |
| IN_B     | 27       |
| IN_S     | 25       |
| IN_MODE  | 26       |
| ACK_A    | 19       |
| ACK_B    | 2        |
| ACK_AGG  | 4        |
| ACK_C    | 17       |
| ACK_D    | 5        |
| ACK_S    | 16       |
| PIN_ERR  | 13       |

### Task Set
| Task | Period | Execution Time | Priority | Description |
|------|--------|----------------|----------|-------------|
| A    | 10ms   | 2.8ms         | 5        | Pulse counting and token generation |
| B    | 20ms   | 4.0ms         | 4        | Pulse counting and token generation |
| AGG  | 20ms   | 2.0ms         | 3        | Token aggregation |
| C    | 50ms   | 7.0ms         | 2        | Conditional execution task |
| D    | 50ms   | 4.0ms         | 2        | Conditional execution task |
| S    | Sporadic | 2.5ms       | 1        | Button-triggered sporadic task |

## Implementation Details

### FreeRTOS Components Used
- **Tasks**: 7 tasks (6 periodic + 1 monitor)
- **Semaphores**: 1 binary semaphore for sporadic task signaling
- **Mutexes**: 1 mutex for token access protection
- **Scheduling**: Rate Monotonic Scheduling (RMS)

### Key Features
- Zero deadline misses achieved across all tasks
- Priority inheritance for mutex to prevent priority inversion
- ISR-based sporadic task triggering with debouncing
- Integrated timing monitor for deadline tracking
- Absolute periodic timing using vTaskDelayUntil()

## Files

- `Assignment3_FreeRTOS.ino` - Main source code with FreeRTOS implementation
- `README.md` - This documentation file

## Build Instructions

### Prerequisites
- Arduino IDE 1.8.x or 2.x
- ESP32 board support installed
- FreeRTOS (included with ESP32 Arduino core)

### Compilation
1. Open `Assignment3_FreeRTOS.ino` in Arduino IDE
2. Select **Tools → Board → ESP32 Dev Module**
3. Select **Tools → CPU Frequency → 240MHz**
4. Select correct **Port**
5. Click **Upload**

### Serial Monitor Settings
- **Baud Rate**: 115200
- **Line Ending**: Newline

## Results

### Final Report (10-second test)
[MON] A jobs=1005 misses=0 max_exec=2822us worst_late=0us
[MON] B jobs=503 misses=0 max_exec=4008us worst_late=0us
[MON] AGG jobs=503 misses=0 max_exec=2003us worst_late=0us
[MON] C jobs=201 misses=0 max_exec=7002us worst_late=0us
[MON] D jobs=201 misses=0 max_exec=4002us worst_late=0us
[MON] S jobs=10 misses=0 max_exec=5024us worst_late=0us
**All tasks: 0 deadline misses **

## Design Decisions

### Task Synchronization
Initial implementation used vTaskSuspend/Resume but this caused phase misalignment and deadline violations. Final implementation uses vTaskDelay with flag polling for reliable synchronized startup.

### Priority Assignment
Rate Monotonic Scheduling with priorities inversely proportional to task periods ensures optimal deadline adherence.


