# 200 MPH Drone Real-Time-Operating-System (RTOS)

---

My friend and I are attempting to build a drone that goes 200 mph while designing everything from scratch. I am going to be focusing on the software part of the project, making a Real-Time Operating System (RTOS), to schedule processes, run PID loops, etc., with minimal latency.

To safely control a quadcopter moving at these extreme velocities, standard cooperative loop structures are not fast enough. A single delayed calculation or blocked process means the drone travels dozens of feet before the next correction can be made. This repository contains a custom-built, hybrid cooperative/preemptive real-time operating system (RTOS) designed from the ground up for the STM32F7 (ARM Cortex-M7) microcontroller to achieve ultra-low latency, deterministic flight control.

---

## Architecture: Why a Custom RTOS?

At 200 MPH, flight stabilization calculations cannot wait for background tasks like GPS parsing, battery monitoring, or telemetry transmission.

My hybrid RTOS solves this by dividing the execution into two distinct areas: 
1. **The Scheduled Zone (Cooperative):** Low-to-medium priority tasks (like battery telemetry and OSD updating) run sequentially.
2. **The Interrupt Zone (Preemptive):** High-priority tasks (such as IMU sensor reads and critical RC radio packet parsing) completely bypass the standard queue. The kernel uses the ARM Cortex-M7 `PendSV` exception vector to execute instant, deterministic stack swaps.

### Key Technical Milestones Replaced & Completed:
* **Asynchronous Context Saving/Restoring:** Fully implemented in assembly (`PendSV_Handler.S`) to preserve the entire CPU register set (`R4-R11` manually and hardware-stacked registers) during preemptive swaps, preventing task loss and memory corruption.
* **C++ Parameter Overloading:** Created clean compiler-enforced task creation pipelines (`initializeNewTask()`) to natively separate `SCHEDULED` tasks from asynchronous `INTERRUPT` tasks without relying on "hacky" task parameters (e.g., 0 ms execution time).
* **Low-Level Hardware Drivers:** Built non-blocking SPI burst-read pipelines to ingest 12-byte raw IMU packets from the BMI270 and UART DMA ring buffer parsing to extract 26-byte CRSF packets on the fly.

---

## Interactive Simulation Demo: 

I have set up an interactive simulation environment using **Renode** so that reviewers can watch the preemption kernel work in real-time on a virtual STM32F7, without needing physical hardware.

During the simulation, a low-priority background task continuously increments a counter. To simulate real-world asynchronous hardware events, an inline random-number generator inside the `SysTick_Handler` periodically triggers a high-priority interrupt task. You will see the interrupt seamlessly hijack the CPU, print the perfectly preserved counter state, run a heavy processing delay, and hand control back to the exact instruction where the counter left off.

### How to Run:
1. **Install Renode** on your machine ([Download Renode](https://renode.io/) or run `brew install renode` on macOS / `sudo apt install renode` on Linux).
2. **Clone this repository** and navigate to the root directory:
   ```bash
   git clone https://github.com/Guag914/200-MPH-Drone-RTOS.git
   cd 200-MPH-Drone-RTOS
   
3. **Execute the run script** to launch the virtual machine and open the serial analyzer:
   * Mac/Linux: 
     ```bash
     chmod +x run.sh
     ./run.sh
     
   * Windows:
     ```bash
     run.bat
     
4. In the primary **Renode** window, type start to being the simulation, and quit to stop the simulation.
5. **Note:** If you want to code your own custom tasks via `User_Tasks/user_tasks.cpp`, you must install the arm-none-eabi-gcc, and arm-none-eabi-g++ compilers.
   * **Windows**: 
     * Go to [https://developer.arm.com/downloads/-/gnu-rm](https://developer.arm.com/downloads/-/gnu-rm) and download the exe
     * Run the installer, and make sure to check the box that says **Add path to envirnment variable**
     * Test the installation with: 
     
       ```cmd
       arm-none-eabi-gcc --version
   * **MacOS**:
     * Install it with:
       ```bash
         brew install armmbed/formulae/arm-none-eabi-gcc
     
     * Test the installation with:
      
       ```bash
         arm-none-eabi-gcc --version
       
   * **Linux**:
       * Install it with:
         ```bash
           sudo apt update
           sudo apt install gcc-arm-none-eabi
       * Test the installation with:

         ```bash
           arm-none-eabi-gcc --version

### Compiling Your Custom Tasks
Once you have modified `User_Tasks/user_tasks.cpp` with your own loops, you must recompile the binary so the simulator can run your updated code.

1. Ensure you have **CMake** and **Ninja** installed on your system.
2. Run the build script from the root of the project to recompile:
    * **Mac / Linux**:
      ```bash
      chmod +x build.sh
      ./build.sh 
      ```
    * **Windows**:
      ```cmd
      build.bat
      ```
3. Flags: 
   *  ``` 
      --debug #ensures methods log to usart1 peripheral (not usable with user mode)
   * ```
     sim #enables sim mode, allowing renode to simulate the hardware
   * ```
     user #disables drone tasks, and allows the user to compile their own tasks via User_Tasks.cpp
     
4. Example Commands: 
   * **Mac / Linux**:
     ```bash
     ./build.sh sim --debug #run this mode to see the drone tasks actively running
     ./build.sh user #run this mode to compile your own tasks
   * **Windows**:
     ```cmd
       build.bat sim --debug #run this mode to see the drone tasks actively running
       build.bat user #run this mode to compile your own tasks
---

## Repository Structure:
* `/src/flight` - Core flight controller application code:
  * `Tasks.cpp` - Raw IMU processing, CRSF bitwise radio packet parsing, and failsafe logic. 
  * `BufferPopulation.cpp` - Low-level DMA and SPI register config + read and write to populate buffers needed for calculations.
* `src/rtos` - The custom RTOS kernel:
  * `Main.cpp` / `rtos.h` - Task control blocks, prioritized scheduler, and overload initializers.
  * `PendSV_Handler.S` - Core assembly routine saving/restoring CPU context registers (**R4-R11**).

### Roadmap: 
| Phase | Name                 | Description                                                                            | Status            |
|-------|----------------------|----------------------------------------------------------------------------------------|-------------------|
| #1    | Custom RTOS Kernel   | Execute both **preemptive** and **cooperative** tasks with a priority-based system     | ***Complete***    |
| #2    | Flight Dynamics      | **PT1** Low-Pass Filtering, **Axis Alignment** Mapping, OSD Warn **State Machine**     | ***In Progress*** |
| #3    | Controller Loops     | High rate **PID** controller loop and **DShot** generation                             | ***Not Started*** |
| #4    | Hardware Integration | Full Software-In-Loop (SIL) simulation for the ESC and transmitter modules             | ***Not Started*** |
| #5    | User Features        | CLI for config, CPP bootloader for microcontrollers, USBC integration for bench testing | ***Not Started*** |

As for a timeline, I am hoping to get the entire system done before around July 27th. Of course when my friend finishes the hardware portion of the project, we will merge the custom PCBs and the operating system to create a fully functional high-speed drone. 

The first version of the drone will likely be done around mid-August, with the final version being completed around the end of August.