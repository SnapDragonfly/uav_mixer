# uav_mixer 

Here is a draft time analysis.


```
              Image capture               frame encoding                loopback         T_m            UDP transfer
CMOS sensor   ──────────────> hw encoder ────────────────> RTP client ────────────> udp_forward (mix) ──────────────>  uav_bridge (splitter)
                    T_1                       d_t11                       d_t12           ^              d_t4                │   │   │
                                                                                    d_t23 │ popup                            │   │   │
              Data acquirision            Round Trip Time               push              │                                  │   │   │
IMU chip      ──────────────>    UART    ────────────────> imu_thread ────────────>  ring buffer                             │   │   │
                    T_2                       d_t21                       d_t22                                         d_t5 │   │   │
                                      +───────────+                                                                          │   │   │
                                      │           │  <─────────────────────────────────────────── "/tmp/uav_imu"   <─────────+   │   │
                             <─────── │   VINS    │  T_imu                                                              d_t7     │   │
                             <─────── │           │                                               "/tmp/uav_img"   <─────────────+   │
                             <─────── │           │                                                      │                           │
                                      │           │                                         d_t8         v              d_t6         │
                                      │           │  <─────────────  "/tmp/uav_cam0"   <───────────   uav_camera   <─────────────────+
                                      +───────────+  T_img                                                              loopback
```

Step 1: Definition
- T_imu = T1 + d_t11 + d_t12 + d_t4 + d_t5
- T_img = T2 + d_t21 + d_t22 + d_t23 + d_t4 +d_t6 +d_t8
- T_tim = T2 + d_t21 + d_t22 + d_t23 + d_t4 +d_t7 +d_t8

Step 2: Estimation

**Sensor Characteristic**
- T_1: unknow
- T_2: unknow

**Time Line(Single Point)**
- T_m: time sync point (Camera OS Time)

**Simple Analysis**
- d_t11: depends on image complexity
- d_t12: [ignore] memory + cpu, loop back
- d_t21: depends on UART round trip time
- d_t22: [ignore] memory + cpu, ring buffer push (O1)
- d_t23: [ignore] memory + cpu, ring buffer popup (O1)
- d_t4: depends on network and communication protocol (wfb-ng, ignore)
- d_t5: [ignore] memory + cpu, topic publish
- d_t6: [ignore] memory + cpu, loop back
- d_t7: [ignore] memory + cpu, topic publish
- d_t8: [ignore] memory + cpu, topic publish

Step 3: Simplification

- T_imu = T1 + d_t11 + d_t12[ignore] + d_t4[ignore] + d_t5[ignore] = T1 + d_t11
- T_img = T2 + d_t21 + d_t22[ignore] + d_t23[ignore] + d_t4[ignore] + d_t6[ignore] + d_t8[ignore] = T2 + d_t21
- T_tim = T2 + d_t21 + d_t22[ignore] + d_t23[ignore] + d_t4[ignore] + d_t7[ignore] + d_t8[ignore] = T2 + d_t21

Step 4: Factor

1. encoding time: can't be sure, no value from RTP packet
2. uart transfer time is not stable
3. T_m is the ONLY time sync point
