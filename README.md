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
                             <─────── │   VINS    │  T_imu                                                                       │   │
                             <─────── │           │                                               "/tmp/uav_img"   <─────────────+   │
                             <─────── │           │                                                      │                           │
                                      │           │                                         d_t7         v              d_t6         │
                                      │           │  <─────────────  "/tmp/uav_cam0"   <───────────   uav_camera   <─────────────────+
                                      +───────────+  T_img                                                              loopback
```

Step 1: Definition
- T_imu = T1 + d_t11 + d_t12 + d_t4 + d_t5
- T_img = T2 + d_t21 + d_t22 + d_t23 + d_t4 +d_t6 +d_t7
- T_1 = T_m - d_t12 - d_t11
- T_2 = T_m - d_t23 - d_t22 - d_t21

Step 2: Estimation

- T_1: unknow
- T_2: unknow
- T_m: time sync point
- d_t11: depends on image complexity
- d_t12: [ignore] memory + cpu, loop back
- d_t21: depends on UART round trip time
- d_t22: [ignore] memory + cpu, ring buffer push (O1)
- d_t23: [ignore] memory + cpu, ring buffer popup (O1)
- d_t4: depends on network and communication protocl
- d_t5: [ignore] memory + cpu, topic publish
- d_t6: [ignore] memory + cpu, loop back
- d_t7: [ignore] memory + cpu, topic publish

Step 3: Simplification

- T_imu = T1 + d_t11 + d_t4
- T_img = T2 + d_t21 + d_t4
- T_1 = T_m - d_t11
- T_2 = T_m - d_t21

Step 4: Factor

- encoding time: can't be sure, no value from RTP packet
- gap time: depends on encoding/loopback transfer/FPS

estimated(assume):

- encoding is longer than RTP worst case, T_a = T_m - RTP_max_transfer_time

```
     T_a               T_m            T_l        T_a +interval(1/FPS)
      │                 │              │            │
      │    encoding     │ RTP transfer │   gap      │
      │                 │              │            │
      │                 │              │            │
   Timestamp          OS time        OS time      
```
