Finding the Breaking Point: Rotary Inverted Pendulum LQR Experiment

Hello! This repository documents my school science fair project: a custom-built Rotary Inverted Pendulum (Furuta Pendulum). I wanted to move beyond a basic project and build something that combined my interest in 3D printing and robotics with complex control theory.

The Project Goal

The core objective of this experiment is to determine how the position of a fixed mass along the pendulum’s arm affects the stabilization time of a Linear Quadratic Regulator (LQR) control system. This has direct real-world relevance to aerospace rocket gimbals, autonomous drones, and the mechanics of modern prosthetics.

The Hypothesis

If a fixed mass is moved further from the pivot point, the moment of inertia increases. This changes the physical parameters the controller was designed for, eventually exceeding the stability limits of the fixed LQR gains.

Hardware & Technical Stack

I designed this to be a high-performance, low-cost prototype using primarily parts I already had:

    Host: Odroid XU4 (Running Armbian and Klipper).
    MCU: Tronxy X1 Motherboard (Interfaced with Klipper for stepper control).
    Actuator: NEMA 17 Stepper Motor.
    Sensor: AS5048A Magnetic Encoder connected via an ESP32 for high-speed angle reading.
    Logic: Custom Python LQR script designed to compensate for a 45ms system latency through predictive functions.


Development Status

As of March 2026, the first prototype is fully assembled. I have successfully shrunk the electronics enclosure to less than half its original size for a cleaner look.

Hardware Solutions: To solve the challenge of mounting the encoder near the magnet without interference, I utilized a specialized 3D-printed Tower that aligns the AS5048A centered over the magnet, secured with non-interfering M2 screws.
Findings

By the end of the study, I conducted 1,200 automated trials across six different mass positions. The data revealed a critical "breaking point" at Hole 5, where the controller could no longer overcome the moment of inertia, resulting in a total loss of stability.
