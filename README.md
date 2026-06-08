<p align="center">
  <img src="https://capsule-render.vercel.app/api?type=waving&amp;color=0:00e1ff,100:0055ff&amp;height=200&amp;section=header&amp;text=Esp-mocap-drone&amp;fontSize=45&amp;fontAlignY=35&amp;animation=fadeIn&amp;fontColor=ffffff"/>
</p>

<p align="center">
  <img src="https://img.shields.io/badge/-TypeScript-3178C6?logo=typescript&amp;logoColor=white&amp;style=for-the-badge"/> <img src="https://img.shields.io/badge/-React-61DAFB?logo=react&amp;logoColor=white&amp;style=for-the-badge"/> <img src="https://img.shields.io/badge/-Python-3776AB?logo=python&amp;logoColor=white&amp;style=for-the-badge"/> <img src="https://img.shields.io/badge/-C%2B%2B-00599C?logo=cplusplus&amp;logoColor=white&amp;style=for-the-badge"/> <img src="https://img.shields.io/badge/-Three.js-000000?logo=threedotjs&amp;logoColor=white&amp;style=for-the-badge"/>
</p>

<p align="center">
  <b>Low-cost motion capture system for autonomous indoor drone flight</b>
</p>

---

## 🛸 Overview

A complete indoor motion capture system using ESP32 receivers, a flight controller, and a React+Three.js web interface with computer vision for autonomous drone tracking.

---

## 🛠️ Architecture

```mermaid
flowchart LR
    A[ESP32 Receivers] --> B[Position Server\nPython/Node]
    B --> C[Web GCS\nReact + Three.js]
    D[Drone FC\nArduino] --> B
    C --> E[3D Visualization]
    C --> F[Chart Telemetry]
    B --> G[Socket.io]
```

---

## 📁 Components

| Component | Tech | Role |
|-----------|------|------|
| Receivers | ESP32 / Arduino | Signal capture |
| Computer Vision | Python | Position tracking |
| GCS | React + Three.js | 3D visualization & control |
| Comms | Socket.io | Real-time data |

---

## 🚀 Tech Stack

<img src="https://img.shields.io/badge/-TypeScript-3178C6?logo=typescript&amp;logoColor=white&amp;style=for-the-badge"/> <img src="https://img.shields.io/badge/-React-61DAFB?logo=react&amp;logoColor=white&amp;style=for-the-badge"/> <img src="https://img.shields.io/badge/-Three.js-000000?logo=threedotjs&amp;logoColor=white&amp;style=for-the-badge"/> <img src="https://img.shields.io/badge/-Python-3776AB?logo=python&amp;logoColor=white&amp;style=for-the-badge"/> <img src="https://img.shields.io/badge/-C%2B%2B-00599C?logo=cplusplus&amp;logoColor=white&amp;style=for-the-badge"/> <img src="https://img.shields.io/badge/-Socket.io-010101?logo=socketdotio&amp;logoColor=white&amp;style=for-the-badge"/> <img src="https://img.shields.io/badge/-Vite-646CFF?logo=vite&amp;logoColor=white&amp;style=for-the-badge"/>

---

<p align="center">
  <img src="https://capsule-render.vercel.app/api?type=soft&amp;color=0:00e1ff,100:0055ff&amp;height=100&amp;section=footer&amp;text=see%20the%20unseen&amp;fontSize=20&amp;fontAlignY=50&amp;fontColor=ffffff"/>
</p>
