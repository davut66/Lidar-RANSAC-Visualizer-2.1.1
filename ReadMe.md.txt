# Lidar Data Processing & RANSAC Visualization

[TR]
Bu proje, Lidar sensöründen alınan ham verileri işleyerek ortam haritasını çıkaran ve **RANSAC algoritması** kullanarak engelleri (duvarları) tespit eden bir C uygulamasıdır. 

**Projenin Amacı:**
Gürültülü sensör verilerini anlamlı hale getirmek ve robotun çevresindeki engelleri matematiksel doğrular olarak tanımlamaktır. Elde edilen veriler **Gnuplot** kullanılarak görselleştirilir.

**Özellikler:**
- URL üzerinden veri çekme (`curl` entegrasyonu).
- Polar koordinatların Kartezyen koordinatlara dönüşümü.
- RANSAC ile doğru/duvar tespiti.
- Doğruların kesişim noktalarının hesaplanması.

---

[EN]
This project is a C application that processes raw Lidar sensor data to map the environment and detects obstacles (walls) using the **RANSAC algorithm**.

**Goal:**
To interpret noisy sensor data and define surrounding obstacles as mathematical lines. The results are visualized using **Gnuplot**.

**Features:**
- Data fetching via URL (`curl` integration).
- Conversion from Polar to Cartesian coordinates.
- Line/Wall detection using RANSAC.
- Calculation of intersection points.

## Build & Run / Kurulum ve Çalıştırma

**Dependencies / Gereksinimler:**
- GCC Compiler
- Gnuplot
- Curl

**Compile / Derleme:**
```bash
gcc main.c -o lidar_app -lm