# Intelligent Assistive Cane with Vision-Based Navigation and Sensor Fusion
## 📌 Overview
This project presents a smart assistive cane designed to improve mobility for visually 
impaired individuals using AI-enhanced sensor fusion, cloud vision, and GPS SOS on the 
ESP32-S3.

## 🚀 Features
- Obstacle detection using ultrasonic sensor (fail-safe, low-latency, works offline)
- Object recognition & OCR (text reading) via ESP32-CAM + Cloud Vision API
- Phone-based GPS tracking with SOS alert to caregiver
- Natural voice feedback via TTS (text-to-speech), with buzzer/vibration as fail-safe
- Ergonomic handle design (CAD — planned, not yet finalized)

## 🧠 System Components
- ESP32-S3 (dual-core: one core for real-time safety, one for AI/communication)
- ESP32-CAM module (scene capture)
- Cloud Vision API (object ID + OCR + scene description)
- HC-SR04 Ultrasonic Sensor (hazard distance)
- Phone GPS via companion app (location + SOS)
- TTS engine (voice feedback)
- Buzzer & Vibration Motor (fail-safe feedback)
- Regulated power bank (replacing earlier MT3608/TP4056 setup)

## 🛠️ Design
The system is divided into:
- **Power section** — regulated power bank (replacing earlier MT3608/TP4056 setup)
- **Interface section** — shaft connection (ergonomic handle, CAD-designed — planned)
- **Logic section** — ESP32-S3, ESP32-CAM + Cloud Vision (object ID/OCR), ultrasonic 
  sensor (hazard detection), phone GPS (location + SOS), TTS engine (voice feedback)

## 📊 Project Status
🔧 Prototype stage — functional and tested

✅ Working: obstacle detection (ultrasonic + ESP32-CAM), cloud vision object ID/OCR, 
   phone-based GPS tracking, TTS voice feedback
🚧 In progress: CAD-designed enclosure, fall detection (gyroscope + SOS button), 
   on-device TinyML, weatherproofing

## 🛠️ Design
The system is divided into:
- **Power section** — regulated power bank (replacing earlier MT3608/TP4056 setup)
- **Interface section** — shaft connection (ergonomic handle, CAD-designed)
- **Logic section** — ESP32-S3, ESP32-CAM + Cloud Vision (object ID/OCR), 
  ultrasonic sensor (hazard detection), phone GPS (location + SOS), TTS engine (voice feedback)


📊 Project Status
🔧 Prototype stage — functional and tested

✅ Working: obstacle detection (ultrasonic + ESP32-CAM), cloud vision object ID/OCR, 
   phone-based GPS tracking, TTS voice feedback
🚧 In progress: CAD-designed enclosure, fall detection (gyroscope + SOS button), 
   on-device TinyML, weatherproofing

## 👥 Team
- Ansinath Nassar
- Divya James
- Milan Roy 
