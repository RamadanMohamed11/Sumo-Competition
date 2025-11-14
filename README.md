# Sumo Robot Competition

A competitive robotics project developed for Sumo robot competitions. This repository contains the electrical schematics, firmware, and control systems for an autonomous Sumo wrestling robot.

## 📋 Project Overview

Sumo Robot Competition is a robotics challenge where autonomous robots compete to push opponents out of a circular ring (Dohyo). This project implements the complete hardware and software solution for a competitive Sumo robot.

## ✨ Features

- **Autonomous Operation**: Self-contained decision-making and movement control
- **Opponent Detection**: Sensor-based opponent location and tracking
- **Edge Detection**: Ring boundary detection to prevent self-elimination
- **Strategic Movement**: Aggressive pushing and defensive maneuvering algorithms
- **Competition-Ready**: Designed to meet standard Sumo robot competition specifications

## 🛠️ Technologies Used

- **Microcontroller**: Arduino/AVR-based control system
- **Programming Language**: C/C++ (Arduino)
- **Sensors**: 
  - Ultrasonic/IR sensors for opponent detection
  - Line sensors for edge detection
- **Motors**: DC motors with H-bridge motor drivers
- **Power System**: LiPo/NiMH battery management

## 📁 Project Structure

```
Sumo-Competition/
└── Electrical Section/
    ├── Circuit diagrams
    ├── PCB layouts
    └── Component specifications
```

## 🎯 Competition Strategy

The robot is programmed with multiple behavioral modes:
- **Search Mode**: Scans for opponent when no target is detected
- **Attack Mode**: Charges at detected opponent
- **Edge Avoidance**: Reverses and repositions when ring edge is detected
- **Defense Mode**: Strategic positioning and counter-attacks

## 🚀 Getting Started

### Prerequisites

- Arduino IDE (version 1.8.x or higher)
- USB cable for programming
- Required libraries (list specific libraries used)

### Installation

1. Clone the repository:
```bash
git clone https://github.com/RamadanMohamed11/Sumo-Competition.git
cd Sumo-Competition
```

2. Open the main Arduino sketch in Arduino IDE

3. Install required libraries via Library Manager

4. Connect your microcontroller via USB

5. Select the correct board and port in Tools menu

6. Upload the code to your robot

### Hardware Setup

1. Assemble the robot chassis according to the mechanical design
2. Mount sensors at optimal positions:
   - Front-facing opponent detection sensors
   - Underside edge detection sensors
3. Connect motor drivers to the microcontroller
4. Wire the power distribution system
5. Perform sensor calibration before competition

## ⚙️ Configuration

Adjust robot behavior by modifying constants in the code:
- **Speed Settings**: Motor PWM values
- **Sensor Thresholds**: Detection sensitivity
- **Timing Parameters**: Reaction delays and movement durations
- **Strategy Weights**: Behavioral decision priorities

## 🏆 Competition Rules Compliance

This robot is designed to comply with standard Sumo robot competition rules:
- Maximum dimensions: 20cm x 20cm (verify with your competition)
- Weight limit: 3kg (verify with your competition)
- Autonomous operation (no remote control)
- 5-second startup delay after placement

## 🔧 Troubleshooting

**Robot doesn't respond:**
- Check battery voltage and connections
- Verify code upload was successful
- Ensure power switch is ON

**Sensors not detecting:**
- Clean sensor lenses
- Check sensor wiring connections
- Recalibrate sensor thresholds

**Erratic movement:**
- Verify motor connections
- Check motor driver functionality
- Balance weight distribution

## 🤝 Contributing

Contributions are welcome! This project can be improved with:
- Enhanced detection algorithms
- Improved strategy implementations
- Better power management
- Documentation improvements

## 📝 License

This project is open source and available for educational and competitive robotics purposes.

## 👨‍💻 Author

**Ramadan Mohamed**
- GitHub: [@RamadanMohamed11](https://github.com/RamadanMohamed11)
- Role: Embedded Systems Engineer & Flutter Developer

## 🙏 Acknowledgments

- Assiut Robotics Team for collaboration and testing
- Competition organizers and fellow roboticists
- Open-source robotics community

---

**Note**: This is a competition robot project. Always follow safety guidelines when testing and competing with autonomous robots.