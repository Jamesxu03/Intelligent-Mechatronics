# Intelligent Mechatronics V2.0 🤖

Welcome to the V2.0 architecture upgrade of the Humanoid Robot and Autonomous Segway platforms!

## Architectural Highlights
- **OOP Refactoring**: Raw loop-based scripts have been replaced with rigorous Object-Oriented classes (e.g., `VisualGestureRecognizer`, `AutonomousPerceptionModule`) mapping strictly to physical domains.
- **Robust Exception Handling**: Migrated raw `print()` loops to the mature `logging` library. Integrated explicit `try-except` fallback constraints for hardware drops (e.g. Serial disconnects, Camera unavailability).
- **Test-Driven Validation**: Incorporated CI/CD friendly offline testing. Core perception methods handle edge cases securely without bringing down the global loop.

## Quick Start
1. Ensure all packages run in their standardized environment:
   ```bash
   pip install -r requirements.txt
   ```
2. To deploy the hand CV integration, launch: 
   ```bash
   python src/Hand/hand_controller.py
   ```
3. To test the visual trajectory pipeline for the car, execute:
   ```bash
   python src/Car/perception.py
   ```

## Running the Automated Test Suite
To verify core calculations without attaching physical cameras or Arduino systems:
```bash
pytest tests/
```
