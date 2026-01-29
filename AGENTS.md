# Repository Guidelines

## Project Structure & Module Organization
- `FtcRobotController/`: SDK app module and sample OpModes provided by FTC.
- `TeamCode/`: team-owned robot code; put new OpModes and subsystems here.
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`: primary Java sources.
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/test/`: on-robot test OpModes.
- `doc/` and `libs/`: documentation, assets, and bundled libraries.
- Gradle files in the repo root (`build.gradle`, `settings.gradle`, `gradlew`) configure the Android build.

## Build, Test, and Development Commands
- `./gradlew assembleDebug`: builds debug APKs for the app modules.
- `./gradlew :TeamCode:assembleDebug`: builds the TeamCode debug APK only.
- `./gradlew build`: runs the full Gradle build (useful for CI-style verification).

## Coding Style & Naming Conventions
- Language: Java. Use Android Studio default formatting (4-space indentation, braces on the same line).
- Class names: `PascalCase`; methods/fields: `lowerCamelCase`; constants: `UPPER_SNAKE_CASE`.
- OpModes: annotate with `@TeleOp` or `@Autonomous`; remove `@Disabled` to make them visible.
- Follow FTC sample naming patterns (e.g., `Concept*`, `Robot*`, `Sensor*`) when creating new sample-like OpModes.

## Testing Guidelines
- No unit-test framework is configured. Validation is primarily on-robot.
- Put quick checks in `TeamCode/.../test/` and keep test OpModes clearly named (e.g., `TestMotorPIDF`).
- Run tests by deploying from Android Studio and selecting the OpMode on the Driver Station.

## Commit & Pull Request Guidelines
- Commit history shows no strict convention; use concise, imperative summaries (e.g., "Fix turret encoder ticks").
- Keep commits focused and describe behavior changes clearly in the PR description.
- Include: purpose, test OpModes run, and any configuration changes (e.g., new devices in the robot configuration).
- If UI or Driver Station changes are relevant, add a screenshot or short note.

## Security & Configuration Tips
- `local.properties` stores local Android SDK paths; do not commit changes.
- Keep keystores and device-specific configuration out of version control unless explicitly required.
