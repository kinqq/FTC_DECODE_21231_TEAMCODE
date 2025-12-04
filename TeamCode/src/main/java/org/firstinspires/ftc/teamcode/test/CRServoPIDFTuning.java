package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.constant.ConstantsServo;

@TeleOp(name = "Magazine PIDF Tuner", group = "Tuning")
public class CRServoPIDFTuning extends OpMode {

    // === HARDWARE NAMES (CHANGE TO MATCH YOUR CONFIG) ===
    private static final String SERVO_NAME   = "turntable";        // CR servo
    private static final String ENCODER_NAME = "encoderDigital";  // digital encoder motor
    private static final String ANALOG_NAME  = "encoder";   // absolute analog encoder

    // Hardware
    private CRServoImplEx servo;
    private DcMotorEx encoder;
    private AnalogInput analog;

    // Time step (assume ~50Hz loop)
    private double dt = 0.02;

    // Homing state
    private boolean homed = false;

    // === HOMING PID (analog angle in degrees) ===
    private double homeKp = 0.01;
    private double homeKi = 0.0;
    private double homeKd = 0.0;

    private double homeIntegral = 0.0;
    private double homeLastError = 0.0;

    // Home target & tolerance
    private double homeTargetDeg = 0.0;   // adjust if "home" is not exactly 0°
    private double homeToleranceDeg = 3.0;
    private double maxHomePower = 0.3;    // keep homing gentle

    // === POSITION PIDF (digital encoder ticks) ===
    private double kP = 0.00033;
    private double kI = 0.0000;
    private double kD = 0.00004;
    private double kF = 0.0;              // often 0 for position

    private double integral = 0.0;
    private double lastError = 0.0;

    private int targetPos = 0;

    // Simple edge detector for buttons
    private GamepadState previous = new GamepadState();

    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void init() {
        servo   = (CRServoImplEx) hardwareMap.get(CRServo.class, SERVO_NAME);
        encoder = hardwareMap.get(DcMotorEx.class, ENCODER_NAME);
        analog  = hardwareMap.get(AnalogInput.class, ANALOG_NAME);

        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Magazine PIDF Tuner init");
        telemetry.addLine("Will home using analog, then hold target with PIDF on digital ticks.");
        telemetry.update();
        runtime.reset();
    }

    @Override
    public void loop() {
        GamepadState current = new GamepadState(gamepad1);
        handleGainAdjustments(current, previous);
        handleTargetAdjustments(current, previous);

        if (!homed) {
            // Optionally allow skipping homing with B
            if (current.b && !previous.b) {
                homed = true; // skip homing (digital encoder zero stays where it is)
            } else {
                homed = homeWithAnalog();
            }
        }

        // After homing, run PIDF on the digital encoder
        if (homed) {
            runPositionPidf();
        } else {
            servo.setPower(0);
        }

        // Telemetry
        int pos = encoder.getCurrentPosition();
        double voltage = analog.getVoltage();
        double angleDeg = (voltage / 3.3) * 360.0;

        telemetry.addLine("=== Magazine PIDF Tuner ===");
        telemetry.addData("Homed", homed);
        telemetry.addData("Analog V", "%.3f", voltage);
        telemetry.addData("Servo Power", servo.getPower());
        telemetry.addData("Analog Angle", "%.1f deg", angleDeg);
        telemetry.addData("Encoder Pos", pos);
        telemetry.addData("Target Pos", targetPos);

        telemetry.addLine("--- Gains ---");
        telemetry.addData("kP", "%.6f", kP);
        telemetry.addData("kI", "%.10f", kI);
        telemetry.addData("kD", "%.6f", kD);
        telemetry.addData("kF", "%.6f", kF);

        telemetry.addLine("--- Controls ---");
        telemetry.addLine("A/Y: kP +/-");
        telemetry.addLine("X/B: kD +/-");
        telemetry.addLine("LB/RB: kI +/-");
        telemetry.addLine("D-pad up/down: target +/- 100 ticks");
        telemetry.addLine("D-pad right: target = 1400");
        telemetry.addLine("D-pad left: target = 0");
        telemetry.addLine("B (before homed): skip homing");
        telemetry.update();

        previous = current;
    }

    // === HOMING USING ANALOG ABSOLUTE ENCODER ===
    private boolean homeWithAnalog() {
        dt = runtime.seconds();

        double voltage = analog.getVoltage();
        double angleDeg = (voltage / 3.3) * 360.0;

        double errorDeg = homeTargetDeg - angleDeg;

        homeIntegral += errorDeg * dt;
        double derivative = (errorDeg - homeLastError) / dt;
        homeLastError = errorDeg;

        double pidOutput = homeKp * errorDeg
                + homeKi * homeIntegral
                + homeKd * derivative;

        double power = Range.clip(pidOutput, -maxHomePower, maxHomePower);
        servo.setPower(power);

        if (Math.abs(errorDeg) < homeToleranceDeg) {
            // Considered homed
            servo.setPower(0);

            encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            targetPos = 0;
            integral = 0.0;
            lastError = 0.0;

            return true;
        }

        return false;
    }

    // === POSITION PIDF USING DIGITAL ENCODER ===
    private void runPositionPidf() {
        dt = runtime.seconds();

        int currentPos = encoder.getCurrentPosition();
        double error = targetPos - currentPos;

        integral += error * dt;
        double derivative = (error - lastError) / dt;
        lastError = error;

        double pd = kP * error + kD * derivative;

        int I_ZONE = 400;
        double I_MAX = 1.0;

        boolean inIZone = Math.abs(error) < I_ZONE;
        boolean notSaturated = Math.abs(pd) < 0.9;

        if (inIZone && notSaturated)
        {
            integral += error * dt;
            integral = Range.clip(integral, -I_MAX, I_MAX);
        } else {
            integral = 0.0;
        }

        double output = pd + kI * integral;
        double power = Range.clip(output, -1.0, 1.0);
        servo.setPower(power);
    }

    // === GAIN ADJUSTMENTS (PER BUTTON PRESS) ===
    private void handleGainAdjustments(GamepadState current, GamepadState prev) {
        double stepP = 0.00001;
        double stepI = 0.000000001;
        double stepD = 0.000001;
        double stepF = 0.000001;

        // kP: A increases, Y decreases
        if (current.a && !prev.a) {
            kP += stepP;
        }
        if (current.y && !prev.y) {
            kP = Math.max(0, kP - stepP);
        }

        // kD: X increases, B decreases (only after homed; before homed B is used to skip)
        if (current.x && !prev.x) {
            kD += stepD;
        }
        if (current.b && !prev.b && homed) {
            kD = Math.max(0, kD - stepD);
        }

        // kI: LB increases, RB decreases
        if (current.left_bumper && !prev.left_bumper) {
            kI += stepI;
        }
        if (current.right_bumper && !prev.right_bumper) {
            kI = Math.max(0, kI - stepI);
        }

        // Optional: adjust kF with stick click
        if (current.left_stick_button && !prev.left_stick_button) {
            kF += stepF;
        }
        if (current.right_stick_button && !prev.right_stick_button) {
            kF = Math.max(0, kF - stepF);
        }
    }

    // === TARGET POSITION ADJUSTMENTS ===
    private void handleTargetAdjustments(GamepadState current, GamepadState prev) {
        int coarseStep = 100;

        if (current.dpad_up && !prev.dpad_up) {
            targetPos += coarseStep;
        }
        if (current.dpad_down && !prev.dpad_down) {
            targetPos -= coarseStep;
        }

        if (current.dpad_right && !prev.dpad_right) {
            targetPos += 1320;// your "up one" reference
            integral = 0;
            lastError = 0;
        }
        if (current.dpad_left && !prev.dpad_left) {
            targetPos = 0;
            integral = 0;
            lastError = 0;
        }
    }

    // Helper to snapshot gamepad state for edge detection
    private static class GamepadState {
        boolean a, b, x, y;
        boolean dpad_up, dpad_down, dpad_left, dpad_right;
        boolean left_bumper, right_bumper;
        boolean left_stick_button, right_stick_button;

        GamepadState() {}

        GamepadState(com.qualcomm.robotcore.hardware.Gamepad g) {
            if (g == null) return;
            a = g.a; b = g.b; x = g.x; y = g.y;
            dpad_up = g.dpad_up;
            dpad_down = g.dpad_down;
            dpad_left = g.dpad_left;
            dpad_right = g.dpad_right;
            left_bumper = g.left_bumper;
            right_bumper = g.right_bumper;
            left_stick_button = g.left_stick_button;
            right_stick_button = g.right_stick_button;
        }
    }
}