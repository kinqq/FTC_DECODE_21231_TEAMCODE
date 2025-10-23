package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.ConstantsPIDF;
import org.firstinspires.ftc.teamcode.util.ConstantsServo;

public class Turret {
    private DcMotorEx turretMotor;
    private ServoImplEx launchAngle;
    private PIDController pid;

    // mechanical parameters
    private static final double MOTOR_TO_TURRET_GEAR_RATIO = 5.6111111111;
    private static final double TICKS_PER_REV = 537.7; // e.g. GoBilda 5202-0002-0007 motor (edit if different)

    // state
    private double targetTurretDeg = 0.0;
    private double lastCmd = 0.0;
    private double angle = 0.0;

    private final ElapsedTime loopTimer = new ElapsedTime();

    public Turret(HardwareMap hwMap) {
        turretMotor = hwMap.get(DcMotorEx.class, "turret");
        launchAngle = hwMap.get(ServoImplEx.class, "launchAngle");

        launchAngle.setPwmRange(new PwmControl.PwmRange(500, 2500));

        pid = new PIDController(ConstantsPIDF.p, ConstantsPIDF.i, ConstantsPIDF.d);

        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setPower(0);

        targetTurretDeg = getAngleDeg();
        loopTimer.reset();
    }

    /** Main control loop */
    public void update() {
        pid.setPID(ConstantsServo.kP, ConstantsServo.kI, ConstantsServo.kD);

        double measuredMotorDeg = getMotorAngleDeg();
        double measuredMotorTick = measuredMotorDeg * TICKS_PER_REV / 360.0;
        double targetMotorDeg = targetTurretDeg * MOTOR_TO_TURRET_GEAR_RATIO;
        double targetMotorTick = targetMotorDeg * TICKS_PER_REV / 360.0;

        double pidOut = pid.calculate(measuredMotorTick, targetMotorTick);

        lastCmd = Range.clip(pidOut, -1.0, 1.0);
        turretMotor.setPower(lastCmd);

        double anglePos = -0.0038 * angle + 0.871;
        anglePos = Range.clip(anglePos, 0.0, 1.0);
        launchAngle.setPosition(anglePos);
    }

    /** Convert encoder ticks → motor degrees */
    private double getMotorAngleDeg() {
        return turretMotor.getCurrentPosition() * (360.0 / TICKS_PER_REV);
    }

    /** Convert motor deg → turret deg */
    public double getAngleDeg() {
        return getMotorAngleDeg() / MOTOR_TO_TURRET_GEAR_RATIO;
    }

    /** Set absolute turret angle (degrees) */
    public void setTarget(double turretDeg) {
        targetTurretDeg = turretDeg;
    }

    /** Adjust target relative to current value */
    public void adjustTarget(double deltaTurretDeg) {
        targetTurretDeg += deltaTurretDeg;
    }

    /** Reset encoder, zero turret */
    public void zeroHere() {
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        targetTurretDeg = 0.0;
        pid.reset();
    }

    /** Launch angle control (20–50 deg) */
    public void setLaunchAngle(double angle) {
        this.angle = Range.clip(angle, 20, 50);
    }

    // telemetry helpers
    public double getTargetDeg() { return targetTurretDeg; }
    public double getCommand() { return lastCmd; }
    public double getErrorDeg() { return targetTurretDeg - getAngleDeg(); }
    public double getLaunchAngle() { return angle; }

    public void stop() {
        turretMotor.setPower(0);
    }
}
