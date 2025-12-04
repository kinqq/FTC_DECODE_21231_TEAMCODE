package org.firstinspires.ftc.teamcode.Subsystems.Old;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.constant.ConstantsPIDF;
import org.firstinspires.ftc.teamcode.constant.ConstantsServo;

public class turret {
    public DcMotorEx turretMotor, launchMotor;
    private ServoImplEx launchAngle;
    private PIDController pid;

    // mechanical parameters
    private static final double MOTOR_TO_TURRET_GEAR_RATIO = 5.6111111111;
    private static final double TICKS_PER_REV = 537.7; // e.test. GoBilda 5202-0002-0007 motor (edit if different)

    // state
    private double targetTurretDeg = 0.0;
    private double offset = 0.0;
    private double lastCmd = 0.0;
    private double angle = 0.0;

    private final ElapsedTime loopTimer = new ElapsedTime();

    public turret(HardwareMap hwMap) {
        turretMotor = hwMap.get(DcMotorEx.class, "turret");
        launchMotor = hwMap.get(DcMotorEx.class, "launcher");
        launchAngle = hwMap.get(ServoImplEx.class, "launchAngle");

        launchAngle.setPwmRange(new PwmControl.PwmRange(500, 2500));

        pid = new PIDController(ConstantsPIDF.p, ConstantsPIDF.i, ConstantsPIDF.d);

        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setPower(0);

        launchMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        launchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launchMotor.setPower(0);

        targetTurretDeg = getAngleDeg();
        loopTimer.reset();
    }

    /** Main control loop */
    public void update() {
        pid.setPID(ConstantsServo.kP, ConstantsServo.kI, ConstantsServo.kD);

        double measuredMotorDeg = getMotorAngleDeg();
        double measuredMotorTick = measuredMotorDeg * TICKS_PER_REV / 360.0;
        double targetMotorDeg = (targetTurretDeg + offset) * MOTOR_TO_TURRET_GEAR_RATIO;
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
        targetTurretDeg = Range.clip(turretDeg, -135, 135);
    }

    /** Adjust target relative to current value */
    public void adjustTarget(double deltaTurretDeg) {
        offset += deltaTurretDeg;
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

    public void activateLauncher() {
        this.launchMotor.setPower(1.0);
        this.launchMotor.setVelocity(1900);
    }

    public void deactivateLauncher() {
        this.launchMotor.setPower(0.0);
        this.launchMotor.setVelocity(0, AngleUnit.DEGREES);
    }

    public void toggleLauncher() {
        if (launchMotor.getPower() == 1.0) deactivateLauncher();
        else activateLauncher();
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
