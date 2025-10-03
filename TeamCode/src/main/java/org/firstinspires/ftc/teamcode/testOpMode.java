package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp(name="TeleOp", group="Linear OpMode")
public class testOpMode extends LinearOpMode{

    private ElapsedTime runtime = new ElapsedTime();

    private IMU imu;

    private DcMotor FLDrive, BLDrive, FRDrive, BRDrive = null;
    private DcMotor LAL, LAR = null;
    private DcMotor INL, INR = null;
    
    private Servo MG = null;

    private RevColorSensorV3 bob = null;

    @Override
    public void runOpMode()
    {
        sub magazine = new sub(this);

        initialize();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive())
        {
            drive();
            gameLoop();
        }
    }

    private void initialize()
    {
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        //DT Init
        FLDrive = hardwareMap.get(DcMotor.class, "FLD");
        FRDrive = hardwareMap.get(DcMotor.class, "FRD");
        BLDrive = hardwareMap.get(DcMotor.class, "BLD");
        BRDrive = hardwareMap.get(DcMotor.class, "BRD");
       
        //Launcher Init
        LAL = hardwareMap.get(DcMotor.class, "LAL");
        LAR = hardwareMap.get(DcMotor.class, "LAR");

        //Intake Init
        INR = hardwareMap.get(DcMotor.class, "INR");
        INL = hardwareMap.get(DcMotor.class, "INL");

        //Turntable Init
        MG = hardwareMap.get(Servo.class, "MG");

        //Color Sensor Init
        bob = hardwareMap.get(RevColorSensorV3.class, "bob");

        //DT Directions
        FLDrive.setDirection(DcMotor.Direction.REVERSE);
        BLDrive.setDirection(DcMotor.Direction.REVERSE);
        FRDrive.setDirection(DcMotor.Direction.FORWARD);
        BRDrive.setDirection(DcMotor.Direction.FORWARD);

        //LA Directions
        LAR.setDirection(DcMotor.Direction.FORWARD);
        LAL.setDirection(DcMotor.Direction.FORWARD);

        //IN Directions
        INR.setDirection(DcMotor.Direction.FORWARD);
        INL.setDirection(DcMotor.Direction.FORWARD);

        //Send Init Signal to DH
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    private void drive()
    {
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        // This button choice was made so that it is hard to hit on accident,
        // it can be freely changed based on preference.
        // The equivalent button is start on Xbox-style controllers.
        if (gamepad1.options) {
            imu.resetYaw();
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        FLDrive.setPower(frontLeftPower);
        BLDrive.setPower(backLeftPower);
        FRDrive.setPower(frontRightPower);
        BRDrive.setPower(backRightPower);
    }

    private void gameLoop()
    {
        boolean[] magazine = new boolean[3];
        boolean mgFull = false;
        int mgActive = 0;


        //IF COLOR SENSOR SEES GREEN
            //magazine[mgActive] = true;
           //mgActive = mgActive + 1 % 3;
    }
}
