package org.firstinspires.ftc.teamcode.teleop;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.*;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import java.util.concurrent.TimeUnit;


@TeleOp(name = "VERYBasicDrive")
public class testTeleop extends OpMode {

    public static ElapsedTime runtime = new ElapsedTime();
    public float timer = 2;

    public boolean pressed1 = false;
    public boolean pressed2 = false;

    @Override
    public void init()
    {
        //driveMotors.init(hardwareMap); //Initialize the drive subsystem
        //magazine.init(hardwareMap); //Initialize magazine subsystem
        colorSensor.init(hardwareMap);
        runtime.reset();
    }

    @Override
    public void loop() {
        /*
        //Send stick values to drive for odo drive
        driveMotors.odoDrive(
                gamepad1.left_stick_x,
                -gamepad1.left_stick_y,
                gamepad1.right_stick_x
        );

        //Control intake manually
        if (gamepad2.a) driveMotors.intakeOff(false);
        if (gamepad2.b) driveMotors.intakeOff(true);

        //Wait to check till 0.5 seconds pass
        if (runtime.now(TimeUnit.SECONDS) - timer >= 2) {
            magazine.checkColors(); //Check and update all magazine slots
            timer = runtime.now(TimeUnit.SECONDS); //Reset timer to now to reflect new countdown
        }

        if (gamepad2.leftBumperWasPressed()) magazine.find(0);//Find a purple and put in launch position
        if (gamepad2.rightBumperWasPressed()) magazine.find(1); //Find a green and put in launch position

        magazine.updatePosition(); //Set servo to the servo position variable

        telemetry.addLine("Current Magazine Contents:");
        telemetry.addData(" Active Slot: ", magazine.MGAr[magazine.activeMG]);
        telemetry.addData(" Top Left: ", magazine.MGAr[(magazine.activeMG + 2) % 3]);
        telemetry.addData(" Top Right: ", magazine.MGAr[(magazine.activeMG + 1) % 3]);
        telemetry.addLine();

        telemetry.addData("Servo Position (Reported): ", magazine.servoPosition);
        telemetry.addData("Servo Position (Real): ", magazine.magazine.getPosition());

        telemetry.addLine("Color Sensors");
        telemetry.addLine("Bob (active slot):");
        telemetry.addData(" Red: ", magazine.bob.red());
        telemetry.addData(" Green: ", magazine.bob.green());
        telemetry.addData(" Blue: ", magazine.bob.blue());

        telemetry.addLine("Gary (top left):");
        telemetry.addData(" Red: ", magazine.gary.red());
        telemetry.addData(" Green: ", magazine.gary.green());
        telemetry.addData(" Blue: ", magazine.gary.blue());

        telemetry.addLine("Joe (top right): ");
        telemetry.addData(" Red: ", magazine.joe.red());
        telemetry.addData(" Green: ", magazine.joe.green());
        telemetry.addData(" Blue: ", magazine.joe.blue());

        telemetry.update();

         */

        colorSensor.bobDetect(telemetry);
        telemetry.addData("Detected", colorSensor.bobDetect(telemetry));
        telemetry.update();
    }
}
