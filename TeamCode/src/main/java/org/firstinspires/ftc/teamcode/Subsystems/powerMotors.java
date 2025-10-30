package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class powerMotors
{
    public static GoBildaPinpointDriver odo; //Odo pod

    public static DcMotor frontLeftDrive; //Front left driving motor
    public static DcMotor backLeftDrive; //Back left driving motor
    public static DcMotor frontRightDrive; //Front right driving motor
    public static DcMotor backRightDrive; //Back right driving moto

    public static DcMotor intake; //Intake motor
    public static DcMotor launcher; //Launcher motor

    public static boolean intakeRunning; //Check for the intake running

    public static void init(HardwareMap hwMap)
    {
        frontLeftDrive = hwMap.get(DcMotor.class, "FLD"); //Init frontLeftDrive
        frontRightDrive = hwMap.get(DcMotor.class, "FRD"); //Init frontRightDrive
        backLeftDrive = hwMap.get(DcMotor.class, "BLD"); //Init backLeftDrive
        backRightDrive = hwMap.get(DcMotor.class, "BRD"); //Init backRightDrive
        intake = hwMap.get(DcMotor.class, "IN"); //Init intake
        launcher = hwMap.get(DcMotor.class, "LA"); //Init launcher

        odo = hwMap.get(GoBildaPinpointDriver.class, "odo"); //Init odometry

        //Set motor directions for expected drive and intake/launch function
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        launcher.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        //Initialization for odometry
        odo.setOffsets(2.3622047244, -6.6141732283, DistanceUnit.INCH);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );
        odo.resetPosAndIMU();

        //Reset vars
        intakeRunning = true;

        //Start intake and launcher
        intakeOff(false);
        //launcher.setPower(1);
    }


    //Basic drive script without odometry or field centric drive
    public static void basicDrive(double x, double y, double rx)
    {
        double max;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each basicDrive wheel to save the power level for telemetry.
        double frontLeftPower  = y + x + rx;
        double frontRightPower = y - x - rx;
        double backLeftPower   = y - x + rx;
        double backRightPower  = y + x - rx;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower  /= max;
            frontRightPower /= max;
            backLeftPower   /= max;
            backRightPower  /= max;
        }

        // This is test code:
        //
        // Uncomment the following code to test your motor directions.
        // Each button should make the corresponding motor run FORWARD.
        //   1) First get all the motors to take to correct positions on the robot
        //      by adjusting your Robot Configuration if necessary.
        //   2) Then make sure they run in the correct direction by modifying the
        //      the setDirection() calls above.
        // Once the correct motors move in the correct direction re-comment this code.

            /*
            frontLeftPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            backLeftPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            frontRightPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            backRightPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

        frontLeftPower = frontLeftPower * 0.5;
        frontRightPower = frontRightPower * 0.5;
        backLeftPower = backLeftPower * 0.5;
        backRightPower = backRightPower * 0.5;

        // Send calculated power to wheels
        frontLeftDrive.setPower(frontLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backLeftDrive.setPower(backLeftPower);
        backRightDrive.setPower(backRightPower);
    }

    //Basic drive script using odometry for field centric drive
    public static void odoDrive(double x, double y, double rx)
    {
        odo.update();
        double heading = -odo.getHeading(AngleUnit.RADIANS);

        double xSpeed = x * Math.cos(heading) - y * Math.sin(heading);
        double ySpeed = x * Math.sin(heading) + y * Math.cos(heading);

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (ySpeed + xSpeed + rx) / denominator;
        double backLeftPower = (ySpeed - xSpeed + rx) / denominator;
        double frontRightPower = (ySpeed - xSpeed - rx) / denominator;
        double backRightPower = (ySpeed + xSpeed - rx) / denominator;

        frontLeftDrive.setPower(frontLeftPower);
        backLeftDrive.setPower(backLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backRightDrive.setPower(backRightPower);
    }


    //Allows disable or enable of intake
    public static void intakeOff(boolean stop)
    {
        if (stop)
        {
            intake.setPower(0); //Turn off intake
            intakeRunning = false; //Change variable to reflect change
        }
        else
        {
            intake.setPower(1); //Turn on intake
            intakeRunning = true; //Change variable to reflect change//Change variable to reflect change
        }
    }



}
