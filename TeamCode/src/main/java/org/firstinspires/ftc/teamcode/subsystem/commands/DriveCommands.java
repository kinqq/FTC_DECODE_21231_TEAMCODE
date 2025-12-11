package org.firstinspires.ftc.teamcode.subsystem.commands;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import dev.nextftc.core.commands.Command;

public class DriveCommands {
    public static GoBildaPinpointDriver odo; //Odo pod

    public static DcMotor frontLeftDrive; //Front left driving motor
    public static DcMotor backLeftDrive; //Back left driving motor
    public static DcMotor frontRightDrive; //Front right driving motor
    public static DcMotor backRightDrive;
    public static DcMotorEx intake; //Intake motor



    public DriveCommands(HardwareMap hwMap) {
        frontLeftDrive = hwMap.get(DcMotor.class, "leftFront"); //Init frontLeftDrive
        frontRightDrive = hwMap.get(DcMotor.class, "rightFront"); //Init frontRightDrive
        backLeftDrive = hwMap.get(DcMotor.class, "leftBack"); //Init backLeftDrive
        backRightDrive = hwMap.get(DcMotor.class, "rightBack"); //Init backRightDrive
        intake = hwMap.get(DcMotorEx.class, "intake"); //Init intake

        odo = hwMap.get(GoBildaPinpointDriver.class, "odo"); //Init odometry

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        //Initialization for odometry
        odo.setOffsets(-48, -182.5, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );
        odo.resetPosAndIMU();

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void basicDrive(double x, double y, double rx)
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

        // Send calculated power to wheels
        frontLeftDrive.setPower(frontLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backLeftDrive.setPower(backLeftPower);
        backRightDrive.setPower(backRightPower);

        odo.update();
    }

    //Basic drive script using odometry for field centric drive
    public void odoDrive(double x, double y, double rx)
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

    public class intakeOff extends CommandBase
    {
        @Override
        public void initialize() {
            intake.setPower(0);
        }

        public boolean isFinished() {return true;}
    }

    public class intakeOn extends CommandBase
    {
        @Override
        public void initialize(){
            intake.setPower(1);
        }

        public boolean isFinished() {return true;}
    }

    public class intakeIdle extends CommandBase {
        @Override
        public void initialize() {
            intake.setPower(0.3);
        }

        @Override
        public boolean isFinished() {return true;}
    }

    public GoBildaPinpointDriver getOdo() {
        return odo;
    }

    public void setOdo(double x, double y) {
        odo.setPosX(x, DistanceUnit.MM);
        odo.setPosY(y, DistanceUnit.MM);
    }

}
