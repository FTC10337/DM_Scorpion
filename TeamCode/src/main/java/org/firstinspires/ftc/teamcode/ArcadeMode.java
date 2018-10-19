package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;


@TeleOp(name="Scorpion: ArcadeMode", group="DarkMatter2019")
//@Disabled
public class ArcadeMode extends OpMode
{
    DriveTrain scorpion = new DriveTrain();

    private ElapsedTime runtime = new ElapsedTime();
    boolean hyper = false;
    boolean isButtonPressed = true;
    double turnCoefficient = 4;
    double driveCoefficient = 2;

    @Override
    public void init() {

        telemetry.addData("Scorpion Says", "Hello DarkMatter!");

        scorpion.init(hardwareMap);

        // Stop all motion;
        scorpion.setPowerLevel(0);

        //setting motors to use Encoders
        //scorpion.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        RobotLog.i("Initialized, Ready to Start!");
    }

    //Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
    @Override
    public void init_loop() {}

    @Override
    public void start() { runtime.reset(); }

    //Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {

        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;

        // Arcade(POV) Mode uses left stick to go forward, and right stick to turn.
        double drive = -gamepad1.left_stick_y;
        double turn  =  gamepad1.right_stick_x;

        telemetry.addData("Path0",  "Starting at %7d :%7d :%7d :%7d",
                scorpion.leftFront.getCurrentPosition(),
                scorpion.leftRear.getCurrentPosition(),
                scorpion.rightFront.getCurrentPosition(),
                scorpion.rightRear.getCurrentPosition());
        telemetry.update();

        // Smooth and deadzone the joystick values
        drive = scorpion.smoothPowerCurve(scorpion.deadzone(drive, 0.10)) / driveCoefficient;
        turn = scorpion.smoothPowerCurve(scorpion.deadzone(turn, 0.10)) / turnCoefficient;

        leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
        rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

        // Send calculated power to wheels
        scorpion.leftFront.setPower(leftPower);
        scorpion.leftRear.setPower(leftPower);
        scorpion.rightFront.setPower(rightPower);
        scorpion.rightRear.setPower(rightPower);

        //Activating slowMo slow motion mode with controller left bumper
        if (gamepad1.left_bumper && isButtonPressed) {
            hyper = true;
            telemetry.addData("Says", "Hyper is ON");
        }else{
            hyper = false;
            telemetry.addData("Says", "Hyper is OFF");
        }

        //Setting new values if slowMo is true
        if (hyper) {
            turnCoefficient = 1;
            driveCoefficient = 1;
        }else{
            turnCoefficient = 4;
            driveCoefficient = 2;
        }

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
    }


    //Code to run ONCE after the driver hits STOP
    @Override
    public void stop() {}

}
