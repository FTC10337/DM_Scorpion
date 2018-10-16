package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbotMatrix;

@TeleOp(name="ArcadeMode", group="DarkMatter2019")
//@Disabled
public class ArcadeMode extends OpMode
{
    DriveTrain scorpion = new DriveTrain();

    private ElapsedTime runtime = new ElapsedTime();
    boolean slowMo = false;
    boolean isButtonPressed = true;
    double turnCoefficient = 1;
    double driveCoefficient = 1;

    @Override
    public void init() {

        telemetry.addData("Scorpion Says", "Hello DarkMatter!");
        scorpion.init(hardwareMap);
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        RobotLog.i("Initialized, Ready to Start!");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
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
            slowMo = true;
            telemetry.addData("Says", "SlowMo is ON");
        }else{
            slowMo = false;
            telemetry.addData("Says", "SlowMo is OFF");
        }

        //Setting new values if slowMo is true
        if (slowMo) {
            turnCoefficient = 4;
            driveCoefficient = 2;
        }else{
            turnCoefficient = 1;
            driveCoefficient = 1;
        }

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
