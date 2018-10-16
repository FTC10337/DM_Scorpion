package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

//import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

@Autonomous(name="Scorpion: AutoMode", group="DarkMatter2019")
//@Disabled
public class AutoMode extends LinearOpMode {

    /* Declare OpMode members. */
    DriveTrain scorpion = new DriveTrain();
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 7 ;    // Neverest 20
    static final double     DRIVE_GEAR_REDUCTION    = 19.2 * 72 / 48  ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (4 * COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() {

        scorpion.init(hardwareMap);

        //setting motors to use Encoders
        scorpion.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        scorpion.leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        scorpion.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        scorpion.rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        scorpion.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        scorpion.leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        scorpion.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        scorpion.rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d :%7d :%7d",
                scorpion.leftFront.getCurrentPosition(),
                scorpion.leftRear.getCurrentPosition(),
                scorpion.rightFront.getCurrentPosition(),
                scorpion.rightRear.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        encoderDrive(DRIVE_SPEED,  -73,  -73, 8.0);  // S1: Forward 73 Inches with 8 Sec timeout
        sleep(300);
        encoderDrive(TURN_SPEED,   -12, 12, 2);  // S2: Turn Right 6 Inches with 2.5 Sec timeout
        encoderDrive(DRIVE_SPEED, -86, -86, 10.0);  // S3: Forward 86 Inches with 10 Sec timeout

//        robot.leftClaw.setPosition(1.0);            // S4: Stop and close the claw.
//        robot.rightClaw.setPosition(0.0);
//        sleep(1000);     // pause for servos to move

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches,
                             double rightInches,
                             double timeoutS) {

        int newLeftTargetF;
        int newLeftTargetR;
        int newRightTargetF;
        int newRightTargetR;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTargetF = scorpion.leftFront.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newLeftTargetR = scorpion.leftRear.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTargetF = scorpion.rightFront.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newRightTargetR = scorpion.rightRear.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            scorpion.leftFront.setTargetPosition(newLeftTargetF);
            scorpion.leftRear.setTargetPosition(newLeftTargetR);
            scorpion.rightFront.setTargetPosition(newRightTargetF);
            scorpion.rightRear.setTargetPosition(newRightTargetR);

            // Turn On RUN_TO_POSITION
            scorpion.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            scorpion.leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            scorpion.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            scorpion.rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            scorpion.leftFront.setPower(Math.abs(speed));
            scorpion.leftRear.setPower(Math.abs(speed));
            scorpion.rightFront.setPower(Math.abs(speed));
            scorpion.rightRear.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (scorpion.leftFront.isBusy() && scorpion.leftRear.isBusy() && scorpion.rightFront.isBusy() && scorpion.rightRear.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d :%7d :%7d", newLeftTargetF, newLeftTargetR, newRightTargetF, newRightTargetR);
                telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d",
                                            scorpion.leftFront.getCurrentPosition(),
                                            scorpion.leftRear.getCurrentPosition(),
                                            scorpion.rightFront.getCurrentPosition(),
                                            scorpion.rightRear.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            scorpion.leftFront.setPower(0);
            scorpion.leftRear.setPower(0);
            scorpion.rightFront.setPower(0);
            scorpion.rightRear.setPower(0);

            // Turn off RUN_TO_POSITION
            scorpion.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            scorpion.leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            scorpion.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            scorpion.rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
}
