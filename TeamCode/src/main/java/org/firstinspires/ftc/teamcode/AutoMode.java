package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Scorpion: AutoMode", group="DarkMatter2019")
//@Disabled
public class AutoMode extends LinearOpMode {

    /* Declare OpMode members. */
    DriveTrain scorpion = new DriveTrain();
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 7 ;    // Neverest 20
    static final double     DRIVE_GEAR_REDUCTION    = 19.2 * 72 / 48;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (4 * COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.4;
    static final double     TURN_SPEED              = 0.3;

    /**
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {

        // Creating new targets for each DcMotor
        int newLeftFrontTarget;
        int newLeftRearTarget;
        int newRightFrontTarget;
        int newRightRearTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = scorpion.leftFront.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newLeftRearTarget = scorpion.leftRear.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightFrontTarget = scorpion.rightFront.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newRightRearTarget = scorpion.rightRear.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

            while (scorpion.leftFront.getTargetPosition() != newLeftFrontTarget) {
                scorpion.leftFront.setTargetPosition(newLeftFrontTarget);
                sleep(1);
            }
            while (scorpion.leftRear.getTargetPosition() != newLeftRearTarget) {
                scorpion.leftRear.setTargetPosition(newLeftRearTarget);
                sleep(1);
            }
            while (scorpion.rightFront.getTargetPosition() != newRightFrontTarget) {
                scorpion.rightFront.setTargetPosition(newRightFrontTarget);
                sleep(1);
            }
            while (scorpion.rightRear.getTargetPosition() != newRightRearTarget) {
                scorpion.rightRear.setTargetPosition(newRightRearTarget);
                sleep(1);
            }

            // Turn On RUN_TO_POSITION
            scorpion.setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            // Setting power to DcMotors and make sure the speed it positive with Math.abs
            scorpion.setPowerLevel(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    scorpion.leftFront.isBusy() &&
                    scorpion.leftRear.isBusy() &&
                    scorpion.rightFront.isBusy() &&
                    scorpion.rightRear.isBusy()) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d :%7d :%7d",
                        newLeftFrontTarget,
                        newLeftRearTarget,
                        newRightFrontTarget,
                        newRightRearTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d",
                        scorpion.leftFront.getCurrentPosition(),
                        scorpion.leftRear.getCurrentPosition(),
                        scorpion.rightFront.getCurrentPosition(),
                        scorpion.rightRear.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            scorpion.setPowerLevel(0);

            // Turn off RUN_TO_POSITION
            scorpion.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Pause after each move
            sleep(250);
        }
    }


    @Override
    public void runOpMode() {

        scorpion.init(hardwareMap);

        //setting motors to use Encoders
        scorpion.setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        scorpion.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
        encoderDrive(DRIVE_SPEED, 40, 40, 3.0);
        encoderDrive(TURN_SPEED, 12, -12, 2.0);
        encoderDrive(DRIVE_SPEED, 20, 20, 2.0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

}
