package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.subsytems.ArmControl;
import org.firstinspires.ftc.teamcode.subsytems.Gripper;
import org.firstinspires.ftc.teamcode.subsytems.SliderControl;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class ObserverSide_Auto extends LinearOpMode {

    private ElapsedTime AutoTimer = new ElapsedTime();
    private ArmControl armControl;
    private Gripper gripper;
    private SliderControl sliderControl;
    private double speedFactor = 0.65;


    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize Mecanum Drive
        MecanumDriveBase drive = new MecanumDriveBase(hardwareMap);
        drive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // - - - Setting up Arm motors - - - //
        armControl = new ArmControl(this);
        armControl.init(hardwareMap);

        // - - - Setting up Slider motors - - - //
        sliderControl = new SliderControl(this);
        sliderControl.init(hardwareMap);

        // - - - Initialize gripper to starting position - - - //
        gripper = new Gripper(this);
        gripper.init(hardwareMap);
        gripper.gripperStopped();
        gripper.setAnglerInit();


        // Initialize telemetry
        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Define starting position
        Pose2d startPos = new Pose2d(8, 53, Math.toRadians(0));
        drive.setPoseEstimate(startPos);

        Pose2d SpecimenDropoffPos = new Pose2d(33, 66, Math.toRadians(0));

        // Define the trajectory sequence for the Observer side
        TrajectorySequence StageRedObserver = drive.trajectorySequenceBuilder(startPos)
                // Step 1: Set Arm to the Right Angle with the Gripper parallel to ground.
                // At the same time Extend slider length to 1 inch and wait for 0.5 second in the end
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {armControl.setDesArmPosDeg(58);})
                .UNSTABLE_addTemporalMarkerOffset(0.5,()->{gripper.setAnglerUP();})
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {sliderControl.setDesSliderLen(1);})
                .waitSeconds(0.5)

                // Step 2: Move the robot to the Specimen drop off position and set the gripper to
                // be parallel to the side for drop off operation and wait for 0.2 second in the end
                .lineToLinearHeading(SpecimenDropoffPos)
                .UNSTABLE_addTemporalMarkerOffset(0.2,()->{gripper.setAnglerDown();})
                .waitSeconds(0.2)

                // Step 3: Move forward 6 inch to prepare for specimen drop off, set arm angle down to 45 deg
                // and set Gripper to be rolling in to hold the specimen and wait for 0.1 second in the end
                .forward(7.5)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {armControl.setDesArmPosDeg(45);})
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> gripper.gripperForward(0.3))
                .waitSeconds(0.1)

                // Step 4: Stop the gripper after 0.4 second and move the robot backward 11 inch
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> gripper.gripperForward(0.3))
                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> {gripper.gripperStopped();})
                .back(1)
                .waitSeconds(1)

                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> gripper.gripperReverse(-0.3))
                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> {gripper.gripperStopped();})
                .waitSeconds(0.5)

                // Step 5: Rollback the slider and set gripper parallel to the ground and strafe to the right
                // for 33 in with the arm power off. Then move forward 28 inch then straft right 12 inch
                // for the first sample push back preparation
                .strafeRight(33)

                .UNSTABLE_addTemporalMarkerOffset(0.2,()->{gripper.setAnglerUP();})
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {sliderControl.setDesSliderLen(0);})
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {armControl.setArmPower(0);})
                .forward(28)

                // Step 6: Strafe right for 12 inch and back 43 inch for pushing the first sampel to the
                // observer zone
                .strafeRight(12)
                .back(43)

                // Step 7: Move forward for 43 inch and then strafe to the right for 9 inch then
                // back for 43 inch to push the second sample to the observe zone
                .forward(43)
                .strafeRight(9)
                .back(43)

                // Final build for this trajectory
                .build();

        // Wait for start signal
        waitForStart();

        // Execute the trajectory sequence
        drive.followTrajectorySequence(StageRedObserver);



    }
}