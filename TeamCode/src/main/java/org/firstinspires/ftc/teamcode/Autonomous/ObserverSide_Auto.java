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
        //Gripper closed state
        gripper.setGripperClosed();
        //Gripper holder to the side
        gripper.setGripperHolderParallel();
        gripper.setAnglerSide();


        // Define starting position
        Pose2d startPos = new Pose2d(8, 53, Math.toRadians(0));
        drive.setPoseEstimate(startPos);

        Pose2d SpecimenDropoffPos = new Pose2d(37, 64, Math.toRadians(0));
        Pose2d SamplePickUpPos1 = new Pose2d(29.3, 22, Math.toRadians(0));
        Pose2d SpecimenDropoffPos2 = new Pose2d(37, 61, Math.toRadians(0));

        // Define the trajectory sequence for the Observer side
        TrajectorySequence StageRedObserver = drive.trajectorySequenceBuilder(startPos)

                // Step 1: Set the gripper and arm in the right position for Specimen drop off
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {armControl.setDesArmPosDeg(74);})
                .UNSTABLE_addTemporalMarkerOffset(0.2,()->{gripper.setAnglerForward();})
                .UNSTABLE_addTemporalMarkerOffset(0.2,()->{gripper.setGripperHolderPerpendicular();})
                .waitSeconds(0.5)

                // Step 2: Move the robot to the Specimen drop off position and move forward,
                // then set the Arm down to prepare for placing the Specimen
                .lineToLinearHeading(SpecimenDropoffPos)
                .forward(3.5)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {armControl.setDesArmPosDeg(40);})
                .waitSeconds(0.2)

                // Step 3: Move backward and open the Gripper to place and release the Specimen.
                // At the same time, drop the arm all the way down and set its power to zero
                // afterwards
                .back(7.5)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> gripper.setGripperOpen())
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> armControl.setDesArmPosDeg(-20))
                .waitSeconds(0.4)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> armControl.setArmPower(0))


                // Step 4: Move to Sample 1 and extend the slide to pick up the Sample 1
                .lineToLinearHeading(SamplePickUpPos1)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> sliderControl.setDesSliderLen(6))
                .UNSTABLE_addTemporalMarkerOffset(1.5, () -> gripper.setGripperClosed())
                .UNSTABLE_addTemporalMarkerOffset(1.7, () -> sliderControl.setDesSliderLen(3))
                .waitSeconds(2)

                //Step 5: Turn right 135 degree to drop off Sample 1
                .turn(-Math.toRadians(135))
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> gripper.setGripperOpen())


                // Step 6: Turn back and strafe right 12 inch to pick up Sample 2
                .turn(Math.toRadians(135))
                .strafeRight(12)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> sliderControl.setDesSliderLen(6))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> gripper.setGripperClosed())
                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> sliderControl.setDesSliderLen(3))
                .waitSeconds(1)

                // Step 7: Turn right 135 degree to drop off Sample 2
                .turn(-Math.toRadians(135))
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> gripper.setGripperOpen())
                .waitSeconds(0.5)
                // Step 8: Pick up Specimen 2
                .back(6)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> sliderControl.setDesSliderLen(6))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> gripper.setGripperClosed())
                .waitSeconds(1.0)
                // Step 9: Turn left 135 degrees and raise the Arm to prepare for
                // Specimen 2 drop off attempt
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {armControl.setDesArmPosDeg(74);})
                .turn(Math.toRadians(135))

                // Step 10: Specimen 2 drop off attempt
                .lineToLinearHeading(SpecimenDropoffPos2)
                .forward(3.5)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {armControl.setDesArmPosDeg(40);})
                .waitSeconds(0.2)

                // Step 11: Move backward and open the Gripper to place and release the Specimen.
                // At the same time, drop the arm all the way down and set its power to zero
                // afterwards
                .back(7.5)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> gripper.setGripperOpen())
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> armControl.setDesArmPosDeg(-20))
                .waitSeconds(0.4)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> armControl.setArmPower(0))

                // Step 12: Strafe right to park
                .strafeRight(41)
                .back(3)
                .waitSeconds(2)

                // Final build for this trajectory
                .build();

        // Wait for start signal
        waitForStart();

        // Execute the trajectory sequence
        drive.followTrajectorySequence(StageRedObserver);



    }
}