package org.firstinspires.ftc.teamcode.newprograms;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;


@TeleOp(name="SKYSTONE Vuforia Nav", group ="Concept")
@Disabled
public class TestVuforiaTest extends LinearOpMode {

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false  ;

    private static final String VUFORIA_KEY =
            "AaP8TtL/////AAABmdQCMtnFIU3jr122XQx7bytjuKILg7/gbxc26uQ0q259PacMleX7j4QyGQqIQto8rld" +
                    "uCWLnt/RN71XGH3LlBYm8j4LhkhrE02qlhMXsyQ/DlBS6NLkPZGuT/568zeyS4Rm/Kl1esOqSWcs45n" +
                    "DowCjUMsF8WUGWk02rWPvyNYh6u7hAMCDmo7PNxCm3Hz11kVWH3ny9GsnUA6E59aieAgUXzeUWhB/" +
                    "H7hAkHhUQ1RPvHxxGjn+bl/sqOiP2qJOYIHKf3P2lg1S5CPSFAZ4Ok47GgVQ/v1MYXXTvSJQ+Ob3495" +
                    "tqXpNv54/ZOZnURVjanaqh3Os0u002ue3aOQ58RojtHa4IBtLepJrc8tm4SE9Z";

    @Override public void runOpMode()
    {

    }
}
