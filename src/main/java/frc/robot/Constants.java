package frc.robot;

public final class Constants {
    /** Constants relating to the driver input */
    public static class OperatorConstants {
        /** The driver station port to use as the driver controller port */
        public static final int driverControllerPort = 0;

        /** The deadband for the driver controller joysticks */
        public static final double driveDeadband = 0.1;
    }

    /** Constants for use during Auto */
    public static class AutoConstants {

    }

    /** Constants for vision */
    public static class VisionConstants {

    }

    /** Constants for turret */
    public static class TurretConstants {

    }

    /** Constants for intake */
    public static class IntakeConstants {

    }

    /** Constants for storage rollers */
    public static class RollerConstants {

    }

    /** Constants for climber */
    public static class ClimbConstants {

    }

    /** Constants for LEDs */
    public static class LEDConstants {
        public static final int led_pin = 0;

        /** The seconds before the hub activates where the leds do the warning lights */
        public static final double shiftAlmostNum = 3.0;

        public static final double hotPink = 0.57; //trans flag colours
        public static final double darkred = 0.59;
        public static final double red = 0.61; //resevered for errors/ warnings
        public static final double redOrange = 0.63;
        public static final double orange = 0.65;
        public static final double gold = 0.67;
        public static final double yellow = 0.69;
        public static final double lawnGreen = 0.71;
        public static final double lime = 0.73;
        public static final double darkGreen = 0.75;
        public static final double green = 0.77;
        public static final double blueGreen = 0.79;
        public static final double aqua = 0.81;
        public static final double skyBlue = 0.83;
        public static final double darkBlue = 0.85;
        public static final double blue = 0.87;
        public static final double blueViolet = 0.89;
        public static final double violet = 0.91;
        public static final double white = 0.93;
        public static final double gray = 0.95;
        public static final double darkGray = 0.97;
        public static final double black = 0.99;

        

        public static final double rainbow_rainbow = -0.99; //rainbow pattern and rainbow colours

        public static final double off = 0.99;

        public static final double warning = 0.61;

        public static final double[] warning_blink = {
        4.0, warning, off, warning, off
        };

        //pride flag presets
        public static final double[] trans_flag = {
        2.0, skyBlue, hotPink, white, hotPink, skyBlue, off, off
        };

        public static final double[] aroace_flag = {
        2.0, orange, gold, white, skyBlue, darkBlue, off, off
        };

        public static final double[] lesbian_flag = {
        2.0, redOrange, orange, white, hotPink, violet, off, off
        };
        
        public static final double[] sapphic_flag = {
        2.0, hotPink, white, hotPink, off, off
        };

        public static final double[] queer_flag = {
        2.0, red, orange, yellow, lime, blue, violet, off, off, off
        };

        public static final double[] lithrosexual_flag = {
        2.0, redOrange, orange, yellow, white, black, off, off
        };

        public static final double[] femaric_flag = {
        2.0, black, white, hotPink, off, off
        };

        public static final double[] mascic_flag = {
        2.0, black, white, skyBlue, off, off
        };

        public static final double[] genderfluid_flag = {
        2.0, hotPink, white, violet, black, blueViolet, off, off
        };

        public static final double[] achillean_flag = {
        2.0, skyBlue, white, skyBlue, off, off
        };

        public static final double[] genderqueer_flag = {
        2.0, violet, white, lime, off, off
        };
        
        public static final double[] aro_flag = {
        2.0, darkGreen, lime, white, gray, black, off, off
        };

        public static final double[] ace_flag = {
        2.0, black, gray, white, violet, off, off
        };

        public static final double[] oriented_aroace = {
        2.0, black, gray, white, blueGreen, off, off
        };

        public static final double[] enby_flag = {
        2.0, yellow, white, violet, black, off, off
        };

        public static final double[] neutrois_flag = {
        2.0, white, lime, black, off, off
        };

        public static final double[] androgyne_flag = {
        2.0, hotPink, violet, skyBlue, off, off
        };

        public static final double[] polyamorous_flag = {
        2.0, blue, red, black, off, off
        };

        public static final double[] transfem_flag = {
        2.0, skyBlue, white, hotPink, white, skyBlue, off, off
        };

        public static final double[] bisexual_flag = {
        2.0, hotPink, violet, blueViolet, off, off
        };

        public static final double[] agender_flag = {
        2.0, black, gray, white, lime, gray, black, off, off
        };

        public static final double[] diamoric_flag = {
        2.0, lime, white, lime, off, off
        };

        public static final double[] libramasc_flag = {
        2.0, black, gray, white, skyBlue, white, gray, black, off, off
        };

        public static final double[] librafemme_flag = {
        2.0, black, gray, white, violet, white, gray, black, off, off
        };

        public static final double[] polygender_flag = {
        2.0, black, gray, blue, yellow, hotPink, off,
        };

        public static final double[] queerplatonic_flag = {
        2.0, yellow, hotPink, white, gray, black, off, off
        };

        public static final double[] pansexual_flag = {
        2.0, hotPink, yellow, skyBlue, off, off
        };

        public static final double[] maverique_flag = { 
        2.0, yellow, white, orange, off, off
        };

        public static final double[] greysexual_flag = {
        2.0, violet, gray, white, gray, violet, off, off
        };

        public static final double[] polysexual_flag = {
        2.0, hotPink, lime, skyBlue, off, off
        };

        public static final double[] trigender_flag = {
        2.0, hotPink, violet, lime, violet, hotPink, off, off
        };

        public static final double[] greyromantic_flag = {  
        2.0, darkGreen, gray, white, gray, darkGreen, off, off
        };
    }
}
