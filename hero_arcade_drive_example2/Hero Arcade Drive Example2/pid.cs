using System;
using Microsoft.SPOT;
using CTRE.Phoenix;
using CTRE.Phoenix.Controller;

namespace HERO_Serial_Example10
{
    class pid
    {

        public static double power = 0;
        public static double distance_totalError = 0;
        //public static GameController _gamepad = new GameController(UsbHostDevice.GetInstance());

        // button ID's


        public static double pid_sol_arka(double kP, double kI, double kD, double target)//////talon4
        {

            double distance_error;

            double enco_rpm;

            enco_rpm = (((double)(Program.axis_5.GetSelectedSensorVelocity()) / (850 * 12*3)));

            Debug.Print("  ENKO RPM ==========   "+enco_rpm.ToString());
            distance_error = (target) - enco_rpm;
            double distance_oldError = distance_error;
            if (-0.03 < distance_error && distance_error < 0.03)
            {
                power = target;
                distance_totalError = 0;
            }

            else
            {
                distance_totalError += distance_error;
                power = ((kP * distance_error) + (kI * (distance_totalError)) + (kD * (distance_error - distance_oldError)));
            }
            Debug.Print("POWER =====   " + power.ToString());

            if (power > 0.333)
            {
                power = 0.333;
                Debug.Print("xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx");
            }

            else if (power < -0.333)
            {
                power = -0.333;
                Debug.Print("zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz");
            }
            Debug.Print("POWER =====   " + power.ToString());

            return power;
        }
    }
}
