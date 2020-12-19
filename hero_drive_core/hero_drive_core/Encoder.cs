using System;
using Microsoft.SPOT;
using System.Threading;
using System.Text;

using CTRE.Phoenix;
using CTRE.Phoenix.Controller;
using CTRE.Phoenix.MotorControl.CAN;
using CTRE.Phoenix.Motion;
using CTRE.Phoenix.MotorControl;

namespace hero_drive_core
{
    class Encoder
    {
        public static double do_encoder(TalonSRX talon_name)
        {
            int encoder_rpm = talon_name.GetSelectedSensorVelocity();
            if (talon_name == IO.sag_on || talon_name == IO.sol_on)
            {
                encoder_rpm *= -1;
            }

            return (double)encoder_rpm * -147 / 100000;
        }
    }
}
