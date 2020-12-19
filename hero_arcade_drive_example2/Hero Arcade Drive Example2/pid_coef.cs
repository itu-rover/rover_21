using System;
using System.Threading;
using Microsoft.SPOT;
using System.Text;
using CTRE.Phoenix;
using CTRE.Phoenix.Controller;
using CTRE.Phoenix.MotorControl;
using CTRE.Phoenix.MotorControl.CAN;

using CTRE.Phoenix.Motion;

namespace HERO_Serial_Example10
{
    class pid_coef
    {
        public static uint kP_inc = 4;
        public static uint kP_dec = 2;
        public static uint kI_inc = 3;
        public static uint kI_dec = 1;
        public static uint kD_inc = 6;
        public static uint kD_dec = 5;
        public static uint[] param_incs = { kP_inc, kI_inc, kD_inc };
        public static uint[] param_decs = { kP_dec, kI_dec, kD_dec };
        public static double kP = 0.70;
        public static double kI = 0;
        public static double kD = 0;
        public static double[] parameters = { kP, kI, kD };
        public static Boolean kP_inc_prev = false;
        public static Boolean kP_inc_current = false;
        public static Boolean kP_dec_prev = false;
        public static Boolean kP_dec_current = false;
        public static Boolean kI_inc_prev = false;
        public static Boolean kI_inc_current = false;
        public static Boolean kI_dec_prev = false;
        public static Boolean kI_dec_current = false;
        public static Boolean kD_inc_prev = false;
        public static Boolean kD_inc_current = false;
        public static Boolean kD_dec_prev = false;
        public static Boolean kD_dec_current = false;

        public static void parameter_changer(double x)
        {
            /*
            foreach (double parameter in parameters) {
                foreach (uint param_inc in param_incs)
                {
                    if (Program._gamepad.GetButton(param_inc) == true) // Butona basılırsa
                    {
                        parameter += 0.05;
                    }
                }
                foreach (uint param_dec in param_decs)
                {
                    if (Program._gamepad.GetButton(param_dec) == true)// Butona basılırsa
                    {
                        parameter -= 0.05;
                    }
                }
                Debug.Print(" : " + parameter);
            }
             */

            // kP arttıran butonun şimdiki değeri ile kumandadan gelen buton değer (true-false) eşit
            kP_inc_current = Program._gamepad.GetButton(kP_inc);

            if (kP_inc_current && !kP_inc_prev) // Butona basılırsa
            {
                kP += 0.05;
                Debug.Print(" kP: " + kP);
            }
            //Debug.Print(Program._gamepad.GetButton(kP_inc).ToString());

            kP_inc_prev = kP_inc_current; // önceki değer ve şimdiki değeri birbirine eşitle
            // ki butona basıldığında 2. kez true döndürmesin

            kP_dec_current = Program._gamepad.GetButton(kP_dec);

            if (kP_dec_current && !kP_dec_prev) // Butona basılırsa
            {
                kP -= 0.05;
                Debug.Print(" kP: " + kP);
            }
            kP_dec_prev = kP_dec_current; // önceki değer ve şimdiki değeri birbirine eşitle
            // ki butona basıldığında 2. kez true döndürmesin

            // -----------------------------------------------------------------------------------------------------------------
            // -----------------------------------------------------------------------------------------------------------------
            // -----------------------------------------------------------------------------------------------------------------
            // -----------------------------------------------------------------------------------------------------------------


            // kP arttıran butonun şimdiki değeri ile kumandadan gelen buton değer (true-false) eşit
            kI_inc_current = Program._gamepad.GetButton(kI_inc);

            if (kI_inc_current && !kI_inc_prev) // Butona basılırsa
            {
                kI += 0.05;
                Debug.Print(" kI: " + kI);
            }

            kI_inc_prev = kI_inc_current; // önceki değer ve şimdiki değeri birbirine eşitle
            // ki butona basıldığında 2. kez true döndürmesin

            kI_dec_current = Program._gamepad.GetButton(kI_dec);

            if (kI_dec_current && !kI_dec_prev) // Butona basılırsa
            {
                kI -= 0.05;
                Debug.Print(" kI: " + kP);
            }
            kI_dec_prev = kI_dec_current; // önceki değer ve şimdiki değeri birbirine eşitle
            // ki butona basıldığında 2. kez true döndürmesin


            // -----------------------------------------------------------------------------------------------------------------
            // -----------------------------------------------------------------------------------------------------------------
            // -----------------------------------------------------------------------------------------------------------------
            // -----------------------------------------------------------------------------------------------------------------

            // kP arttıran butonun şimdiki değeri ile kumandadan gelen buton değer (true-false) eşit
            kD_inc_current = Program._gamepad.GetButton(kD_inc);

            if (kD_inc_current && !kD_inc_prev) // Butona basılırsa
            {
                kD += 0.05;
                Debug.Print(" kD: " + kD);
            }

            kD_inc_prev = kD_inc_current; // önceki değer ve şimdiki değeri birbirine eşitle
            // ki butona basıldığında 2. kez true döndürmesin

            kD_dec_current = Program._gamepad.GetButton(kD_dec);

            if (kD_dec_current && !kD_dec_prev) // Butona basılırsa
            {
                kD -= 0.05;
                Debug.Print(" kD: " + kD);
            }
            kD_dec_prev = kD_dec_current; // önceki değer ve şimdiki değeri birbirine eşitle
            // ki butona basıldığında 2. kez true döndürmesin


            Program.axis_5.Set(ControlMode.PercentOutput, pid.pid_sol_arka(kP, kI, kD, x));
            // --------------------------------------------------------
        }
    }
}
