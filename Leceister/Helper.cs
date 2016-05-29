using System;
using System.Drawing;
using Robocode;

namespace Leceister
{
    public class Helper
    {
        public static double GetDistance(double x1, double y1, double x2, double y2)
        {
            return Math.Sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
        }

        public static double GetBulletVelocity(double p)
        {
            return 20d - 3 * p;
        }

        public static double GetHeat(double p)
        {
            return 1 + 0.2 * p;
        }

        public static double GetBulletInterval(double p, Robot bot)
        {
            return (GetBulletVelocity(p) - Rules.MAX_VELOCITY) * GetHeat(p) / bot.GunCoolingRate;
        }

        public static double GetDamage(double p)
        {
            return 4 * p + Math.Max(0, 2 * (p - 1));
        }

        public static double GetEnergyReturn(double p)
        {
            return 3 * p;
        }

        public static double GetEnergyCost(double p)
        {
            return p;
        }

        public static double GetMaxTurnRate(double v)
        {
            return 10 - 0.75 * Math.Abs(v);
        }

        public static double Limit(double min, double value, double max)
        {
            return Math.Max(min, Math.Min(value, max));
        }

        public static double GetAbsBearingInRadian(double meX, double meY, double targetX, double targetY)
        {
            return Math.Atan2(targetX - meX, targetY - meY);
        }

        public static PointF Project(double curX, double curY, double angleInRadian, double distance)
        {
            return new PointF((float)(curX + Math.Sin(angleInRadian) * distance), (float)(curY + Math.Cos(angleInRadian) * distance));
        }
    }


}
