using System;
using Robocode.Util;

namespace Leceister
{
    public class BulletWave
    {
        public double StartX { get; set; }
        public double StartY { get; set; }
        public long FireTime { get; set; }
        public double Power { get; set; }
        public double Angle { get; set; }
        public double TargetAngle { get; set; }
        public int TargetDirection { get; set; } //1 is cloclwise; -1 is counter-clockwise
        public int[] Stats { get; set; }

        public BulletWave()
        {
            Angle = Double.NaN;
            TargetAngle = Double.NaN;
            TargetDirection = 1;
        }

        public BulletWave(double startX, double startY, long fireTime, double power, double angle, double targetAngle, int targetDirection)
        {
            StartX = startX;
            StartY = startY;
            FireTime = fireTime;
            Power = power;
            Angle = angle;
            TargetAngle = targetAngle;
            TargetDirection = targetDirection;
        }

        public double GetTraveledDistance(long curTime)
        {
            return (curTime - FireTime) * Velocity;
        }

        public double Velocity
        {
            get { return Helper.GetBulletVelocity(Power); }
        }

        public double MaxEscapeAngleRadian
        {
            get { return Math.Asin(8.0 / Velocity); }
        }

        public bool CheckHit(double enemyX, double enemyY, long currentTime)
        {
            // if the distance from the wave origin to our enemy has passed
            // the distance the bullet would have traveled...
            if (Helper.GetDistance(StartX, StartY, enemyX, enemyY) <= GetTraveledDistance(currentTime))
            {
                double desiredDirection = Math.Atan2(enemyX - StartX, enemyY - StartY);
                double angleOffset = Utils.NormalRelativeAngle(desiredDirection - TargetAngle);
                double guessFactor = Helper.Limit(-1, angleOffset / MaxEscapeAngleRadian, 1) * TargetDirection;
                int index = (int)Math.Round((Stats.Length - 1) * 0.5 * (guessFactor + 1));
                Stats[index]++;
                return true;
            }
            return false;
        }
    }
}