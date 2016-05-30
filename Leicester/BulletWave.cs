using System;
using Robocode;
using Robocode.Util;

namespace Leicester
{
    public class BulletWave
    {
        public double StartX { get; set; }
        public double StartY { get; set; }
        public long FireTime { get; set; }
        public double Power { get; set; }
        /// <summary>
        /// in radian
        /// </summary>
        public double Angle { get; set; }
        /// <summary>
        /// in radian
        /// </summary>
        public double TargetAngle { get; set; }
        /// <summary>
        /// 1 is cloclwise; -1 is counter-clockwise
        /// </summary>
        public int TargetDirection { get; set; }
        public double[] Stats { get; set; }

        public BulletWave()
        {
            Angle = Double.NaN;
            TargetAngle = Double.NaN;
            TargetDirection = 1;
            Stats = null;
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
            Stats = null;
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
            get { return Math.Asin(Rules.MAX_VELOCITY / Velocity); }
        }

        public bool CheckHit(double enemyX, double enemyY, long currentTime, double radius = 0)
        {
            // if the distance from the wave origin to our enemy has passed
            // the distance the bullet would have traveled...
            if (Helper.GetDistance(StartX, StartY, enemyX, enemyY) + radius <= GetTraveledDistance(currentTime))
            {
                if (Stats != null)
                {
                    double desiredDirection = Math.Atan2(enemyX - StartX, enemyY - StartY);
                    double angleOffset = Utils.NormalRelativeAngle(desiredDirection - TargetAngle);
                    double guessFactor = Helper.Limit(-1, angleOffset / MaxEscapeAngleRadian, 1) * TargetDirection;
                    int index = (int)Math.Round((Stats.Length - 1) * 0.5 * (guessFactor + 1));
                    Helper.UpdateStats(index, Stats);
                }
                return true;
            }
            return false;
        }
    }
}