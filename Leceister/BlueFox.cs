using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using Robocode;
using Robocode.Util;

namespace Leceister
{
    public class BlueFox : AdvancedRobot
    {
        const double radarLockFactor = 2.0;

        private double randomAhead = 50;
        private double randomTurn = 90;
        private int randomDirection = 1;
        private Random rnd = null;
        private bool loseRadar = true;

        private int radarCount = 0;

        private RectangleF innerField;
        private const float WallStick = 150;
        private float radius;

        private List<BulletWave> enemyBullets = new List<BulletWave>();
        private List<int> _surfDirections = new List<int>();
        private List<double> _surfAbsBearings = new List<double>();

        private List<BulletWave> myBullets = new List<BulletWave>();
        private List<BulletWave> virtualBullets = new List<BulletWave>();

        const int StatsCount = 19;
        public double[] _surfStats = new double[StatsCount];
        public int[] _fireStats = new int[StatsCount];

        private PointF _lastEnemyLocation;  // enemy bot's location
        private double _lastEnemyEnergy = 100d;

        private PointF _sittingDuck = new PointF(0, 0);
        private PointF _lastHitEnemyPoint = new PointF(0, 0);

        public override void Run()
        {
            SetColors(Color.RoyalBlue, Color.RoyalBlue, Color.Goldenrod, Color.White, Color.Yellow);

            IsAdjustGunForRobotTurn = true;
            IsAdjustRadarForGunTurn = true;

            radius = (float)(Math.Max(Width, Height) / 1.41421);
            innerField = new RectangleF(radius, radius, (float)BattleFieldWidth - 2 * radius, (float)BattleFieldHeight - 2 * radius);

            //initial radar scan
            SetTurnRadarRight(double.MaxValue);

            //initial walk
            rnd = new Random(DateTime.Now.Second);
            rnd = new Random(DateTime.Now.Millisecond * rnd.Next(DateTime.Now.Second) + 1);

            while (true)
            {
                //                Out.WriteLine("!execute at round {0}", Time);

                Scan();

                if (radarCount == 0)
                {
                    Out.WriteLine("lose radar at round {0}", Time);
                    WalkWhenLoseRadar();
                }
                else
                {
                    radarCount--;
                }

                Execute();
                //                Out.WriteLine("execute finished at round {0}", Time);
            }
        }

        private void WalkWhenLoseRadar()
        {
            randomDirection = Math.Sign(rnd.Next(-1, 1));
            randomTurn = rnd.Next(0, 20) * randomDirection;
            SetTurnRight(randomTurn);
            SetAhead(200);

            SetTurnRadarRight(double.MaxValue);
        }

        public override void OnStatus(StatusEvent e)
        {
            //            Out.WriteLine("on round {0}", Time);
        }

        public override void OnScannedRobot(ScannedRobotEvent e)
        {
            //            Out.WriteLine("found enemy at round {0}", Time);

            radarCount++;
            //loseRadar = false;

            //lock radar
            double radarTurn = Utils.NormalRelativeAngleDegrees(Heading + e.Bearing - RadarHeading);
            SetTurnRadarRight(radarTurn * radarLockFactor);

            #region old
            //var p = GetBulletPower();
            //aiming
            //double gunTurn = Utils.NormalRelativeAngleDegrees(Heading + e.Bearing - GunHeading);
            //double eVLateral = e.Velocity * Math.Sin(e.HeadingRadians - HeadingRadians - e.BearingRadians);//positive means clockwise
            //double eVF = e.Velocity * Math.Cos(e.HeadingRadians - HeadingRadians - e.BearingRadians); //positive means away
            //double predictAngel = Utils.ToDegrees(Math.Asin(eVLateral / Helper.GetBulletVelocity(p)));
            //double factor = GuessAimingFactor(e.Velocity, eVLateral);
            //SetTurnGunRight(gunTurn + factor * predictAngel);

            ////fire
            //var offset = e.Distance <= 0 ? double.MaxValue : Utils.ToDegrees(Math.Atan(0.5 * Width / e.Distance));
            ////Out.WriteLine("GunTurnRemaining " + GunTurnRemaining + " offset " + offset);
            //if (GunHeat <= 0 && Math.Abs(GunTurnRemaining) <= offset)
            //    SetFireBullet(p);



            /*walk
                        double vLateral = Velocity * Math.Sin(e.BearingRadians); //positive means clockwise
                        double vF = Velocity * Math.Cos(e.BearingRadians);//positive means close
                        var rVF = vF - eVF;
                        var rVLateral = vLateral + eVLateral;
            
                        double turn;
                        if (e.Bearing >= 0)
                        {
                            turn = e.Bearing - 90;
                        }
                        else
                        {
                            turn = e.Bearing + 90;
                        }
                        SetTurnRight(turn + 180 * (rnd.Next(-1, 1)));
                        SetAhead(400);
*/
            #endregion
            SurfWave(e);
            Fire(e);
        }

        private void SurfWave(ScannedRobotEvent e)
        {
            var lateralVelocity = Velocity * Math.Sin(e.BearingRadians);
            var absBearing = e.BearingRadians + HeadingRadians;

            _surfDirections.Insert(0, lateralVelocity >= 0 ? 1 : -1);
            _surfAbsBearings.Insert(0, absBearing + Math.PI);

            double energyDrop = _lastEnemyEnergy - e.Energy;
            if (energyDrop < 3.01 && energyDrop > 0.09 && _surfDirections.Count > 2)
            {
                BulletWave ew = new BulletWave()
                {
                    StartX = _lastEnemyLocation.X,
                    StartY = _lastEnemyLocation.Y,
                    FireTime = Time - 1,
                    Power = energyDrop,
                    TargetAngle = _surfAbsBearings[2],
                    TargetDirection = _surfDirections[2],
                };
                enemyBullets.Add(ew);
            }
            _lastEnemyEnergy = e.Energy;
            _lastEnemyLocation = Helper.Project(X, Y, absBearing, e.Distance);

            //update enemy waves
            for (var i = 0; i < enemyBullets.Count; i++)
            {
                var ew1 = enemyBullets[i];
                var traveledDistance = ew1.GetTraveledDistance(Time);
                var curDistance = Helper.GetDistance(ew1.StartX, ew1.StartY, X, Y);
                if (traveledDistance > curDistance + 2 * radius)
                {
                    enemyBullets.RemoveAt(i);
                    i--;
                }
            }
            DoSurfing();
        }

        private void Fire(ScannedRobotEvent e)
        {
            // Enemy absolute bearing, you can use your one if you already declare it.
            double absBearing = HeadingRadians + e.BearingRadians;

            // find our enemy's location:
            double ex = X + Math.Sin(absBearing) * e.Distance;
            double ey = Y + Math.Cos(absBearing) * e.Distance;

            //update bullet waves
            for (int i = 0; i < myBullets.Count; i++)
            {
                var currentWave = myBullets[i];
                if (currentWave.CheckHit(ex, ey, Time))
                {
                    myBullets.Remove(currentWave);
                    i--;
                }
            }
            //update virtual bullet waves
            for (int i = 0; i < virtualBullets.Count; i++)
            {
                var currentWave = virtualBullets[i];
                if (currentWave.CheckHit(ex, ey, Time))
                {
                    virtualBullets.Remove(currentWave);
                    i--;
                }
            }

            double power = GetBulletPower(e);
            var enemyDirection = 1;
            // don't try to figure out the direction they're moving 
            // they're not moving, just use the direction we had before
            if (Math.Abs(e.Velocity) > 0)
            {
                if (Math.Sin(e.HeadingRadians - absBearing) * e.Velocity < 0)
                    enemyDirection = -1;
                else
                    enemyDirection = 1;
            }
            int[] currentStats = _fireStats; // This seems silly, but I'm using it to // show something else later
            var newWave = new BulletWave()
            {
                StartX = X,
                StartY = Y,
                FireTime = Time,
                TargetAngle = absBearing,
                TargetDirection = enemyDirection,
                Stats = currentStats
            };

            double angleOffset = enemyDirection * newWave.MaxEscapeAngleRadian * GetGuessfactor();

            //handle easy case
            if (Math.Abs(ex - _lastHitEnemyPoint.X) < Rules.MAX_VELOCITY && Math.Abs(ey - _lastHitEnemyPoint.Y) < Rules.MAX_VELOCITY)
            {
                angleOffset = 0;
                power = 3;
            }

            double gunAdjust = Utils.NormalRelativeAngle(absBearing - GunHeadingRadians + angleOffset);
            SetTurnGunRightRadians(gunAdjust);

            Bullet fireBullet = null;
            if (GunHeat <= 0 && gunAdjust < Math.Atan2(9, e.Distance))
            {
                fireBullet = SetFireBullet(power);
                if (fireBullet != null)
                {
                    myBullets.Add(newWave);
                    newWave.Power = power;
                    newWave.Angle = fireBullet.HeadingRadians;
                }
            }
            //virtual bullet
            if (fireBullet == null)
            {
                virtualBullets.Add(newWave);
                newWave.Power = power;
                newWave.Angle = absBearing;
            }

        }

        private double GetGuessfactor()
        {
            int bestindex = 15; // initialize it to be in the middle, guessfactor 0.
            for (int i = 0; i < 31; i++)
                if (_fireStats[bestindex] < _fireStats[i])
                    bestindex = i;

            // this should do the opposite of the math in the WaveBullet:
            return (double)(bestindex - (_fireStats.Length - 1) / 2) / ((_fireStats.Length - 1) / 2);
        }

        private double GuessAimingFactor(double v, double vLateral)
        {
            return Math.Pow(Math.Abs(vLateral) / Rules.MAX_VELOCITY, 1d);
        }

        private double GetBulletPower(ScannedRobotEvent e)
        {
            if (e.Distance <= 4 * radius)
                return 3d;
            if (e.Distance <= 8 * radius)
                return 1d;
            return 0.1d;
        }

        private void DoSurfing()
        {
            BulletWave surfWave = GetClosestSurfableWave();

            if (surfWave == null) return;

            double dangerLeft = CheckDanger(surfWave, -1);
            double dangerRight = CheckDanger(surfWave, 1);

            double goAngle = Helper.GetAbsBearingInRadian(surfWave.StartX, surfWave.StartY, X, Y);
            if (dangerLeft < dangerRight)
            {
                goAngle = WallSmoothing(X, Y, goAngle - (Math.PI * 0.5), -1);
            }
            else
            {
                goAngle = WallSmoothing(X, Y, goAngle + (Math.PI * 0.5), 1);
            }

            SetBackAsFront(goAngle);
        }

        private BulletWave GetClosestSurfableWave()
        {
            double closestDistance = double.MaxValue;
            BulletWave surfWave = null;

            foreach (var ew in enemyBullets)
            {
                var traveledDistance = ew.GetTraveledDistance(Time);
                var curDistance = Helper.GetDistance(ew.StartX, ew.StartY, X, Y);

                double distance = curDistance - traveledDistance;

                if (distance > ew.Velocity && distance < closestDistance)
                {
                    surfWave = ew;
                    closestDistance = distance;
                }
            }

            return surfWave;
        }

        // Given the EnemyWave that the bullet was on, and the point where we
        // were hit, calculate the index into our stat array for that factor.
        private static int GetFactorIndex(BulletWave ew, double targetX, double targetY)
        {
            double offsetAngle = Helper.GetAbsBearingInRadian(ew.StartX, ew.StartY, targetX, targetY) - ew.TargetAngle;
            double factor = Utils.NormalRelativeAngle(offsetAngle) / ew.MaxEscapeAngleRadian * ew.TargetDirection;

            return (int)Helper.Limit(0, 0.5 * (1 + factor) * (StatsCount - 1), StatsCount - 1);
        }

        // Given the EnemyWave that the bullet was on, and the point where we
        // were hit, update our stat array to reflect the danger in that area.
        private void LogHit(BulletWave ew, double targetX, double targetY)
        {
            int index = GetFactorIndex(ew, targetX, targetY);

            for (int i = 0; i < StatsCount; i++)
            {
                // for the spot bin that we were hit on, add 1;
                // for the bins next to it, add 1 / 2;
                // the next one, add 1 / 5; and so on...
                _surfStats[i] += 1.0 / (Math.Pow(index - i, 2) + 1);
            }
        }

        private void SetBackAsFront(double angleInRadian)
        {
            double turn = Utils.NormalRelativeAngle(angleInRadian - HeadingRadians);
            if (Math.Abs(turn) > (Math.PI / 2))
            {
                if (turn < 0)
                {
                    SetTurnRightRadians(Math.PI + turn);
                }
                else
                {
                    SetTurnLeftRadians(Math.PI - turn);
                }
                SetBack(100);
            }
            else
            {
                if (turn < 0)
                {
                    SetTurnLeftRadians(-turn);
                }
                else
                {
                    SetTurnRightRadians(turn);
                }
                SetAhead(100);
            }
        }

        private PointF PredictPosition(BulletWave surfWave, int direction)
        {
            PointF predictedPosition = new PointF((float)X, (float)Y);
            double predictedVelocity = Velocity;
            double predictedHeading = HeadingRadians;
            double maxTurning, moveAngle, moveDir;

            int counter = 0; // number of ticks in the future
            bool intercepted = false;

            do
            {    // the rest of these code comments are rozu's
                var angleInRadian = Helper.GetAbsBearingInRadian(surfWave.StartX, surfWave.StartY, predictedPosition.X, predictedPosition.Y) + (direction * (Math.PI / 2));
                moveAngle = WallSmoothing(predictedPosition.X, predictedPosition.Y, angleInRadian, direction) - predictedHeading;
                moveDir = 1;

                if (Math.Cos(moveAngle) < 0)
                {
                    moveAngle += Math.PI;
                    moveDir = -1;
                }

                moveAngle = Utils.NormalRelativeAngle(moveAngle);

                // maxTurning is built in like this, you can't turn more then this in one tick
                maxTurning = Math.PI / 720d * (40d - 3d * Math.Abs(predictedVelocity));
                predictedHeading = Utils.NormalRelativeAngle(predictedHeading + Helper.Limit(-maxTurning, moveAngle, maxTurning));

                // this one is nice ;). if predictedVelocity and moveDir have
                // different signs you want to breack down
                // otherwise you want to accelerate (look at the factor "2")
                predictedVelocity +=
                    (predictedVelocity * moveDir < 0 ? 2 * moveDir : moveDir);
                predictedVelocity = Helper.Limit(-8, predictedVelocity, 8);

                // calculate the new predicted position
                predictedPosition = Helper.Project(predictedPosition.X, predictedPosition.Y, predictedHeading, predictedVelocity);

                counter++;

                if (Helper.GetDistance(predictedPosition.X, predictedPosition.Y, surfWave.StartX, surfWave.StartY) <
                    surfWave.GetTraveledDistance(Time) + (counter + 1) * surfWave.Velocity)
                {
                    intercepted = true;
                }
            } while (!intercepted && counter < 500);

            return predictedPosition;
        }

        private double CheckDanger(BulletWave surfWave, int direction)
        {
            var predictPosition = PredictPosition(surfWave, direction);
            int index = GetFactorIndex(surfWave, predictPosition.X, predictPosition.Y);
            return _surfStats[index];
        }

        private double WallSmoothing(double curX, double curY, double angleInRadian, int offsetTurn)
        {
            while (!innerField.Contains(Helper.Project(curX, curY, angleInRadian, WallStick)))
            {
                angleInRadian += offsetTurn * 0.1;
            }
            return angleInRadian;
        }

        public override void OnWin(WinEvent evnt)
        {
            SetColors(Color.Gold, Color.Gold, Color.Gold, Color.Gold, Color.Gold);
            TurnRadarRight(double.MaxValue);
        }

        public override void OnHitByBullet(HitByBulletEvent e)
        {
            // If the _enemyWaves collection is empty, we must have missed the
            // detection of this wave somehow.
            if (enemyBullets.Any())
            {
                BulletWave hitWave = null;
                foreach (var ew in enemyBullets)
                {
                    if (Math.Abs(ew.GetTraveledDistance(Time) - Helper.GetDistance(X, Y, ew.StartX, ew.StartY)) <= radius &&
                        Math.Abs(Helper.GetBulletVelocity(e.Bullet.Power) - ew.Velocity) < 0.001)
                    {
                        hitWave = ew;
                        break;
                    }
                }

                if (hitWave != null)
                {
                    LogHit(hitWave, e.Bullet.X, e.Bullet.Y);
                    enemyBullets.Remove(hitWave);
                }
            }
        }

        public override void OnHitRobot(HitRobotEvent e)
        {
            _lastEnemyEnergy = e.Energy;
        }

        public override void OnBulletHit(BulletHitEvent e)
        {
            _lastEnemyEnergy = e.VictimEnergy;

            foreach (var bullet in myBullets)
            {
                if (Math.Abs(bullet.GetTraveledDistance(Time) - Helper.GetDistance(e.Bullet.X, e.Bullet.Y, bullet.StartX, bullet.StartY)) <= radius &&
                    Math.Abs(e.Bullet.Power - bullet.Power) < 0.001 && Math.Abs(e.Bullet.HeadingRadians - bullet.Angle) < 0.001)
                {
                    bullet.CheckHit(e.Bullet.X, e.Bullet.Y, Time); //maybe add more weight
                                                                   //                    Out.WriteLine("angle: {0}, bearing: {1}", bullet.Angle, bullet.TargetAngle);
                    break;
                }
            }

            _lastHitEnemyPoint = new PointF((float)e.Bullet.X, (float)e.Bullet.Y);
        }

        public override void OnBulletHitBullet(BulletHitBulletEvent e)
        {
            BulletWave hitWave = null;
            foreach (var ew in enemyBullets)
            {
                if (Math.Abs(ew.GetTraveledDistance(Time) -
                             Helper.GetDistance(e.HitBullet.X, e.HitBullet.Y, ew.StartX, ew.StartY)) < 0.001 &&
                    Math.Abs(Helper.GetBulletVelocity(e.HitBullet.Power) - ew.Velocity) < 0.001)
                {
                    hitWave = ew;
                    break;
                }
            }

            if (hitWave != null)
            {
                LogHit(hitWave, e.HitBullet.X, e.HitBullet.Y);
                enemyBullets.Remove(hitWave);
            }

            foreach (var bullet in myBullets)
            {
                if (Math.Abs(bullet.GetTraveledDistance(Time) - Helper.GetDistance(e.HitBullet.X, e.HitBullet.Y, bullet.StartX, bullet.StartY)) < 0.001 &&
                    Math.Abs(e.HitBullet.Power - bullet.Power) < 0.001 && Math.Abs(e.Bullet.HeadingRadians - bullet.Angle) < 0.001)
                {
                    bullet.CheckHit(e.HitBullet.X, e.HitBullet.Y, Time); //maybe add more weight
                    Out.WriteLine("angle: {0}, bearing: {1}", bullet.Angle, bullet.TargetAngle);
                    break;
                }
            }
        }

        public override void OnBulletMissed(BulletMissedEvent evnt)
        {
            _lastHitEnemyPoint = new PointF(0, 0);
        }
    }
}
