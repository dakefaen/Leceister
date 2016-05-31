using System;
using System.Collections.Generic;
using System.Drawing;
using System.IO;
using System.Linq;
using Robocode;
using Robocode.Util;

namespace Leicester
{
    public class BlueFox : AdvancedRobot
    {
        private const float WallStick = 160;
        private const double radarLockFactor = 2.0;
        private const int StatsCount = 63;
        private const int DistanceIndexCount = 13;
        private const int SafeDistance = 300;
        private const string FileName = "Stats.txt";

        private double randomTurn = 90;
        private int randomDirection = 1;
        private Random rnd = null;

        private int _radarCount = 0;

        private RectangleF _innerField;
        private float _radius;

        private List<BulletWave> enemyBullets = new List<BulletWave>();
        private List<int> _surfDirections = new List<int>();
        private List<double> _surfAbsBearings = new List<double>();

        private List<BulletWave> myBullets = new List<BulletWave>();
        private List<BulletWave> virtualBullets = new List<BulletWave>();

        public double[] _surfStats = new double[StatsCount];
        public double[][] _fireStats = new double[DistanceIndexCount][];

        private PointF _lastEnemyLocation;
        private double _lastEnemyEnergy = 100d;
        //        private double _enemyGunHeat = 0;

        private PointF _predictPoint = new PointF(0, 0);
        //        private PointF _lastHitEnemyPoint = new PointF(0, 0);

        private static int medKitTurnPast = 0;
        private bool medKitAppeared = false;
        private bool medKitLocked = false;
        private PointF _lastMedPoint = new PointF(0, 0);
        private double medEnergy = 5;

        private static int round = 1;
        //        private bool hasScanedMed = false;
        //        private long startMedScanTime = 0;

        public override void Run()
        {
            SetColors(Color.RoyalBlue, Color.Blue, Color.Gold, Color.Yellow, Color.Yellow);

            IsAdjustGunForRobotTurn = true;
            IsAdjustRadarForGunTurn = true;

            _radius = (float)(Math.Max(Width, Height) / 1.41421);
            _innerField = new RectangleF(_radius, _radius, (float)BattleFieldWidth - 2 * _radius, (float)BattleFieldHeight - 2 * _radius);


            for (int i = 0; i < _fireStats.Length; i++)
            {
                _fireStats[i] = new double[StatsCount];
            }

            //initial radar scan
            SetTurnRadarRight(double.MaxValue);

            //initial walk
            rnd = new Random(DateTime.Now.Second);
            rnd = new Random(DateTime.Now.Millisecond * rnd.Next(DateTime.Now.Second) + 1);

            //ReadStats();
            //ReadRound();

            while (true)
            {
                Out.WriteLine(Time + "\t" + medKitTurnPast);
                if (round == 1 && medKitTurnPast % 500 == 0)
                //                    || (round > 1 && medKitTurnPast % 50 == 0))
                {
                    medKitAppeared = true;
                    medKitLocked = false;
                }

                Scan();
                if (_radarCount == 0)
                {
                    //                Out.WriteLine("lose radar lock: "+Time);
                    WalkWhenLoseRadar();
                    SetTurnRadarRight(double.MaxValue);
                }

                if (medKitAppeared && !medKitLocked)
                {
                    //                    Out.WriteLine("start scan med kit " + Time + "\t" + medKitTurnPast);
                    SetTurnRadarRight(double.MaxValue);
                    //                    if (!hasScanedMed)
                    //                        startMedScanTime = Time;
                }


                Execute();

                medKitTurnPast += 2;
            }
        }

        private void WalkWhenLoseRadar()
        {
            SetTurnAwayFromWall();
            SetAhead(rnd.Next(50, 150));
        }

        public override void OnScannedRobot(ScannedRobotEvent e)
        {
            Out.WriteLine("found enemy " + Time + "\t" + medKitTurnPast);
            if (_radarCount < 3)
                _radarCount++;

            if (medKitAppeared && !medKitLocked)
            {
                //                Out.WriteLine("start scan med kit " + Time + "\t" + medKitTurnPast);
                SetTurnRadarRight(double.MaxValue);
            }
            else
            {
                //lock radar
                double radarTurn = Utils.NormalRelativeAngle(HeadingRadians + e.BearingRadians - RadarHeadingRadians);
                SetTurnRadarRightRadians(radarTurn * radarLockFactor);
            }

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

            UpdateEnemyWave(e);
            if (medKitLocked && Energy - _lastEnemyEnergy < medEnergy)
            {
                var maxV = rnd.Next(6, 8);
                MaxVelocity = maxV;
                GoToMed();
                //FireOnTheWayToMedKit(e);
            }
            else
            {
                MaxVelocity = Rules.MAX_VELOCITY;
                DoSurfing();
            }
            Fire(e);
        }

        private void FireOnTheWayToMedKit(ScannedRobotEvent e)
        {
            var ev = 8;
            var power = GetBulletPower(e);

            var absBearing = e.BearingRadians + HeadingRadians;

            _lastEnemyEnergy = e.Energy;
            var enemyLocation = Helper.GetProjection(X, Y, absBearing, e.Distance);


            var distToMed = Helper.GetDistance(X, Y, _lastMedPoint.X, _lastMedPoint.Y);
            var distFromEnemyToMed = Helper.GetDistance(enemyLocation.X, enemyLocation.Y, _lastMedPoint.X, _lastMedPoint.Y);

            var lam = Helper.GetAbsBearingInRadian(X, Y, _lastMedPoint.X, _lastMedPoint.Y) -
                      Helper.GetAbsBearingInRadian(X, Y, enemyLocation.X, enemyLocation.Y);
            var sinB = distToMed / Math.Max(0.0001, distFromEnemyToMed) * Math.Sin(lam);

            var sinA = ev / Helper.GetBulletVelocity(power) * sinB;

            var gunAdjust = absBearing + Math.Asin(sinA) - GunHeadingRadians;
            SetTurnGunRightRadians(gunAdjust);

            if (GunHeat <= 0 && gunAdjust < Math.Atan2(0.5 * _radius, e.Distance))
            {
                var fireBullet = SetFireBullet(power);
            }
        }

        private void GoToMed()
        {
            var bearing = Helper.GetAbsBearingInRadian(X, Y, _lastMedPoint.X, _lastMedPoint.Y);
            var dist = Helper.GetDistance(X, Y, _lastMedPoint.X, _lastMedPoint.Y);

            SetBackAsFront(bearing, dist);
        }

        public override void OnScannedMedicalKit(ScannedMedicalKitEvent e)
        {
            medKitAppeared = true;
            var absBearing = HeadingRadians + e.BearingRadians;
            _lastMedPoint = Helper.GetProjection(X, Y, absBearing, e.Distance);
            medEnergy = e.HealEnergy;
            medKitLocked = true;
//            hasScanedMed = true;
            //            Out.WriteLine("Time : {0} \t medturn: {1}", Time, medKitTurnPast);
        }

        public override void OnFetchMedicalKit(FetchedMedicalKitEvent evnt)
        {
            Out.WriteLine("eat!!! Time : {0} \t medturn: {1}", Time, medKitTurnPast);

            medKitAppeared = false;
            medKitLocked = false;
//            hasScanedMed = true;
        }

        private void UpdateEnemyWave(ScannedRobotEvent e)
        {
            var lateralVelocity = Velocity * Math.Sin(e.BearingRadians);
            var absBearing = e.BearingRadians + HeadingRadians;

            _surfDirections.Insert(0, lateralVelocity >= 0 ? 1 : -1);
            _surfAbsBearings.Insert(0, absBearing + Math.PI);

            double energyDrop = _lastEnemyEnergy - e.Energy;
            if (energyDrop < Rules.MAX_BULLET_POWER + 0.0001 && energyDrop > Rules.MIN_BULLET_POWER - 0.0001 && _surfDirections.Count > 2)
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
            _lastEnemyLocation = Helper.GetProjection(X, Y, absBearing, e.Distance);

            //update enemy waves
            for (var i = 0; i < enemyBullets.Count; i++)
            {
                if (enemyBullets[i].CheckHit(X, Y, Time, 2 * _radius))
                {
                    enemyBullets.RemoveAt(i);
                    i--;
                }
            }
        }

        private void Fire(ScannedRobotEvent e)
        {
            // Enemy absolute bearing
            double absBearing = HeadingRadians + e.BearingRadians;

            // enemy's location:
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
            var distanceIndex = (int)(e.Distance / 100);
            double[] currentStats = _fireStats[distanceIndex];
            var newWave = new BulletWave()
            {
                StartX = X,
                StartY = Y,
                FireTime = Time,
                Power = power,
                TargetAngle = absBearing,
                TargetDirection = enemyDirection,
                Stats = currentStats
            };

            double angleOffset = enemyDirection * newWave.MaxEscapeAngleRadian * GetGuessfactor(currentStats);

            //handle easy case
            if (e.Energy < 0.1)
            {
                angleOffset = 0;
                power = Math.Min(3, 0.5 * Energy);
            }

            double gunAdjust = Utils.NormalRelativeAngle(absBearing - GunHeadingRadians + angleOffset);
            SetTurnGunRightRadians(gunAdjust);

            Bullet fireBullet = null;
            if (GunHeat <= 0 && gunAdjust < Math.Atan2(0.5 * _radius, e.Distance))
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

        private double GetGuessfactor(double[] fireStats)
        {
            int bestindex = (StatsCount + 1) / 2; // initialize it to be in the middle, guessfactor 0.
            for (int i = 0; i < StatsCount; i++)
                if (fireStats[bestindex] < fireStats[i])
                    bestindex = i;

            // this should do the opposite of the math in the WaveBullet:
            return (double)(bestindex - (fireStats.Length - 1) / 2) / ((fireStats.Length - 1) / 2);
        }

        private double GetBulletPower(ScannedRobotEvent e)
        {
            if (e.Distance <= 4 * _radius)
                return 3d;
            if (e.Distance <= 6 * _radius)
                return 2d;
            return Helper.Limit(0.1, 0.1d * rnd.Next(1, 10), Energy * 0.1);
        }

        private void DoSurfing()
        {
            BulletWave surfWave = GetClosestBullet();

            if (surfWave == null)
            {
                if (!WalkAwayFromEnemy())
                {
                    SetTurnAwayFromWall();
                    SetAhead(rnd.Next(50, 150));
                }
                return;
            }

            PointF predictPoint1, predictPoint2;
            double dangerLeft = CheckDanger(surfWave, -1, out predictPoint1);
            double dangerRight = CheckDanger(surfWave, 1, out predictPoint2);

            double goAngle = Helper.GetAbsBearingInRadian(surfWave.StartX, surfWave.StartY, X, Y);
            if (dangerLeft < dangerRight)
            {
                goAngle = WallSmoothing(X, Y, goAngle - (Math.PI * 0.5), -1);
                _predictPoint = predictPoint2;
            }
            else
            {
                goAngle = WallSmoothing(X, Y, goAngle + (Math.PI * 0.5), 1);
                _predictPoint = predictPoint1;
            }

            SetBackAsFront(goAngle, rnd.Next(50, 150));
        }

        private void SetTurnAwayFromWall()
        {
            int i = 0;
            if (Y < 2 * _radius)
                i += 1;
            else if (Y > BattleFieldHeight - 2 * _radius)
                i += 2;
            if (X < 2 * _radius)
                i += 4;
            if (X > BattleFieldWidth - 2 * _radius)
                i += 7;
            double[] radians = { -1, 0, Math.PI, -1, Math.PI / 2, Math.PI / 4, Math.PI * 3 / 4, Math.PI * 3 / 2,
                Math.PI * 7 / 4, Math.PI * 5 / 4 };
            if (radians[i] < 0) return;

            double heading = radians[i] + Utils.ToRadians(rnd.Next(-20, 20));
            double bearing = Utils.NormalRelativeAngle(heading - HeadingRadians);
            SetTurnRightRadians(bearing);
        }

        private bool WalkAwayFromEnemy()
        {
            var distance = Helper.GetDistance(X, Y, _lastEnemyLocation.X, _lastEnemyLocation.Y);
            if (distance < SafeDistance)
            {
                double goAngle = HeadingRadians;
                var bearing = Utils.NormalRelativeAngle(Helper.GetAbsBearingInRadian(X, Y, _lastEnemyLocation.X, _lastEnemyLocation.Y) - HeadingRadians);
                if (Math.Abs(bearing) < Math.PI * 0.5)
                {
                    if (bearing > 0)
                        goAngle = HeadingRadians + bearing - Math.PI * 0.5;
                    else
                        goAngle = HeadingRadians + bearing + Math.PI * 0.5;
                }

                goAngle = WallSmoothing(X, Y, goAngle, _surfDirections[0]);
                SetBackAsFront(goAngle, SafeDistance - distance);

                return true;
            }
            return false;
        }

        private BulletWave GetMostDangerousBullet()
        {
            return enemyBullets.Select(e => new Tuple<BulletWave, double>(e, CalculateDistanceToBullet(e)))
                    .OrderByDescending(t => t.Item2).First().Item1;
        }

        //todo: get most dangerous wave
        private BulletWave GetClosestBullet()
        {
            double closestDistance = double.MaxValue;
            BulletWave surfWave = null;

            foreach (var ew in enemyBullets)
            {
                var distance = CalculateDistanceToBullet(ew);
                if (distance > ew.Velocity && distance < closestDistance)
                {
                    surfWave = ew;
                    closestDistance = distance;
                }
            }

            return surfWave;
        }

        private double CalculateDistanceToBullet(BulletWave ew)
        {
            var traveledDistance = ew.GetTraveledDistance(Time);
            var curDistance = Helper.GetDistance(ew.StartX, ew.StartY, X, Y);
            return curDistance - traveledDistance;
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
            Helper.UpdateStats(index, _surfStats);
        }

        private void SetBackAsFront(double angleInRadian, double distance)
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
                SetBack(distance);
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
                SetAhead(distance);
            }
        }

        // CREDIT: mini sized predictor from Apollon, by rozu
        // http://robowiki.net?Apollon
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
                predictedVelocity += (predictedVelocity * moveDir < 0 ? 2 * moveDir : moveDir);
                predictedVelocity = Helper.Limit(-8, predictedVelocity, 8);

                // calculate the new predicted position
                predictedPosition = Helper.GetProjection(predictedPosition.X, predictedPosition.Y, predictedHeading, predictedVelocity);

                counter++;

                //todo: check if we can add _radius
                if (Helper.GetDistance(predictedPosition.X, predictedPosition.Y, surfWave.StartX, surfWave.StartY) + _radius <
                    surfWave.GetTraveledDistance(Time) + (counter + 1) * surfWave.Velocity)
                {
                    intercepted = true;
                }
            } while (!intercepted && counter < 500);

            return predictedPosition;
        }

        private double CheckDanger(BulletWave surfWave, int direction, out PointF predictPosition)
        {
            predictPosition = PredictPosition(surfWave, direction);
            int index = GetFactorIndex(surfWave, predictPosition.X, predictPosition.Y);
            return _surfStats[index];
        }

        private double WallSmoothing(double curX, double curY, double angleInRadian, int offsetTurn)
        {
            while (!_innerField.Contains(Helper.GetProjection(curX, curY, angleInRadian, WallStick)))
            {
                angleInRadian += offsetTurn * 0.1;
            }
            return angleInRadian;
        }

        public override void OnWin(WinEvent evnt)
        {
            while (true)
            {
                ScanColor = Color.Black;
                TurnRadarLeft(45);
                Scan();
                ScanColor = Color.White;
                TurnRadarRight(45);
                Scan();
            }
        }

        public override void OnHitByBullet(HitByBulletEvent e)
        {
            // If the _enemyWaves collection is empty, we must have missed the
            // detection of this wave somehow.

            BulletWave hitWave = null;
            foreach (var ew in enemyBullets)
            {
                if (Math.Abs(ew.GetTraveledDistance(Time) - Helper.GetDistance(X, Y, ew.StartX, ew.StartY)) <= _radius &&
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

            if (medKitLocked)
            {
                SetBackAsFront(HeadingRadians + Math.PI, rnd.Next(50, 100));
            }
        }

        public override void OnHitRobot(HitRobotEvent e)
        {
            _lastEnemyEnergy -= 0.6;
        }

        public override void OnBulletHit(BulletHitEvent e)
        {
            _lastEnemyEnergy = e.VictimEnergy;

            foreach (var bullet in myBullets)
            {
                if (Math.Abs(bullet.GetTraveledDistance(Time) - Helper.GetDistance(e.Bullet.X, e.Bullet.Y, bullet.StartX, bullet.StartY)) <= _radius &&
                    Math.Abs(e.Bullet.Power - bullet.Power) < 0.001 && Math.Abs(e.Bullet.HeadingRadians - bullet.Angle) < 0.001)
                {
                    bullet.CheckHit(e.Bullet.X, e.Bullet.Y, Time); //maybe add more weight
                    break;
                }
            }

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
                    break;
                }
            }
        }

        public override void OnStatus(StatusEvent e)
        {
            if (Time > 10 && _radarCount > 0)
                _radarCount--;

            //            if (medKitTurnPast % 500 == 0)
            //            {
            //                Out.WriteLine("status:" + Time);
            //                medKitTurnPast = 0;
            //                medKitAppeared = true;
            //                medKitLocked = false;
            //            }

            //medKitTurnPast++;
        }

        public override void OnRoundEnded(RoundEndedEvent evnt)
        {
            round++;
            //            Out.WriteLine("The round has ended");

            //WriteRound();
            //WriteStats();
        }

        public override void OnBattleEnded(BattleEndedEvent evnt)
        {
            medKitTurnPast = 0;
            //WriteRound();
        }

        private void WriteStats()
        {
            try
            {
                using (var writer = new StreamWriter(GetDataFile(FileName)))
                {
                    foreach (var stat in _surfStats)
                    {
                        writer.Write(stat + " ");
                    }

                    foreach (var stats in _fireStats)
                    {
                        writer.WriteLine();
                        foreach (var stat in stats)
                        {
                            writer.Write(stat + " ");
                        }
                    }
                }

            }
            catch (Exception)
            {
                Out.WriteLine("exception in writing");
            }
        }

        private void ReadStats()
        {
            var surfStats = new double[StatsCount];
            var fireStats = new double[DistanceIndexCount][];
            for (int i = 0; i < fireStats.Length; i++)
            {
                fireStats[i] = new double[StatsCount];
            }

            try
            {
                bool isSurf = true;
                int j = 0;
                using (var reader = new StreamReader(GetDataFile(FileName)))
                {
                    while (!reader.EndOfStream)
                    {
                        var line = reader.ReadLine().Trim();
                        if (!string.IsNullOrEmpty(line))
                        {
                            var stats = line.Split(' ');

                            if (isSurf)
                            {
                                isSurf = false;
                                for (int i = 0; i < stats.Length; i++)
                                {
                                    surfStats[i] = double.Parse(stats[i]);
                                }
                            }
                            else
                            {
                                for (int i = 0; i < stats.Length; i++)
                                {
                                    fireStats[j][i] = double.Parse(stats[i]);
                                }
                                j++;
                            }
                        }
                    }
                }
            }
            catch (Exception)
            {
                Out.WriteLine("exception in reading");
                return;
            }

            _fireStats = fireStats;
            _surfStats = surfStats;
        }

        //        private void ReadRound()
        //        {
        //            try
        //            {
        //                using (var reader = new StreamReader(GetDataFile(FileName)))
        //                {
        //                    var str = reader.ReadLine();
        //                    if (!string.IsNullOrEmpty(str))
        //                    {
        //                        medKitTurnPast = int.Parse(str);
        //                    }
        //                }
        //            }
        //            catch (Exception)
        //            {
        //                Out.WriteLine("exception in reading");
        //            }
        //        }
        //
        //        private void WriteRound()
        //        {
        //            try
        //            {
        //                using (var writer = new StreamWriter(GetDataFile(FileName)))
        //                {
        //                    writer.Write(medKitTurnPast);
        //                }
        //            }
        //            catch (Exception)
        //            {
        //                Out.WriteLine("exception in writing");
        //            }
        //        }
        public override void OnPaint(IGraphics g)
        {
            if (!enemyBullets.Any()) return;

            var danger = GetClosestBullet();

            var pen1 = new Pen(Color.Green);
            var pen2 = new Pen(Color.Red);
            var pen3 = new Pen(Color.DarkRed);
            var hitRadius = 5f;
            var hitRadius2 = 3f;
            //draw waves
            foreach (var ew in enemyBullets)
            {

                var dist = ew.GetTraveledDistance(Time);
                var pen = ew.Equals(danger) ? pen2 : pen1;
                g.DrawEllipse(pen, (float)ew.StartX - (float)dist, (float)ew.StartY - (float)dist, 2 * (float)dist, 2 * (float)dist);

                if (ew.Equals(danger))
                {
                    g.DrawEllipse(pen3, _predictPoint.X - hitRadius, _predictPoint.Y - hitRadius, 2 * hitRadius, 2 * hitRadius);
                    g.DrawEllipse(pen3, _predictPoint.X - hitRadius2, _predictPoint.Y - hitRadius2, 2 * hitRadius2, 2 * hitRadius2);


                    var length = Helper.GetDistance(ew.StartX, ew.StartY, _predictPoint.X, _predictPoint.Y);
                    var ratio = dist / length;
                    var x = (_predictPoint.X - ew.StartX) * ratio + ew.StartX;
                    var y = (_predictPoint.Y - ew.StartY) * ratio + ew.StartY;
                    g.DrawLine(pen, (float)ew.StartX, (float)ew.StartY, (float)x, (float)y);
                }
            }

        }
    }
}
