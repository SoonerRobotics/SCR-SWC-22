﻿using UnityEngine;

namespace RosSharp.RosBridgeClient.MessageTypes.Swc
{
    public class GPSPublisher : UnityPublisher<GPS>
    {
        private GPS message;

        private float latNoiseStdDev = 1.843f;
        private float lonNoiseStdDev = 2.138f;

        private float lat0Pos = 35.205853f;
        private float lon0Pos = -97.442325f;

        private float previousScanTime = 5;
        private float updatePeriod = 0.1f;

        public bool noNoiseOverride = false;

        protected override void Start()
        {
            base.Start();
            message = new GPS();

            switch (ConfigLoader.competition.NoiseLevel) {
                case ConfigLoader.CompetitionConfig.NoiseLevels.none:
                    latNoiseStdDev *= 0;
                    lonNoiseStdDev *= 0;
                    break;
                case ConfigLoader.CompetitionConfig.NoiseLevels.reduced:
                    latNoiseStdDev *= 0.5f;
                    lonNoiseStdDev *= 0.5f;
                    break;
            }

            if (noNoiseOverride) {
                latNoiseStdDev = 0;
                lonNoiseStdDev = 0;
            }
        }

        private void FixedUpdate()
        {
            if (UnityEngine.Time.time >= previousScanTime + updatePeriod)
            {
                WriteMessage();
                previousScanTime = UnityEngine.Time.time;
            }
        }

        private void WriteMessage() {
            Vector3 pos = this.transform.position;
            message.lat = (pos.x + SimUtils.getRandNormal(0, latNoiseStdDev)) / 110944.33 + lat0Pos;
            message.lon = (pos.z + SimUtils.getRandNormal(0, lonNoiseStdDev)) / 91058.93 + lon0Pos;
            Publish(message);
        }
    }
}