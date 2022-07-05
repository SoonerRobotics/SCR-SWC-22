﻿using UnityEngine;

namespace RosSharp.RosBridgeClient.MessageTypes.Std
{
    [RequireComponent(typeof(BoatController))]
    public class VelocityPublisher : UnityPublisher<Float32>
    {
        private Float32 message;

        private float velocityNoiseStdDev = 0.1f;

        private float previousScanTime = 5;
        private float updatePeriod = 0.05f;
        private BoatController c;

        public bool noNoiseOverride = false;

        protected override void Start()
        {
            base.Start();
            c = GetComponent<BoatController>();
            message = new Float32();

            switch (ConfigLoader.competition.NoiseLevel) {
                case ConfigLoader.CompetitionConfig.NoiseLevels.none:
                    velocityNoiseStdDev *= 0;
                    break;
                case ConfigLoader.CompetitionConfig.NoiseLevels.reduced:
                    velocityNoiseStdDev *= 0.5f;
                    break;
            }

            if (noNoiseOverride) {
                velocityNoiseStdDev = 0;
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
            message.data = Mathf.Round((c.Power + SimUtils.getRandNormal(0, velocityNoiseStdDev) * c.Power / 4.0f) * 1000f) / 1000f;
            Publish(message);
        }
    }
}