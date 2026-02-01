using UnityEngine;
using RosMessageTypes.Mavros;
using ROS.Subscribers;
using VehicleComponents.Actuators;

namespace Brov2.Mavros
{
    public class BROV2MavrosRCIn_Sub : Actuator_Sub<RCInMsg>
    {
        [Header("Mavros RCIn Settings")]
        [Tooltip("Match the propellers to the RC channels as per your configuration.")]
        public Propeller P1; 
        public Propeller P2, P3, P4, P5, P6, P7, P8;
        public Propeller[] props;

        [Tooltip("Map RC value to RPM. Adjust these values based on your RC transmitter settings.")]
        public int PropRcMin = 1400;
        public int PropRcMax = 1600;
        public float PropMaxRPM = 500f;

        public int CameraTiltRcMin = 1100;
        public int CameraTiltRcMax = 1900;
        public int CameraTiltChannel = 14;
        public Hinge CameraTiltHinge;

        private float MapRCToActuation(int rcValue, int min, int max, float actuationMax)
        {
            float normalized = Mathf.Clamp01((rcValue - min) / (float)(max - min));
            return (normalized * 2f - 1f) * actuationMax; // Map to [-actuationMax, actuationMax]
        }

        void Awake()
        {
            props = new Propeller[] { P1, P2, P3, P4, P5, P6, P7, P8 };
        }


        protected override void UpdateVehicle(bool reset)
        {
            if (P1 == null || P2 == null || P3 == null || P4 == null ||
               P5 == null || P6 == null || P7 == null || P8 == null)
            {
                Debug.LogError("BROV2MavrosRCIn: One or more propellers not assigned! Disabling script.");
                enabled = false;
                return;
            }

            if (CameraTiltHinge == null)
            {
                Debug.LogError("BROV2MavrosRCIn: CameraTiltHinge not assigned! Disabling script.");
                enabled = false;
                return;
            }

            if (reset)
            {
                for (int i = 0; i < props.Length; i++) props[i].SetRpm(0);
                CameraTiltHinge.SetAngle(0);
                return;
            }

            // Assuming RC channels 0-7 map to propellers P1-P8 respectively
            for (int i = 0; i < props.Length; i++)
            {
                props[i].SetRpm(MapRCToActuation(ROSMsg.channels[i], PropRcMin, PropRcMax, PropMaxRPM));
            }
            
            // Assuming RC channel 9 maps to camera tilt
            CameraTiltHinge.SetAngle(MapRCToActuation(ROSMsg.channels[CameraTiltChannel], CameraTiltRcMin, CameraTiltRcMax, CameraTiltHinge.AngleMax));
        }
    }
}