using UnityEngine;
using RosMessageTypes.Mavros;
using ROS.Core;

namespace Brov2.Mavros
{
    [RequireComponent(typeof(BROV2MavrosRCIn_Sub))]
    public class BROV2MavrosRCOut_Pub : ROSPublisher<RCOutMsg>
    {

        BROV2MavrosRCIn_Sub rcInSub;
        protected override void InitPublisher()
        {
            rcInSub = GetComponent<BROV2MavrosRCIn_Sub>();
            ROSMsg.channels = new ushort[rcInSub.CameraTiltChannel + 1];
        }

        protected override void UpdateMessage()
        {
            for(int i = 0; i < rcInSub.props.Length-1; i++)
            {
                ROSMsg.channels[i] = (ushort)MapActuationToRC(rcInSub.props[i].rpm, rcInSub.PropRcMin, rcInSub.PropRcMax, rcInSub.PropMaxRPM);
            }
            ROSMsg.channels[rcInSub.CameraTiltChannel] = (ushort)MapActuationToRC(rcInSub.CameraTiltHinge.angle, rcInSub.CameraTiltRcMin, rcInSub.CameraTiltRcMax, rcInSub.CameraTiltHinge.AngleMax);
        }
        

        private int MapActuationToRC(float actuationValue, int min, int max, float actuationMax)
        {
            float normalized = (actuationValue / actuationMax + 1f) / 2f; // Map from [-actuationMax, actuationMax] to [0, 1]
            return Mathf.RoundToInt(normalized * (max - min) + min);
        }


        
    }
}