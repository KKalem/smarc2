using UnityEngine;

using Unity.Robotics.Core;
using ROS.Core;

using VehicleComponents.Actuators;
using RosMessageTypes.Smarc;

namespace ROS.SAM
{
    [RequireComponent(typeof(IPercentageActuator))]
    public class SamPercentFeedback_Pub : ROSPublisher<PercentStampedMsg>
    {
        IPercentageActuator act;
        protected override void UpdateMessage()
        {
            if(act == null) act = GetComponent<IPercentageActuator>();
            ROSMsg.header.stamp = new TimeStamp(Clock.time);
            ROSMsg.value = act.GetCurrentValue();
        }
    }
}