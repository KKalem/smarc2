using UnityEngine;

using Unity.Robotics.Core;
using ROS.Core;

using VehicleComponents.Actuators;
using RosMessageTypes.Smarc;

namespace ROS.SAM
{
    [RequireComponent(typeof(Propeller))]
    public class SamThrusterFeedback_Pub : ROSPublisher<ThrusterFeedbackMsg>
    {
        Propeller prop;
        protected override void UpdateMessage()
        {
            if(prop == null) prop = GetComponent<Propeller>();
            ROSMsg.rpm.rpm = (int)prop.rpm;
        }
    }
}