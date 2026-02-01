using UnityEngine;

using ROS.Core;

using VehicleComponents.Sensors;
using RosMessageTypes.Smarc;

namespace ROS.SAM
{
    [RequireComponent(typeof(Leak))]
    public class SamLeakFeedback_Pub : ROSPublisher<LeakMsg>
    {
        Leak leak;
        protected override void UpdateMessage()
        {
            if(leak == null) leak = GetComponent<Leak>();
           ROSMsg.value = leak.leaked;
           ROSMsg.leak_counter = leak.count;
        }
    }
}