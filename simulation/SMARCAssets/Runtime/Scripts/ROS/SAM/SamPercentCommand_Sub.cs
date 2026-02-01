using UnityEngine;

using ROS.Subscribers;
using VehicleComponents.Actuators;
using RosMessageTypes.Smarc;

namespace ROS.SAM
{
    [RequireComponent(typeof(IPercentageActuator))]
    public class SamPercentCommand_Sub : Actuator_Sub<PercentStampedMsg>
    {
        IPercentageActuator act;

        void Awake()
        {
            act = GetComponent<IPercentageActuator>();
        }

        protected override void UpdateVehicle(bool reset)
        {
            if(act == null)
            {
                Debug.Log($"Percent Command Sub found no Percentage Actuator to command! Disabling.");
                enabled = false;
                rosCon.Unsubscribe(topic);
                return;
            }
            
            if(reset)
            {
                act.SetPercentage(act.GetResetValue());
                return;
            }
            act.SetPercentage(ROSMsg.value);
        }
    }
}