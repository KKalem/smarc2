using UnityEngine;

using ROS.Subscribers;
using VehicleComponents.Actuators;
using RosMessageTypes.Smarc;

namespace ROS.SAM
{
    [RequireComponent(typeof(Propeller))]
    public class SamThrusterCommand_Sub : Actuator_Sub<ThrusterRPMMsg>
    {
        Propeller prop;

        void Awake()
        {
            prop = GetComponent<Propeller>();
        }

        protected override void UpdateVehicle(bool reset)
        {
            if(prop == null)
            {
                Debug.Log($"Thruster Command Sub found no Propeller to command! Disabling.");
                enabled = false;
                rosCon.Unsubscribe(topic);
                return;
            }
            
            if(reset)
            {
                prop.SetRpm(0);
                return;
            }
            prop.SetRpm(ROSMsg.rpm);
        }
    }
}