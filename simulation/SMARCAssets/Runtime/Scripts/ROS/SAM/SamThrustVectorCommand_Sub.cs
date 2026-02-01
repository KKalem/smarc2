using UnityEngine;

using ROS.Subscribers;
using VehicleComponents.Actuators;
using RosMessageTypes.Sam;

namespace ROS.SAM
{

    public enum ThrustVectorAxis
    {
        Pitch,
        Yaw
    }

    [RequireComponent(typeof(Hinge))]
    public class SamThrustVectorCommand_Sub : Actuator_Sub<ThrusterAnglesMsg>
    {
        Hinge hinge;
        public ThrustVectorAxis axis;
        

        void Awake()
        {
            hinge = GetComponent<Hinge>();
        }

        protected override void UpdateVehicle(bool reset)
        {
            if(hinge == null)
            {
                Debug.Log($"Thrust Vector Command Sub found no Hinge to command! Disabling.");
                enabled = false;
                rosCon.Unsubscribe(topic);
                return;
            }
            
            if(reset)
            {
                hinge.SetAngle(0);
                return;
            }
            if(axis == ThrustVectorAxis.Pitch)
                hinge.SetAngle(ROSMsg.thruster_vertical_radians);
            else if(axis == ThrustVectorAxis.Yaw)
                hinge.SetAngle(ROSMsg.thruster_horizontal_radians);
        }
    }
}