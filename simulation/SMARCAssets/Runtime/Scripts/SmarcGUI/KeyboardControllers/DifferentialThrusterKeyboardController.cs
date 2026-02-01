using UnityEngine;
using UnityEngine.InputSystem;

using Propeller = VehicleComponents.Actuators.Propeller;

namespace SmarcGUI.KeyboardControllers
{
    public class DifferentialThrusterKeyboardController : KeyboardControllerBase
    {

        public Propeller propLeft, propRight;
        public float moveRpms = 1500f;


        InputAction forwardAction, tvAction;
        

        void Awake()
        {
            forwardAction = InputSystem.actions.FindAction("Robot/Forward");
            tvAction = InputSystem.actions.FindAction("Robot/ThrustVector");
        }

        void Update()
        {
            var forwardValue = forwardAction.ReadValue<float>();
            var tv = tvAction.ReadValue<Vector2>();
            var rudderValue = tv.x;

            propLeft.SetRpm((forwardValue + rudderValue) * moveRpms);
            propRight.SetRpm((forwardValue - rudderValue) * moveRpms);
        }

        public override void OnReset()
        {
            propLeft.SetRpm(0f);
            propRight.SetRpm(0f);
        }

    }
}