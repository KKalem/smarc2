using UnityEngine;
using UnityEngine.InputSystem;
using Hinge = VehicleComponents.Actuators.Hinge;
using Propeller = VehicleComponents.Actuators.Propeller;


namespace SmarcGUI.KeyboardControllers
{
    public class GotlandClassKeyboardController : KeyboardControllerBase
    {
        [Header("Basics")]
        public float moveRpms = 500f;

        [Header("Props")]
        public Propeller Prop;
        public Hinge X_TL, X_TR, X_BL, X_BR, ElevL, ElevR;

        InputAction forwardAction, tvAction;

        void Start()
        {
            forwardAction = InputSystem.actions.FindAction("Robot/Forward");
            tvAction = InputSystem.actions.FindAction("Robot/ThrustVector");
        }

        void Update()
        {
            float forwardInput = forwardAction.ReadValue<float>();
            var tv = tvAction.ReadValue<Vector2>();
            float yawInput = tv.x;
            float pitchInput = tv.y;

            float rpm = forwardInput * moveRpms;
            Prop.SetRpm(rpm);

            X_BL.SetAngle((-yawInput + pitchInput) * X_BL.AngleMax);
            X_TR.SetAngle((-yawInput + pitchInput) * X_TR.AngleMax);
            X_TL.SetAngle((yawInput + pitchInput) * X_TL.AngleMax);
            X_BR.SetAngle((yawInput + pitchInput) * X_BR.AngleMax);
            ElevL.SetAngle((-pitchInput) * ElevL.AngleMax);
            ElevR.SetAngle((-pitchInput) * ElevR.AngleMax);
        }

        public override void OnReset()
        {
            Prop.SetRpm(0f);
            X_TL.SetAngle(0f);
            X_TR.SetAngle(0f);
            X_BL.SetAngle(0f);
            X_BR.SetAngle(0f);
            ElevL.SetAngle(0f);
            ElevR.SetAngle(0f);
        }


    }
}