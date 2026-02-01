using UnityEngine;
using UnityEngine.InputSystem;
using Propeller = VehicleComponents.Actuators.Propeller;


namespace SmarcGUI.KeyboardControllers
{
    public class BlueROVKeyboardController : KeyboardControllerBase
    {
        [Header("Basics")]
        public float moveRpms = 500f;

        [Header("Props")]
        public Propeller FrontLeftProp;
        public Propeller FrontRightProp;
        public Propeller BackLeftProp;
        public Propeller BackRightProp;
        public Propeller MiddleRightProp;
        public Propeller MiddleLeftProp;
        public Propeller HorizontalRightFrontProp;
        public Propeller HorizontalRightBackProp;
        public Propeller HorizontalLeftFrontProp;
        public Propeller HorizontalLeftBackProp;


        InputAction forwardAction, strafeAction, verticalAction, rollAction, tvAction;


        void Start()
        {
            forwardAction = InputSystem.actions.FindAction("Robot/Forward");
            strafeAction = InputSystem.actions.FindAction("Robot/Strafe");
            verticalAction = InputSystem.actions.FindAction("Robot/UpDown");
            rollAction = InputSystem.actions.FindAction("Robot/Roll");
            tvAction = InputSystem.actions.FindAction("Robot/ThrustVector");
        }

        void Update()
        {
            float forwardInput = forwardAction.ReadValue<float>();
            float strafeInput = strafeAction.ReadValue<float>();
            float verticalInput = verticalAction.ReadValue<float>();
            float rollInput = rollAction.ReadValue<float>();
            var tv = tvAction.ReadValue<Vector2>();
            float yawInput = tv.x;
            float pitchInput = tv.y;

            float frontLeftRpm = (forwardInput + strafeInput + yawInput) * moveRpms;
            float backLeftRpm = (-forwardInput + strafeInput - yawInput) * moveRpms;
            float frontRightRpm = (forwardInput - strafeInput - yawInput) * moveRpms;
            float backRightRpm = (-forwardInput - strafeInput + yawInput) * moveRpms;
            float middleRightRpm = (verticalInput + rollInput) * moveRpms;
            float middleLeftRpm = (verticalInput - rollInput) * moveRpms;
            float horizontalRightFrontRpm = (verticalInput + rollInput + pitchInput) * moveRpms;
            float horizontalRightBackRpm = (verticalInput + rollInput - pitchInput) * moveRpms;
            float horizontalLeftFrontRpm = (verticalInput - rollInput + pitchInput) * moveRpms;
            float horizontalLeftBackRpm = (verticalInput - rollInput - pitchInput) * moveRpms;

            FrontLeftProp.SetRpm(frontLeftRpm);
            FrontRightProp.SetRpm(frontRightRpm);
            BackLeftProp.SetRpm(backLeftRpm);
            BackRightProp.SetRpm(backRightRpm);
            MiddleRightProp.SetRpm(middleRightRpm);
            MiddleLeftProp.SetRpm(middleLeftRpm);
            HorizontalRightFrontProp.SetRpm(horizontalRightFrontRpm);
            HorizontalRightBackProp.SetRpm(horizontalRightBackRpm);
            HorizontalLeftFrontProp.SetRpm(horizontalLeftFrontRpm);
            HorizontalLeftBackProp.SetRpm(horizontalLeftBackRpm);
        }

        public override void OnReset()
        {
            FrontLeftProp.SetRpm(0);
            FrontRightProp.SetRpm(0);
            BackLeftProp.SetRpm(0);
            BackRightProp.SetRpm(0);
            MiddleRightProp.SetRpm(0);
            MiddleLeftProp.SetRpm(0);
            HorizontalRightFrontProp.SetRpm(0);
            HorizontalRightBackProp.SetRpm(0);
            HorizontalLeftFrontProp.SetRpm(0);
            HorizontalLeftBackProp.SetRpm(0);
        }
    }


}