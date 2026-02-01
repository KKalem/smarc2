using UnityEngine;
using UnityEngine.InputSystem;

// Source; https://gist.github.com/FreyaHolmer/650ecd551562352120445513efa1d952
// with some mods.


namespace SmarcGUI.WorldSpace
{
	[RequireComponent( typeof(Camera) )]
	public class FlyCamera : MonoBehaviour
	{
		public float acceleration = 50; // how fast you accelerate
		public float accSprintMultiplier = 4; // how much faster you go when "sprinting"
		public float dampingCoefficient = 5; // how quickly you break to a halt after you stop your input
		public bool focusOnEnable = true; // whether or not to focus and lock cursor immediately on enable

		Vector3 velocity; // current velocity

		GUIState guiState;
		SmoothFollow smoothFollow;

		void Start()
		{
			guiState = FindFirstObjectByType<GUIState>();
			smoothFollow = GetComponent<SmoothFollow>();
		}

		static bool Focused {
			get => Cursor.lockState == CursorLockMode.Locked;
			set {
				Cursor.lockState = value ? CursorLockMode.Locked : CursorLockMode.None;
				Cursor.visible = value == false;
			}
		}

		void OnEnable() {
			if( focusOnEnable ) Focused = true;
		}

		void OnDisable() => Focused = false;

		public void EnableMouseLook(bool enable)
		{
			if( enable )
			{
				Focused = true;
				if(smoothFollow) smoothFollow.target = null;
			}
			else
			{
				Focused = false;
			}
		}

		void Update() {
			if(guiState.MouseDragging || guiState.MouseOnGUI)
			{
				Focused = false;
				return;
			}
			
			// Input
			if(Focused) UpdateInput();

			// Physics
			velocity = Vector3.Lerp( velocity, Vector3.zero, dampingCoefficient * Time.deltaTime );
			transform.position += velocity * Time.deltaTime;
		}

		void UpdateInput() 
		{
			// Position
			velocity += GetAccelerationVector() * Time.deltaTime;

			// Rotation
			// var mousePos = Mouse.current.position.ReadValue();
			// Vector2 mouseDelta = lookSensitivity * new Vector2( mousePos.x, -mousePos.y );
			Vector2 mouseDelta = guiState.FlyCameraSensitivity * new Vector2( Mouse.current.delta.x.ReadValue(), -Mouse.current.delta.y.ReadValue() );
			Quaternion rotation = transform.rotation;
			Quaternion horiz = Quaternion.AngleAxis( mouseDelta.x, Vector3.up );
			Quaternion vert = Quaternion.AngleAxis( mouseDelta.y, Vector3.right );
			transform.rotation = horiz * rotation * vert;

			// Leave cursor lock
			if( Mouse.current.rightButton.wasReleasedThisFrame )
				Focused = false;
		}

		Vector3 GetAccelerationVector() {
			Vector3 moveInput = default;

			if(Keyboard.current.wKey.isPressed) moveInput += Vector3.forward;
			if(Keyboard.current.sKey.isPressed) moveInput += Vector3.back;
			if(Keyboard.current.dKey.isPressed) moveInput += Vector3.right;
			if(Keyboard.current.aKey.isPressed) moveInput += Vector3.left;
			if(Keyboard.current.eKey.isPressed) moveInput += Vector3.up;
			if(Keyboard.current.qKey.isPressed) moveInput += Vector3.down;

			Vector3 direction = transform.TransformVector( moveInput.normalized );

			if( Keyboard.current.leftShiftKey.isPressed )
				return direction * ( acceleration * accSprintMultiplier ); // "sprinting"
			return direction * acceleration; // "walking"
		}
	}
}