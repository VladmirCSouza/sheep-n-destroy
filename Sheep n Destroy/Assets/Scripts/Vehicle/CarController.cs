using System;
using UnityEngine;

namespace Vehicle {
    public class CarController : MonoBehaviour {
        [Header("References")]
        [SerializeField] private Rigidbody _carRigidbody;
        [SerializeField] private Transform[] _rayPoints;
        [SerializeField] private LayerMask _drivable;
        [SerializeField] private Transform _accelerationPoint;
        [SerializeField] private GameObject[] _tiresMesh = new GameObject[4];
        [SerializeField] private GameObject[] _frontTiresParent = new GameObject[2];
        
        
        [Header("Suspension Settings")]
        [SerializeField] private float _springStiffness = 30000f;
        [Range(0, 1)][SerializeField] private float _dampingRation = 0.2f;
        [SerializeField] private float _restLenght = 1f;
        [SerializeField] private float _springTravel = 0.5f;
        [SerializeField] private float _wheelRadius = 0.33f;

        [Header("Input")] 
        [SerializeField] private float _moveInput = 0;
        [SerializeField] private float _steerInput = 0;
        
        [Header("Car Settings")]
        [SerializeField] private float _acceleration = 25f;
        [SerializeField] private float _deceleration = 10f;
        [SerializeField] private float _maxSpeed = 100f;
        [SerializeField] private float _steerStrength = 15f;
        [SerializeField] private float _dragCoefficient = 1f;
        [SerializeField] private float _longitudinalDragCoefficient = 1f;
        [SerializeField] private AnimationCurve _turningCurve;
        
        [Header("Visuals")]
        [SerializeField] private float _tireRotationSpeed = 3000f;
        [SerializeField] private float _maxStearingAngle = 30f;
        
        private Vector3 _currentCarLocalVelocity = Vector3.zero;
        private float _carVelocityRatio;
        private int[] _wheelIsGrounded = new int[4];
        private bool _isCarGrounded = false;
        private float _linearDampOnRest = 10000f;

        private VehicleInputActions _vehicleInput;

        private void Start() {
            _carRigidbody = GetComponent<Rigidbody>();
            _vehicleInput = new VehicleInputActions();
            _vehicleInput.Enable();
        }

        private void OnDestroy() {
            _vehicleInput.Disable();
        }

        private void Update() {
            GetPlayerInput();
        }

        private void FixedUpdate() {
            Suspension();
            CarGroundCheck();
            CalculateCarVelocity();
            Movement();
            TiresVisuals();
        }

        private void GetPlayerInput() {
            _moveInput = _vehicleInput.DefaultMap.Throttle.ReadValue<float>() - _vehicleInput.DefaultMap.Brake.ReadValue<float>();
            _steerInput = _vehicleInput.DefaultMap.Steering.ReadValue<float>();
        }

        private void Suspension() {
            for (var i = 0; i < _rayPoints.Length; i++) {
                var rayPoint = _rayPoints[i];
                RaycastHit hit;
                float maxDistance = _restLenght + _springTravel;

                Vector3 tireMeshTargetPosition;
                if (Physics.Raycast(rayPoint.position, -rayPoint.up, out hit, maxDistance + _wheelRadius, _drivable)) {
                    _wheelIsGrounded[i] = 1;
                    float currentSpringLenght = hit.distance - _wheelRadius;
                    float springCompression = (_restLenght - currentSpringLenght) / _springTravel;

                    float springVelocity = Vector3.Dot(_carRigidbody.GetPointVelocity(rayPoint.position), rayPoint.up);
                    float dampForce = CalculateDampingStiffness() * springVelocity;

                    float springForce = _springStiffness * springCompression;
                    float netForce = springForce - dampForce;

                    tireMeshTargetPosition = hit.point + rayPoint.up * _wheelRadius;
                    _carRigidbody.AddForceAtPosition(netForce * rayPoint.up, rayPoint.position);

                    Debug.DrawLine(rayPoint.position, hit.point, Color.red);
                } else {
                    _wheelIsGrounded[i] = 0;
                    tireMeshTargetPosition = rayPoint.position - rayPoint.up * maxDistance;
                    Debug.DrawLine(rayPoint.position, rayPoint.position + (maxDistance + _wheelRadius) * -rayPoint.up, Color.green);
                }
                SetTiresMeshPosition(_tiresMesh[i], tireMeshTargetPosition);
            }
        }

        private void CarGroundCheck() {
            int temGroundedWheels = 0;
            for (int i = 0; i < _wheelIsGrounded.Length; i++) {
                temGroundedWheels += _wheelIsGrounded[i];
            }

            _isCarGrounded = temGroundedWheels > 1;
        }
        
        private void CalculateCarVelocity() {
            _currentCarLocalVelocity = transform.InverseTransformDirection(_carRigidbody.linearVelocity);
            _carVelocityRatio = _currentCarLocalVelocity.z / _maxSpeed;
        }

        private void Movement() {
            if (_isCarGrounded) {
                Acceleration();
                Deceleration();
                Turn();
                SidewaysDrag();
                // ForwardDrag();
                // TryForceLinearDump();
                Debug.Log(_carRigidbody.linearVelocity.magnitude);
            }
        }

        private void Acceleration() {
            _carRigidbody.AddForceAtPosition(_acceleration * _moveInput * transform.forward, _accelerationPoint.position, ForceMode.Acceleration);
        }

        private void Deceleration() {
            _carRigidbody.AddForceAtPosition(_deceleration * _moveInput * -transform.forward, _accelerationPoint.position, ForceMode.Acceleration);
        }

        private void TryForceLinearDump() {
            if (_carRigidbody.linearVelocity.magnitude <= 0.01f) {
                _carRigidbody.linearDamping = _linearDampOnRest;
            } else {
                _carRigidbody.linearDamping = 0;
            }
        }
        
        private void ForwardDrag() {
            float currentForwardSpeed = _currentCarLocalVelocity.z + _carRigidbody.linearVelocity.z;
            float dragMagnitude = -currentForwardSpeed * _longitudinalDragCoefficient;
            
            Vector3 dragForce = transform.forward * dragMagnitude;
            _carRigidbody.AddForceAtPosition(dragForce, _carRigidbody.worldCenterOfMass, ForceMode.Acceleration);
        }

        private void Turn() {
            _carRigidbody.AddTorque(_steerStrength * _steerInput * _turningCurve.Evaluate(_carVelocityRatio) * Mathf.Sign(_carVelocityRatio) * transform.up, ForceMode.Acceleration);
        }

        private void SidewaysDrag() {
            float currentSidewaysSpeed = _currentCarLocalVelocity.x;
            float dragMagnitude = -currentSidewaysSpeed * _dragCoefficient;
            
            Vector3 dragForce = transform.right * dragMagnitude;
            _carRigidbody.AddForceAtPosition(dragForce, _carRigidbody.worldCenterOfMass, ForceMode.Acceleration);
        }
        
        // Usando a formula "Damping Coefficient"
        private float  CalculateDampingStiffness() {
            return 2f * Mathf.Sqrt(_springStiffness * _carRigidbody.mass) * _dampingRation;
        }


        private void SetTiresMeshPosition(GameObject tireMesh, Vector3 targetPosition) {
            tireMesh.transform.position = targetPosition;
        }

        private void TiresVisuals() {
            SetTireMeshRotation();
        }
        
        private void SetTireMeshRotation() {
            for (int i = 0; i < _tiresMesh.Length; i++) {
                if (i < 2) { //Front tires
                    _tiresMesh[i].transform.Rotate(Vector3.right, _tireRotationSpeed * _carVelocityRatio * Time.deltaTime, Space.Self);
                } else {
                    _tiresMesh[i].transform.Rotate(Vector3.right, _tireRotationSpeed * _moveInput * Time.deltaTime, Space.Self);
                }
            }

            float steeringAngle = _maxStearingAngle * _steerInput;
            for (int i = 0; i < _frontTiresParent.Length; i++) {
                _frontTiresParent[i].transform.localEulerAngles = new Vector3(_frontTiresParent[i].transform.localEulerAngles.x, steeringAngle, _frontTiresParent[i].transform.localEulerAngles.z);
            }
        }
    }
}
