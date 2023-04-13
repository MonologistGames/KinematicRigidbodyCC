using System;
using Monologist.Utilities.ExtendPhysics;
using UnityEngine;
using UnityEngine.Serialization;

namespace Monologist.KRCC
{
    [AddComponentMenu("Physics/KRCC/Kinematic Rigidbody CC")]
    [RequireComponent(typeof(Rigidbody))]
    [RequireComponent(typeof(CapsuleCollider))]
    public class KinematicRigidbodyCharacterController : MonoBehaviour
    {
        #region Detect cache and constant value

        // Performance settings and detection alloc
        private const int MaxAllocSize = 16;
        private const float GroundDetectOffset = 0.2f;
        private const float StepForwardOffset = 0.1f;
        private const float SweepOffset = 0.02f;

        private int _cachedOverlapsCount;
        private readonly Collider[] _cachedOverlapColliders = new Collider[MaxAllocSize];

        private int _cachedHitInfoCount;
        private readonly RaycastHit[] _cachedSweepHitInfos = new RaycastHit[MaxAllocSize];

        #endregion

        #region Transform and capsule data

        // Character transform properties
        /// <summary>
        /// Character controller's current up-axis.
        /// </summary>
        public Vector3 CharacterUp => transform.up;

        /// <summary>
        /// Character controller's current forward-axis.
        /// </summary>
        public Vector3 CharacterForward => transform.forward;

        /// <summary>
        /// Character controller's current position. 
        /// </summary>
        public Vector3 CharacterPosition => transform.position;

        /// <summary>
        /// Character controller's current rotation.
        /// </summary>
        public Quaternion CharacterRotation => transform.rotation;

        private Rigidbody _rigidbody;
        private CapsuleCollider _capsuleCollider;

        /// <summary>
        /// Radius of controller's capsule shape.
        /// </summary>
        public float CapsuleRadius => _capsuleCollider.radius;

        /// <summary>
        /// Height of controller's capsule shape.
        /// </summary>
        public float CapsuleHeight => _capsuleCollider.height;

        #endregion

        #region Function Switches

        /// <summary>
        /// Enable slope solving.
        /// </summary>
        public bool SolveSlope = true;

        /// <summary>
        /// Enable stepping.
        /// </summary>
        public bool SolveStepping = true;

        /// <summary>
        /// Enable sliding against wall.
        /// </summary>
        public bool SolveSlide = true;

        /// <summary>
        /// Enable snapping to the ground.
        /// </summary>
        public bool SnapGround = true;

        public enum InteractMode
        {
            Kinematic,
            Dynamic
        }

        /// <summary>
        /// Enable handling rigidbody pushing.
        /// </summary>
        public bool PushRigidbody = true;

        /// <summary>
        /// Mode to interact with rigidbody.
        /// - Kinematic: Push rigidbody with kinematic mode.(Regardless of mass)
        /// - Dynamic: Push rigidbody with dynamic mode.(Consider mass)
        /// </summary>
        public InteractMode PushMode = InteractMode.Dynamic;

        /// <summary>
        /// Dose rigidbody collision affect character velocity.
        /// </summary>
        public bool AffectCharacterVelocity;

        /// <summary>
        /// Ignore slope limits for slope solving process.
        /// For example, when character is doing climbing.
        /// </summary>
        [FormerlySerializedAs("IgnoreSlopeLimit")]
        public bool MustFloat;

        #endregion

        // Used for wall velocity projection
        private enum SweepMode
        {
            Initial,
            FirstSweep,
            Crease,
            Corner
        }

        #region Settings

        /// <summary>
        /// Determines how much energy lost when collided.
        /// 1 means absolute elastic collision.
        /// </summary>
        [Range(0, 1f)] public float EnergyRemainFactor = 0.3f;


        [Min(0)] public float VelocityMoveThreshold = 0.005f;

        /// <summary>
        /// How much speed will the character remain when ground on a slope.
        /// </summary>
        [Range(0, 1f)] public float GroundingSpeedRemain = 0.1f;

        /// <summary>
        /// Default gravity. Will not imply automatic.
        /// </summary>
        public Vector3 Gravity = Vector3.down * 9.81f;

        /// <summary>
        /// Max slope angle controller can climb.
        /// </summary>
        [Range(0, 90f)] public float MaxSlopeAngle = 45f;

        /// <summary>
        /// Max height character can step onto.
        /// </summary>
        [Min(0.1f)] public float MaxStepHeight = 0.3f;

        #endregion

        #region Movement Fields

        /// <summary>
        /// Current character velocity, calculated using
        /// (currentPos - previousPos) / fixed delta time.
        /// </summary>
        public Vector3 CurrentVelocity => _baseVelocity + _attachedVelocity;

        /// <summary>
        /// Current character base velocity.
        /// </summary>
        public Vector3 BaseVelocity
        {
            get => _baseVelocity;
            set => _baseVelocity = value;
        }

        private Vector3 _baseVelocity;
        private float _baseVelocityMagnitude;
        private Vector3 _attachedVelocity;

        private Quaternion _targetRotation;
        private bool _rotationDirtyMark;

        private Vector3 _remainingMoveDirection = Vector3.zero;
        private float _remainingMoveDistance;

        private Vector3 _transientPosition;
        private Quaternion _transientRotation;

        private Vector3 _motionVector;

        #endregion

        #region Ground Info

        public LayerMask GroundLayer;
        [SerializeField] private bool _isGrounded;
        public bool IsGrounded => _isGrounded;
        [SerializeField] private Vector3 _cachedGroundNormal = Vector3.up;
        public Vector3 GroundNormal => _cachedGroundNormal;

        #endregion

        // Moving platform part

        #region Moving Platform

        [SerializeField] private DynamicPlatform _attachedDynamicPlatform;
        private Vector3 _relativePosition;
        private Quaternion _relativeRotation;

        #endregion

        #region MonoBehavours

        private void OnEnable()
        {
            _rigidbody = GetComponent<Rigidbody>();
            _rigidbody.isKinematic = true;
            _capsuleCollider = GetComponent<CapsuleCollider>();
        }

        private void FixedUpdate()
        {
            _transientPosition = CharacterPosition;
            _transientRotation = CharacterRotation;

            SyncWithAttachedDynamicPlatform(ref _transientPosition, ref _transientRotation);

            // Rotate
            if (_rotationDirtyMark)
            {
                _transientRotation = _targetRotation;
            }

            // Movement Iteration
            // Overlap multiple colliders and resolve penetration
            InitialOverlap(ref _transientPosition, _transientRotation);

            if (_motionVector != Vector3.zero)
            {
                _baseVelocityMagnitude = _motionVector.magnitude;
                if (_isGrounded && SnapGround)
                    _motionVector = Vector3.ProjectOnPlane(_motionVector, _cachedGroundNormal);
                _remainingMoveDirection = _motionVector.normalized;
                _remainingMoveDistance = _baseVelocityMagnitude;

                SweepMode sweepMode = SweepMode.Initial;
                Vector3 previousNormal = Vector3.zero;
                Vector3 previousDirection = Vector3.zero;

                for (int i = 0; i < Physics.defaultSolverIterations; i++)
                {
                    if (!SafeMoveUpdate(ref _transientPosition, _transientRotation, ref _remainingMoveDirection,
                            ref _remainingMoveDistance,
                            ref sweepMode, ref previousNormal, ref previousDirection))
                    {
                        break;
                    }

                    if (_remainingMoveDistance <= 0) break;
                }

                _baseVelocityMagnitude -= _remainingMoveDistance;
            }
            else
            {
                _baseVelocityMagnitude = 0f;
            }

            ProbeGround(ref _transientPosition, _transientRotation);
            _baseVelocity =
                _remainingMoveDirection.normalized
                * _baseVelocityMagnitude / Time.deltaTime;

            // Apply transient position
            _rigidbody.MovePosition(_transientPosition);
            _rigidbody.MoveRotation(_transientRotation.normalized);

            _motionVector = Vector3.zero; // Clear motion vector
            _rotationDirtyMark = false; // Clear rotation dirty mark

            SaveRelativePositionAndRotation(_transientPosition, _transientRotation);
        }

        #endregion

        #region Internal Functions

        // TODO: Change another way to solve platform moving.

        #region Solve Dynamic Plartform

        /// <summary>
        /// Before doing collide and slide iterations, 
        /// move the character along with moving platform attached to the desired position and rotation.
        /// </summary>
        private void SyncWithAttachedDynamicPlatform(ref Vector3 transientPosition, ref Quaternion transientRotation)
        {
            _attachedVelocity = Vector3.zero;
            if (!_attachedDynamicPlatform) return;

            // Calculate absolute velocity for rigidbody interaction
            Vector3 targetPosition = _attachedDynamicPlatform.GetSyncedPosition(_relativePosition);
            _attachedVelocity = (targetPosition - CharacterPosition) / Time.fixedDeltaTime;
            transientPosition = targetPosition;

            // Change position with moving platform
            transientRotation = _attachedDynamicPlatform.GetSyncedRotation(_relativeRotation, CharacterUp);
        }

        /// <summary>
        /// Save relative position and rotation for next fixed update Frame.
        /// </summary>
        private void SaveRelativePositionAndRotation(Vector3 transientPosition, Quaternion transientRotation)
        {
            if (!_attachedDynamicPlatform) return;

            _relativePosition = _attachedDynamicPlatform.GetRelativePosition(transientPosition);
            _relativeRotation = _attachedDynamicPlatform.GetRelativeRotation(transientRotation);
        }

        #endregion


        /// <summary>
        /// Probe ground and cache ground normals, and snap to the ground.
        /// </summary>
        /// <param name="transientPosition">Transient character position.</param>
        /// <param name="transientRotation">Transient character rotation.</param>
        private void ProbeGround(ref Vector3 transientPosition,
            Quaternion transientRotation)
        {
            _cachedHitInfoCount = CharacterSweepTestAll(transientPosition, transientRotation, -CharacterUp,
                MaxStepHeight,
                out var isStartPenetrated, out var closetHitInfo, GroundLayer, _cachedSweepHitInfos, IsColliderValid,
                SweepOffset);

            _isGrounded = false;
            if (_cachedHitInfoCount <= 0 || isStartPenetrated) return;
            if (!IsStableOnNormal(closetHitInfo.normal, transientRotation)) return;

            // Ground not probed
            if (closetHitInfo.distance > GroundDetectOffset) return;

            // Cache ground information
            _cachedGroundNormal = closetHitInfo.normal;
            if (!IsStableOnNormal(closetHitInfo.normal, transientRotation)) return;
            _isGrounded = true;

            if (!SnapGround) return;
            // Snap to ground
            transientPosition -= CharacterUp * Mathf.Max(0, closetHitInfo.distance - 3 * Physics.defaultContactOffset);
        }

        #region Safe Move Update

        /// <summary>
        /// Overlap to resolve penetration at initial place.
        /// </summary>
        /// <param name="transientPosition">Character position.</param>
        /// <param name="transientRotation">Character rotation.</param>
        private void InitialOverlap(ref Vector3 transientPosition, Quaternion transientRotation)
        {
            _cachedOverlapsCount = CharacterOverlap(transientPosition, transientRotation, GroundLayer
                , _cachedOverlapColliders);
            if (_cachedOverlapsCount <= 0) return;

            for (int i = 0; i < _cachedOverlapsCount; i++)
            {
                ResolvePenetration(_cachedOverlapColliders[i], ref transientPosition);
            }
        }

        /// <summary>
        /// Get penetrated interactable rigidbody and correct the rigidbody velocity
        /// considering mass ratio. 
        /// </summary>
        /// <param name="transientVelocity">Transient character velocity, used to push interactable rigidbody.</param>
        /// <param name="transientPosition">Transient character position.</param>
        /// <param name="transientRotation">Transient character rotation.</param>
        /// <returns>Is rigidbody interacted.</returns>
        private bool InteractRigidbody(ref Vector3 transientVelocity, Vector3 transientPosition,
            Quaternion transientRotation)
        {
            // Get overlapped colliders
            _cachedOverlapsCount = CharacterOverlap(transientPosition, transientRotation, GroundLayer,
                _cachedOverlapColliders);
            bool rigidbodyInteractMark = false;

            if (_cachedOverlapsCount <= 0) return false;
            
            for (int i = _cachedOverlapsCount - 1; i >= 0; i--)
            {
                Rigidbody interactBody = _cachedOverlapColliders[i].attachedRigidbody;
                if (!interactBody || interactBody.isKinematic) continue;

                rigidbodyInteractMark = true;
                Vector3 bodyVelocity = interactBody.velocity;
                Vector3 characterVelocity = transientVelocity + _attachedVelocity;
                float characterToBodyMassRatio = 1f / (_rigidbody.mass + interactBody.mass);

                // Correct hit point
                Vector3 hitPoint = transientPosition;

                if (Physics.ComputePenetration(_capsuleCollider, transientPosition, transientRotation,
                        _cachedOverlapColliders[i], _cachedOverlapColliders[i].transform.position,
                        _cachedOverlapColliders[i].transform.rotation,
                        out var hitNormal, out _))
                {
                    #region Before Collision

                    // Correct hit normal
                    Vector3 hitNormalLeft = Vector3.Cross(_isGrounded ? _cachedGroundNormal : Vector3.up, hitNormal);
                    Vector3 obstructionNormal = hitNormal = Vector3.Cross(hitNormalLeft, CharacterUp);
                    if (obstructionNormal.sqrMagnitude != 0f) hitNormal = obstructionNormal;

                    // Cast collision velocity on normal direction
                    float characterVelocityDotNormal = Vector3.Dot(characterVelocity, hitNormal);
                    float bodyVelocityDotNormal = Vector3.Dot(bodyVelocity, hitNormal);

                    #endregion

                    // Continue if they don't collide
                    if (bodyVelocityDotNormal <= characterVelocityDotNormal) continue;

                    #region During Collision

                    Vector3 characterVelocityOnNormal = hitNormal * characterVelocityDotNormal;
                    Vector3 bodyVelocityOnNormal = hitNormal * bodyVelocityDotNormal;

                    // Change velocity for character and rigidbody
                    Vector3 interactBodyGetVelocity = (characterVelocityOnNormal - bodyVelocityOnNormal)
                                                      * (2 * _rigidbody.mass * characterToBodyMassRatio *
                                                         EnergyRemainFactor);
                    if (PushMode == InteractMode.Kinematic)
                    {
                        interactBodyGetVelocity = characterVelocityOnNormal - bodyVelocityOnNormal;
                    }

                    // Skip pushing a very heavy rigid-body to avoid twitching
                    if (interactBodyGetVelocity.magnitude * Time.deltaTime < VelocityMoveThreshold)
                    {
                        continue;
                    }

                    interactBody.AddForceAtPosition(interactBodyGetVelocity,
                        hitPoint, ForceMode.VelocityChange);

                    if (PushMode == InteractMode.Dynamic)
                        transientVelocity += (bodyVelocityOnNormal - characterVelocityOnNormal)
                                             * (2 * interactBody.mass * characterToBodyMassRatio * EnergyRemainFactor);

                    #endregion
                }
            }

            return rigidbodyInteractMark;
        }

        /// <summary>
        /// Move the character safely, including functions:
        /// -Overlap and resolve penetration
        /// -Stepping detection
        /// -Slope projection
        /// -Slide on walls 
        /// </summary>
        /// <param name="transientPosition">Transient position.</param>
        /// <param name="transientRotation">Transient rotation.</param>
        /// <param name="transientDirection">Transient moving direction.</param>
        /// <param name="transientDistance">Transient remaining distance.</param>
        /// <param name="sweepMode">A sweep test token, used for wall slide detection.</param>
        /// <param name="previousNormal">Surface normal of last iteration.</param>
        /// <param name="previousDirection">Transient direction of last iteration.</param>
        private bool SafeMoveUpdate(ref Vector3 transientPosition, Quaternion transientRotation,
            ref Vector3 transientDirection,
            ref float transientDistance, ref SweepMode sweepMode, ref Vector3 previousNormal,
            ref Vector3 previousDirection)
        {
            // Sweep test
            _cachedHitInfoCount =
                CharacterSweepTestAll(transientPosition, transientRotation, transientDirection, transientDistance,
                    out var isStartPenetrated, out var closetHitInfo, GroundLayer, _cachedSweepHitInfos,
                    IsColliderValid, Physics.defaultContactOffset);

            // Move without blocking hit
            if (_cachedHitInfoCount <= 0)
            {
                transientPosition += transientDirection * transientDistance;
                transientDistance = 0f;
                return true;
            }

            // Apply movement for this iteration
            if (!((closetHitInfo.collider.attachedRigidbody &&
                   !closetHitInfo.collider.attachedRigidbody.isKinematic)))
                closetHitInfo.distance -= Physics.defaultContactOffset;
            closetHitInfo.distance = Mathf.Max(0, closetHitInfo.distance);
            transientPosition +=
                transientDirection * closetHitInfo.distance;
            transientDistance -= closetHitInfo.distance;

            #region Interact Rigidbodies

            // Interact with dynamic rigidBody
            if (PushRigidbody)
            {
                Vector3 velocityAfterCollision = transientDirection * (_baseVelocityMagnitude / Time.deltaTime);
                float velocityBeforeCollisionMagnitude = velocityAfterCollision.magnitude;

                if (InteractRigidbody(ref velocityAfterCollision, transientPosition, transientRotation))
                {
                    if (AffectCharacterVelocity)
                    {
                        // Keep player on ground with character velocity changed
                        velocityAfterCollision = _isGrounded
                            ? Vector3.ProjectOnPlane(velocityAfterCollision, _cachedGroundNormal)
                            : velocityAfterCollision;

                        transientDistance *= velocityAfterCollision.magnitude / velocityBeforeCollisionMagnitude;
                        transientDirection = velocityAfterCollision.normalized;
                        _baseVelocityMagnitude = velocityAfterCollision.magnitude * Time.deltaTime;
                    }

                    return false;
                }
            }

            #endregion

            #region Depenetration

            // Resolve penetration
            if (isStartPenetrated)
            {
                for (int i = 0; i < _cachedHitInfoCount; i++)
                {
                    if (!IsPenetratedAtStart(_cachedSweepHitInfos[i])) continue;

                    ResolvePenetration(_cachedSweepHitInfos[i].collider, ref transientPosition);
                }
            }

            #endregion

            #region Slope Solving

            if (SolveSlope)
            {
                // Try slope
                bool isStable = IsStableOnNormal(closetHitInfo.normal, transientRotation);
                if (isStable || MustFloat)
                {
                    // Kill grounding remaining velocity
                    if (!_isGrounded && !MustFloat)
                        transientDistance *= GroundingSpeedRemain;

                    if (!MustFloat)
                    {
                        _isGrounded = true;
                        _cachedGroundNormal = closetHitInfo.normal;
                    }

                    transientDirection = Vector3.ProjectOnPlane(transientDirection, _cachedGroundNormal).normalized;
                    sweepMode = SweepMode.Initial;
                    return true;
                }
            }

            #endregion

            #region Step Solving

            if (SolveStepping)
            {
                // Try stepping
                if (DetectStep(ref transientPosition, transientDirection, transientRotation))
                {
                    transientDirection = Vector3.ProjectOnPlane(transientDirection, _cachedGroundNormal).normalized;
                    sweepMode = SweepMode.Initial;
                    return true;
                }
            }

            #endregion

            #region Slide Solving

            // Try Slide
            if (SolveSlide)
            {
                previousDirection = transientDirection.normalized;
                previousNormal = closetHitInfo.normal;

                SlideAlongSurface(closetHitInfo.normal, previousNormal, ref transientDirection, previousDirection,
                    ref sweepMode);

                transientDistance *= transientDirection.magnitude;
                transientDirection = transientDirection.normalized;

                return true;
            }

            #endregion

            return false;
        }

        /// <summary>
        /// Check if a collider of the RaycastHit is penetrated at sweep start.
        /// </summary>
        /// <param name="hit">RaycastHit returned by sweep test.</param>
        /// <returns>Is the collider penetrated.</returns>
        private bool IsPenetratedAtStart(RaycastHit hit)
        {
            if (hit.collider.attachedRigidbody && !hit.collider.attachedRigidbody.isKinematic)
                return false;
            if (hit.distance <= 0)
                return true;

            return false;
        }

        /// <summary>
        /// Resolve collider penetrations.
        /// </summary>
        /// <param name="penetratedCollider">Collider penetrated with character collider.</param>
        /// <param name="transientPosition">Transient position for this iteration.</param>
        private void ResolvePenetration(Collider penetratedCollider, ref Vector3 transientPosition)
        {
            // Compute penetration
            if (Physics.ComputePenetration(_capsuleCollider, transientPosition, CharacterRotation,
                    penetratedCollider, penetratedCollider.transform.position,
                    penetratedCollider.transform.rotation,
                    out var resolveDirection, out var resolveDistance))
            {
                transientPosition += resolveDirection.normalized * (resolveDistance + Physics.defaultContactOffset);
            }
        }

        /// <summary>
        /// Detect Step action ahead, if valid raise the player up.
        /// </summary>
        /// <param name="transientPosition">Transient character position.</param>
        /// <param name="transientDirection">Transient moving direction.</param>
        /// <param name="transientRotation">Transient character rotation.</param>
        /// <returns>Is step action valid.</returns>
        private bool DetectStep(ref Vector3 transientPosition, Vector3 transientDirection, Quaternion transientRotation)
        {
            Vector3 stepTraceStart = transientPosition + transientDirection * StepForwardOffset +
                                     CharacterUp * MaxStepHeight;
            _cachedHitInfoCount = CharacterSweepTestAll(stepTraceStart, transientRotation, -CharacterUp,
                MaxStepHeight, out var isStartPenetrated, out var closetHitInfo, GroundLayer,
                _cachedSweepHitInfos, IsColliderValid, SweepOffset);

            if (_cachedHitInfoCount <= 0 || isStartPenetrated) return false;

            bool isStable = IsStableOnNormal(closetHitInfo.normal, transientRotation);
            if (!isStable) return false;

            _isGrounded = true;
            _cachedGroundNormal = closetHitInfo.normal;
            transientPosition =
                stepTraceStart - CharacterUp * (closetHitInfo.distance - Physics.defaultContactOffset * 3);

            return true;
        }


        /// <summary>
        /// Project velocity on surface depending on surface's stability.
        /// </summary>
        /// <param name="velocity">Velocity to project.</param>
        /// <param name="normal">Surface normal.</param>
        /// <returns>Projected velocity.</returns>
        private Vector3 ProjectVelocityOnSurface(Vector3 velocity, Vector3 normal)
        {
            if (!_isGrounded)
                return Vector3.ProjectOnPlane(velocity, normal);

            Vector3 groundNormal = _cachedGroundNormal;
            // Project aside surface
            Vector3 normalRight = Vector3.Cross(normal, groundNormal).normalized;
            return Vector3.Project(velocity, normalRight);
        }

        /// <summary>
        /// Doing wall velocity projections.
        /// </summary>
        /// <param name="normal">Current hit wall normal.</param>
        /// <param name="previousNormal">Previous hit wall normal.</param>
        /// <param name="transientDirection">Transient move direction.</param>
        /// <param name="previousDirection">Previous velocity.</param>
        /// <param name="sweepMode">Enum sweep mode, used for one wall and two walls.</param>
        private void SlideAlongSurface(Vector3 normal, Vector3 previousNormal, ref Vector3 transientDirection,
            Vector3 previousDirection, ref SweepMode sweepMode)
        {
            // Project on one wall
            if (sweepMode == SweepMode.Initial)
            {
                // Get slide direction
                transientDirection = ProjectVelocityOnSurface(transientDirection, normal);
                sweepMode = SweepMode.FirstSweep;
                return;
            }

            // Project on two walls/corners
            if (sweepMode == SweepMode.FirstSweep)
            {
                Vector3 creaseDirection = Vector3.Cross(normal, previousNormal).normalized;
                float wallNormalDotResult = Vector3.Dot(previousNormal, normal);

                // Cancel calculation when two planes are the same
                if (wallNormalDotResult >= 0.999f) return;

                // Project velocity along the second wall when the angle is beyond 90 degrees
                if (wallNormalDotResult < 0)
                {
                    transientDirection = ProjectVelocityOnSurface(previousDirection, normal);
                    return;
                }

                // Project velocity to the crease restricted direction
                Vector3 creaseDirectionRight = Vector3.Cross(creaseDirection, CharacterUp);
                Vector3 creaseNormal = Vector3.Cross(creaseDirectionRight, creaseDirection).normalized;

                // If crease is too steep, stop moving
                if (!IsStableOnNormal(creaseNormal, _transientRotation))
                {
                    transientDirection = Vector3.zero;
                    sweepMode = SweepMode.Corner;
                    return;
                }

                // Restrict velocity to crease direction
                Vector3 enteringVelocity = Vector3.ProjectOnPlane(previousDirection, creaseDirection);
                if (Vector3.Dot(enteringVelocity, creaseDirection) < 0)
                {
                    creaseDirection = -creaseDirection;
                }

                transientDirection = Vector3.Project(previousDirection, creaseDirection);
                sweepMode = SweepMode.Crease;

                return;
            }

            if (sweepMode == SweepMode.Crease)
            {
                transientDirection = Vector3.zero;
                sweepMode = SweepMode.Corner;
            }
        }

        #endregion

        #endregion

        // Application Interfaces

        #region APIs

        #region Sweep Test

        /// <summary>
        /// Overlap character capsule to get collided collider.
        /// </summary>
        /// <param name="position">Overlap capsule position.</param>
        /// <param name="rotation">Overlap capsule rotation.</param>
        /// <param name="colliders">Collider cache.</param>
        /// <param name="overlapDetectionExtend">Extend capsule size to get a larger detection (usually not used).</param>
        /// <param name="queryLayer">LayerMask to query collision.</param>
        /// <returns>Count of colliders.</returns>
        public int CharacterOverlap(Vector3 position, Quaternion rotation, LayerMask queryLayer, Collider[] colliders,
            float overlapDetectionExtend = 0f)
        {
            Vector3 bottomPosition = position +
                                     CapsuleRadius
                                     * (rotation * Vector3.up);
            Vector3 topPosition = position +
                                  (CapsuleHeight - CapsuleRadius)
                                  * (rotation * Vector3.up);

            int colliderCount = Physics.OverlapCapsuleNonAlloc(bottomPosition, topPosition,
                CapsuleRadius + overlapDetectionExtend,
                colliders, queryLayer, QueryTriggerInteraction.Ignore);

            for (int i = colliderCount - 1; i >= 0; i--)
            {
                if (colliders[i] == _capsuleCollider)
                {
                    colliderCount--;
                    colliders[i] = colliders[colliderCount];
                }
            }

            return colliderCount;
        }

        /// <summary>
        /// Overlap character capsule to get collided collider using internal collider cache.
        /// </summary>
        /// <param name="position">Overlap capsule position.</param>
        /// <param name="rotation">Overlap capsule rotation.</param>
        /// <param name="overlapDetectionExtend">Extend capsule size to get a larger detection (usually not used).</param>
        /// <param name="queryLayer">LayerMask to query collision.</param>
        /// <returns>Count of colliders.</returns>
        public int CharacterOverlap(Vector3 position, Quaternion rotation, LayerMask queryLayer,
            float overlapDetectionExtend = 0f)
        {
            return _cachedOverlapsCount = CharacterOverlap(position, rotation, queryLayer, _cachedOverlapColliders,
                overlapDetectionExtend);
        }

        /// <summary>
        /// Raycast test on certain direction.
        /// </summary>
        /// <param name="position">Ray start point.</param>
        /// <param name="direction">Raycast direction.</param>
        /// <param name="distance">Raycast distance.</param>
        /// <param name="closetHit">Closet raycastHit information.</param>
        /// <param name="queryLayer">LayerMask to query collision.</param>
        /// <param name="hits">Raycast hits cache.</param>
        /// <param name="checkColliderValid">Delegate to define if a collider is valid.</param>
        /// <returns>Count of raycast hits.</returns>
        public int CharacterRaycast(Vector3 position, Vector3 direction, float distance, out RaycastHit closetHit,
            LayerMask queryLayer, RaycastHit[] hits, CheckColliderValid checkColliderValid)
        {
            int hitCount = Physics.RaycastNonAlloc(position, direction, hits, distance, queryLayer);

            closetHit = new RaycastHit();
            float closetDistance = Mathf.Infinity;

            for (int i = hitCount - 1; i >= 0; i--)
            {
                if (hits[i].distance <= 0 || (checkColliderValid != null && checkColliderValid(hits[i].collider)))
                {
                    hitCount--;
                    hits[i] = hits[hitCount];
                    continue;
                }

                if (hits[i].distance <= closetDistance)
                {
                    closetDistance = hits[i].distance;
                    closetHit = hits[i];
                }
            }

            return hitCount;
        }

        /// <summary>
        /// Capsule sweep test multi-colliders on certain direction.
        /// </summary>
        /// <param name="position">Sweep test start position.</param>
        /// <param name="rotation">Capsule rotation.</param>
        /// <param name="direction">Sweep test direction.</param>
        /// <param name="distance">Sweep test distance.</param>
        /// <param name="isStartPenetrated">Return if the character is penetrated with colliders at the sweep
        /// start.</param>
        /// <param name="closetHitInfo">Returns information of the closet hit.</param>
        /// <param name="sweepHits">Sweep hits cache.</param>
        /// <param name="checkColliderValid">Delegate to define if a collider is valid.</param>
        /// <param name="queryLayer">LayerMask to query collision.</param>
        /// <param name="sweepOffset">Capsule radius offset when doing capsule cast.</param>
        /// <returns>Count of hits.
        /// NOTE: If this count is 0, means no hit detected, closet hit is undefined.
        /// Using it in this case may lead to unexpected result.</returns>
        public int CharacterSweepTestAll(Vector3 position, Quaternion rotation, Vector3 direction, float distance,
            out bool isStartPenetrated, out RaycastHit closetHitInfo, LayerMask queryLayer, RaycastHit[] sweepHits,
            CheckColliderValid checkColliderValid, float sweepOffset = 0f)
        {
            // Sweep start position
            Vector3 bottomPosition = position +
                                     CapsuleRadius *
                                     (rotation * Vector3.up);
            Vector3 topPosition = position +
                                  (CapsuleHeight - CapsuleRadius) *
                                  (rotation * Vector3.up);
            isStartPenetrated = false;

            // Sweep test
            int hitCount = Physics.CapsuleCastNonAlloc(bottomPosition, topPosition,
                CapsuleRadius - sweepOffset, direction, sweepHits,
                distance, queryLayer, QueryTriggerInteraction.Ignore);

            // Sweep filter
            closetHitInfo = new RaycastHit();
            float closetDistance = Mathf.Infinity;
            float dotResult = Mathf.Infinity;
            for (int i = hitCount - 1; i >= 0; i--)
            {
                // Check if collider of the hit is valid to sift proper hits
                if (checkColliderValid != null && !checkColliderValid(sweepHits[i].collider))
                {
                    hitCount--;
                    sweepHits[i] = sweepHits[hitCount];
                    continue;
                }

                sweepHits[i].distance -= sweepOffset;

                // Detect penetration
                if (IsPenetratedAtStart(sweepHits[i]))
                {
                    isStartPenetrated = true;
                    hitCount--;
                    sweepHits[i] = sweepHits[hitCount];
                    continue;
                }

                sweepHits[i].normal = sweepHits[i].CorrectHitNormal(direction);

                // Get the most suitable hit
                float tmpDotResult = Vector3.Dot(sweepHits[i].normal, direction);
                if (sweepHits[i].distance > closetDistance ||
                    (Mathf.Approximately(sweepHits[i].distance - closetDistance, 0) &&
                     tmpDotResult > dotResult)) continue;

                dotResult = tmpDotResult;
                closetHitInfo = sweepHits[i];
                closetDistance = closetHitInfo.distance;
            }

            return hitCount;
        }

        /// <summary>
        /// Capsule sweep test multi-colliders on certain direction (Using internal cache).
        /// </summary>
        /// <param name="direction">Sweep test direction.</param>
        /// <param name="distance">Sweep test distance.</param>
        /// <param name="isStartPenetrated">Return if the character is penetrated with colliders at the sweep
        /// start.</param>
        /// <param name="closetHitInfo">Returns information of the closet hit.</param>
        /// <param name="checkColliderValid">Delegate to define if a collider is valid.</param>
        /// <param name="queryLayer">LayerMask to query collision.</param>
        /// <param name="sweepOffset">Capsule radius offset when doing capsule cast.</param>
        /// <returns>Count of hits.
        /// NOTE: If this count is 0, means no hit detected, closet hit is undefined.
        /// Using it in this case may lead to unexpected result.</returns>
        public int CharacterSweepTestAll(Vector3 direction, float distance,
            out bool isStartPenetrated, out RaycastHit closetHitInfo, LayerMask queryLayer,
            CheckColliderValid checkColliderValid, float sweepOffset = 0f)
        {
            int hitCounts = CharacterSweepTestAll(CharacterPosition, CharacterRotation, direction, distance,
                out isStartPenetrated,
                out closetHitInfo, queryLayer, _cachedSweepHitInfos, checkColliderValid, sweepOffset);

            return hitCounts;
        }

        /// <summary>
        /// Capsule sweep test on a certain direction.
        /// </summary>
        /// <param name="direction">Sweep test direction.</param>
        /// <param name="distance">Sweep test max distance.</param>
        /// <param name="hitInfo">Returns information of the hit.</param>
        /// <param name="queryLayer">Layer mask to query collision.</param>
        /// <param name="sweepOffset">Capsule radius offset when doing capsule cast.</param>
        /// <returns>This sweep test hit something or not.</returns>
        public bool CharacterSweepTest(Vector3 direction, float distance,
            out RaycastHit hitInfo, LayerMask queryLayer, float sweepOffset)
        {
            // Sweep start position
            Vector3 bottomPosition = _transientPosition +
                                     CapsuleRadius *
                                     (_transientRotation * Vector3.up);
            Vector3 topPosition = _transientPosition +
                                  (CapsuleHeight - CapsuleRadius) *
                                  (_targetRotation * Vector3.up);

            bool result = Physics.CapsuleCast(bottomPosition, topPosition, CapsuleRadius - sweepOffset,
                direction, out hitInfo, distance, queryLayer, QueryTriggerInteraction.Ignore);

            return result;
        }

        #endregion

        #region Moving Platform Solving

        /// <summary>
        /// Attach this character controller to a moving platform.
        /// </summary>
        /// <param name="platform">Transform of the moving platform.</param>
        public void AttachToDynamicPlatform(DynamicPlatform platform)
        {
            _attachedDynamicPlatform = platform;
            _relativePosition = _attachedDynamicPlatform.GetRelativePosition(_transientPosition);
            _relativeRotation = _attachedDynamicPlatform.GetRelativeRotation(_transientRotation);
        }

        /// <summary>
        /// Detach this character controller from a moving platform,
        /// which means not to move along with or rotate with it.
        /// </summary>
        /// <param name="platform">Transform of the moving platform you want to remove.</param>>
        public void DetachFromMovingPlatform(DynamicPlatform platform)
        {
            if (_attachedDynamicPlatform != platform) return;
            _attachedDynamicPlatform = null;
        }

        #endregion

        /// <summary>
        /// Set character movement by velocity, 
        /// which means the motion will be velocity multiply fixed delta time.
        /// </summary>
        /// <param name="velocity">Given velocity for moving the character controller.</param>
        public void MoveByVelocity(Vector3 velocity)
        {
            _motionVector = velocity * Time.deltaTime;
        }

        /// <summary>
        /// Move character using a given motion vector.
        /// </summary>
        /// <param name="motion">Given motion vector to move character a certain distance.</param>
        public void MoveByMotion(Vector3 motion)
        {
            _motionVector = motion;
        }

        /// <summary>
        /// Set directly the rotation, calculate rotation outside the controller.
        /// </summary>
        /// <param name="rot">Character rotation.</param>
        public void MoveRotation(Quaternion rot)
        {
            _rotationDirtyMark = true;
            _targetRotation = rot;
        }

        /// <summary>
        /// Set character position directly
        /// NOTE: This method will not affect character velocity.
        /// </summary>
        /// <param name="position">Target position.</param>
        public void SetPosition(Vector3 position)
        {
            _rigidbody.position = position;
        }

        /// <summary>
        /// Set character rotation directly (not using Rigidbody.MoveRotation()).
        /// NOTE: This method will not affect character velocity.
        /// </summary>
        /// <param name="rotation">Target rotation.</param>
        public void SetRotation(Quaternion rotation)
        {
            _rigidbody.rotation = rotation;
        }

        /// <summary>
        /// Check if the collider is valid for position iteration.
        /// </summary>
        /// <param name="colliderToCheck">Collider to check.</param>
        /// <returns>Valid or not.</returns>
        public bool IsColliderValid(Collider colliderToCheck)
        {
            if (colliderToCheck == _capsuleCollider) return false;

            return true;
        }

        /// <summary>
        /// Check if the collider is valid for position iteration.
        /// Dynamic rigidbodies will be ignored.
        /// </summary>
        /// <param name="colliderToCheck"></param>
        /// <returns>Valid or not.</returns>
        public bool IsStaticColliderValid(Collider colliderToCheck)
        {
            if (!IsColliderValid(colliderToCheck)) return false;

            if (colliderToCheck.attachedRigidbody && !colliderToCheck.attachedRigidbody.isKinematic)
                return false;

            return true;
        }

        /// <summary>
        /// Check whether relative angel to character is valid.
        /// </summary>
        /// <param name="normal">Normal of the plane.</param>
        /// <param name="transientRotation">Transient Rotation of character.</param>
        /// <returns>Valid or not.</returns>
        public bool IsStableOnNormal(Vector3 normal, Quaternion transientRotation)
        {
            return Vector3.Angle(normal, transientRotation * CharacterUp) < MaxSlopeAngle;
        }

        /// <summary>
        /// Check whether relative angel to character is valid.
        /// </summary>
        /// <param name="normal">Normal of the plane.</param>
        /// <returns>Valid or not.</returns>
        public bool IsStableOnNormal(Vector3 normal)
        {
            return IsStableOnNormal(normal, _transientRotation);
        }

        #endregion
    }
}