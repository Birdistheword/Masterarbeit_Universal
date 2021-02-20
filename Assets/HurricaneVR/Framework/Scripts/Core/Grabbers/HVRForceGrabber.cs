using System;
using System.Collections;
using HurricaneVR.Framework.ControllerInput;
using HurricaneVR.Framework.Core.Utils;
using HurricaneVR.Framework.Shared;
using HurricaneVR.Framework.Shared.HandPoser;
using HurricaneVR.Framework.Shared.Utilities;
using UnityEditor;
using UnityEngine;

namespace HurricaneVR.Framework.Core.Grabbers
{
    public class HVRForceGrabber : HVRGrabberBase
    {
        [Header("Components")]
        public HVRForceGrabberLaser Laser;
        public HVRHandGrabber HandGrabber;
        public GameObject GrabIndicator;

        [Header("Settings")] public HVRForceGrabMode GrabStyle = HVRForceGrabMode.ForcePull;
        [Tooltip("Vibration strength when hovering over something you can pick up.")]
        public float HapticsAmplitude = .1f;
        [Tooltip("Vibration duration when hovering over something you can pick up.")]
        public float HapticsDuration = .1f;
        public AudioClip SFXGrab;

        //[Header("Force Pull Style Settings")]
        [DrawIf("GrabStyle", HVRForceGrabMode.ForcePull)]
        public float DistanceThreshold = .1f;
        [DrawIf("GrabStyle", HVRForceGrabMode.ForcePull)]
        [DrawIf("GrabStyle", HVRForceGrabMode.ForcePull)] public float DynamicGrabThreshold = .1f;
        [DrawIf("GrabStyle", HVRForceGrabMode.ForcePull)] public float PullForce = 100f;
        [DrawIf("GrabStyle", HVRForceGrabMode.ForcePull)] public float MaxSpeed = 5f;
        [DrawIf("GrabStyle", HVRForceGrabMode.ForcePull)] public float InitialSpeed = 2f;
        [DrawIf("GrabStyle", HVRForceGrabMode.ForcePull)] public float Spring = 100000;
        [DrawIf("GrabStyle", HVRForceGrabMode.ForcePull)] public float Damper = 100;
        [DrawIf("GrabStyle", HVRForceGrabMode.ForcePull)] public float PercentToRotate = .4f;
        [DrawIf("GrabStyle", HVRForceGrabMode.ForcePull)] public float MaxMissSpeed = 1f;
        [DrawIf("GrabStyle", HVRForceGrabMode.ForcePull)] public float MaxMissAngularSpeed = 1f;
        

        //[Header("Gravity Gloves Style Settings")]
        [DrawIf("GrabStyle", HVRForceGrabMode.GravityGloves)] public bool RequiresFlick;

        [DrawIf("GrabStyle", HVRForceGrabMode.GravityGloves)] public float ForceTime = 1f;
        [DrawIf("GrabStyle", HVRForceGrabMode.GravityGloves)] public float YOffset = .3f;

        [DrawIf("GrabStyle", HVRForceGrabMode.GravityGloves)] public float FlickStartThreshold = 1.25f;
        [DrawIf("GrabStyle", HVRForceGrabMode.GravityGloves)] public float FlickEndThreshold = .25f;

        [DrawIf("GrabStyle", HVRForceGrabMode.GravityGloves)] public float QuickMoveThreshold = 1.25f;
        [DrawIf("GrabStyle", HVRForceGrabMode.GravityGloves)] public float QuickMoveResetThreshold = .25f;

        [DrawIf("GrabStyle", HVRForceGrabMode.GravityGloves)] public float MaximumVelocityPostCollision = 5f;
        [DrawIf("GrabStyle", HVRForceGrabMode.GravityGloves)] public float MaximumVelocityAutoGrab = 5f;

        [DrawIf("GrabStyle", HVRForceGrabMode.GravityGloves)] public bool AutoGrab = true;
        [DrawIf("GrabStyle", HVRForceGrabMode.GravityGloves)] public float AdditionalAutoGrabTime = 1f;
        [DrawIf("GrabStyle", HVRForceGrabMode.GravityGloves)] public float AutoGrabDistance = .2f;

        public HVRPlayerInputs Inputs => HandGrabber.Inputs;

        private bool _grabbableCollided;

        public override Vector3 JointAnchorWorldPosition => HandGrabber.JointAnchorWorldPosition;


        private bool _canFlick;
        private bool _canQuickStart;
        private Coroutine _additionalGrabRoutine;

        public float VelocityMagnitude => HandGrabber.HVRTrackedController.VelocityMagnitude;
        public float AngularVelocityMagnitude => HandGrabber.HVRTrackedController.AngularVelocityMagnitude;

        public HVRHandSide HandSide => HandGrabber.HandSide;

        public bool IsForceGrabbing { get; private set; }

        public bool IsAiming { get; private set; }

        protected override void Start()
        {
            base.Start();

            if (GrabStyle == HVRForceGrabMode.ForcePull)
            {
                RequiresFlick = false;
            }

            if (!HandGrabber)
            {
                HandGrabber = GetComponentInChildren<HVRHandGrabber>();
            }

            if (!HandGrabber)
            {
                Debug.LogWarning("Cannot find HandGrabber. Make sure to assign or have it on this level or below.");
            }
        }


        protected override void Update()
        {
            CheckFlick();
            CheckDrawRay();
            CheckGripButtonGrab();
            UpdateGrabIndicator();
        }

        private void CheckFlick()
        {
            if (!RequiresFlick)
                return;

            if (IsGrabbing || !IsHovering || !Inputs.GetForceGrabActive(HandSide))
            {
                return;
            }

            if (_canFlick && AngularVelocityMagnitude > FlickStartThreshold)
            {
                TryGrab(HoverTarget);
                _canFlick = false;
            }

            if (AngularVelocityMagnitude < FlickEndThreshold)
            {
                _canFlick = true;
            }

            if (VelocityMagnitude < QuickMoveResetThreshold)
            {
                _canQuickStart = true;
            }

            if (_canQuickStart && VelocityMagnitude > QuickMoveThreshold)
            {
                TryGrab(HoverTarget);
                _canQuickStart = false;
            }
        }

        private void CheckGripButtonGrab()
        {
            if (!RequiresFlick && !IsGrabbing && IsHovering && Inputs.GetForceGrabActivated(HandSide))
            {
                TryGrab(HoverTarget);
            }
        }


        private void CheckDrawRay()
        {
            if (!RequiresFlick)
                return;

            if (!IsGrabbing && HoverTarget && Inputs.GetForceGrabActive(HandSide))
            {
                Laser.Enable(HoverTarget.transform);
            }
            else
            {
                Laser.Disable();
            }
        }


        protected override void CheckUnHover()
        {
            if (RequiresFlick && !HandGrabber.IsGrabbing && Inputs.GetForceGrabActive(HandSide) && HoverTarget && !HoverTarget.IsBeingForcedGrabbed && !HoverTarget.IsBeingHeld)
            {
                IsAiming = true;
                return;
            }
            IsAiming = false;
            base.CheckUnHover();
        }

        public override bool CanGrab(HVRGrabbable grabbable)
        {
            if (!grabbable.ForceGrabbable || grabbable.IsBeingForcedGrabbed || grabbable.IsBeingHeld)
                return false;
            if (HandGrabber.IsGrabbing || HandGrabber.IsHovering || HandGrabber.IsHoveringSocket)
                return false;
            if (!grabbable.Rigidbody)
                return false;
            return base.CanGrab(grabbable);
        }

        public override bool CanHover(HVRGrabbable grabbable)
        {
            if (!CanGrab(grabbable))
                return false;
            return base.CanHover(grabbable);
        }

        protected override void OnGrabbed(HVRGrabArgs args)
        {
            //Debug.Log($"force grabbed!");
            base.OnGrabbed(args);

            if (_additionalGrabRoutine != null)
            {
                StopCoroutine(_additionalGrabRoutine);
            }

            IsForceGrabbing = true;
            if (GrabStyle == HVRForceGrabMode.GravityGloves)
            {
                StartCoroutine(ForceGrab(args.Grabbable));
            }
            else
            {
                StartCoroutine(ForcePull(args.Grabbable));
            }

            Grabbed.Invoke(this, args.Grabbable);
            args.Grabbable.Collided.AddListener(OnGrabbableCollided);
            args.Grabbable.Grabbed.AddListener(OnGrabbableGrabbed);

            if (SFXGrab)
                SFXPlayer.Instance.PlaySFX(SFXGrab, transform.position);
        }

        protected override void OnHoverEnter(HVRGrabbable grabbable)
        {
            base.OnHoverEnter(grabbable);
            
            if (IsMine && !Mathf.Approximately(0f, HapticsDuration))
            {
                HandGrabber.Controller.Vibrate(HapticsAmplitude, HapticsDuration);
            }

            if (GrabIndicator)
            {
                GrabIndicator.SetActive(true);
            }
        }

        protected override void OnHoverExit(HVRGrabbable grabbable)
        {
            base.OnHoverExit(grabbable);
            
            if (GrabIndicator)
            {
                GrabIndicator.SetActive(false);
            }
        }

        public IEnumerator ForcePull(HVRGrabbable grabbable)
        {
            var rb = grabbable.Rigidbody;
            var drag = rb.drag;
            var angularDrag = rb.angularDrag;
            HandGrabber.DisableHandCollision(grabbable);

            rb.useGravity = false;
            rb.drag = 0f;
            rb.angularDrag = 0f;
            grabbable.IsBeingForcedGrabbed = true;
            IsHoldActive = true;

            var grabPoint = grabbable.GetForceGrabPoint(HandGrabber.HandSide) ?? grabbable.transform;
            var posableGrabPoint = grabPoint.GetComponent<HVRPosableGrabPoint>();

            var isPhysicsGrab = grabbable.GrabType == HVRGrabType.PhysicPoser;
            if (!isPhysicsGrab && grabbable.GrabType != HVRGrabType.Offset)
            {
                var actualGrabPoint = HandGrabber.GetGrabPoint(grabbable);
                isPhysicsGrab = actualGrabPoint == null && grabbable.PhysicsPoserFallback;
            }


            var direction = HandGrabber.JointAnchorWorldPosition - grabPoint.position;
            var startDistance = direction.magnitude;
            var distance = startDistance;

            ConfigurableJoint joint = null;
            var startRotation = grabbable.transform.localRotation;
            var targetRotation = Quaternion.identity;
            if (posableGrabPoint)
            {
                targetRotation = posableGrabPoint.GetPoseWithJointRotation(HandSide) * Quaternion.Inverse(HandGrabber.HandWorldRotation);

                joint = grabbable.gameObject.AddComponent<ConfigurableJoint>();
                joint.rotationDriveMode = RotationDriveMode.Slerp;
                joint.connectedBody = Rigidbody;
                joint.slerpDrive = new JointDrive()
                {
                    maximumForce = Spring,
                    positionSpring = Spring,
                    positionDamper = Damper
                };
            }
            else
            {
                rb.angularDrag = 20f;
            }

            rb.velocity = direction.normalized * InitialSpeed / Time.fixedDeltaTime;

            var limit = isPhysicsGrab ? DynamicGrabThreshold : DistanceThreshold;
            
            var startRotating = startDistance * PercentToRotate;
            var percent = 0f;
            var start = grabbable.transform.rotation;
            while (GrabbedTarget && Inputs.GetForceGrabActive(HandSide) && distance > DistanceThreshold)
            {
                direction = HandGrabber.JointAnchorWorldPosition - grabPoint.position;
                distance = direction.magnitude;

                if ((isPhysicsGrab || grabbable.GrabType == HVRGrabType.Offset) && distance < DynamicGrabThreshold && HandGrabber.TryAutoGrab(grabbable))
                {
                    rb.angularVelocity = Vector3.zero;
                    rb.velocity = Vector3.zero;
                    break;
                }

                var traveled = startDistance - distance;
                if (distance < .2f)
                {
                    rb.velocity = direction / Time.fixedDeltaTime;
                }
                else
                {
                    //rb.AddForceAtPosition(direction.normalized * PullForce * rb.mass, grabPoint.position, ForceMode.Force);
                    rb.AddForce(direction.normalized * PullForce * rb.mass, ForceMode.Force);
                }

                if (joint && traveled > startRotating)
                {
                    var rotatePercent = (traveled - startRotating) / ((startDistance - startRotating) * .90f);
                    percent = Mathf.Max(percent, rotatePercent);
                    var target = Quaternion.Slerp(Quaternion.identity, targetRotation, percent);
                    joint.SetTargetRotationLocal(Quaternion.Inverse(Quaternion.Inverse(start) * target), startRotation);
                }


                rb.velocity = Vector3.ClampMagnitude(rb.velocity, MaxSpeed);

                yield return new WaitForFixedUpdate();
            }

            IsForceGrabbing = false;
            IsHoldActive = false;

            if (joint)
            {
                Destroy(joint);
            }

            if (grabbable)
            {
                
                rb.useGravity = true;
                rb.drag = drag;
                rb.angularDrag = angularDrag;
                rb.velocity = Vector3.ClampMagnitude(rb.velocity, MaxMissSpeed);
                rb.angularVelocity = Vector3.ClampMagnitude(rb.angularVelocity, MaxMissAngularSpeed);

                if (IsGrabbing)
                {
                    direction = HandGrabber.JointAnchorWorldPosition - grabPoint.position;
                    if (direction.magnitude < limit)
                    {
                        if (HandGrabber.TryAutoGrab(grabbable))
                        {
                            rb.angularVelocity = Vector3.zero;
                            rb.velocity = Vector3.zero;
                        }
                        else
                        {
                            HandGrabber.EnableHandCollision(grabbable);
                            ForceRelease();
                        }
                    }

                    grabbable.IsBeingForcedGrabbed = false;
                }
            }
        }

        public IEnumerator ForceGrab(HVRGrabbable grabbable)
        {
            var needsCollisionEnabled = true;
            var grabPoint = grabbable.GetForceGrabPoint(HandGrabber.HandSide) ?? grabbable.transform;
            try
            {
                HandGrabber.DisableHandCollision(grabbable);

                _grabbableCollided = false;
                IsHoldActive = true;


                grabbable.IsBeingForcedGrabbed = true;
                grabbable.Rigidbody.useGravity = false;
                grabbable.Rigidbody.drag = 0f;

                fts.solve_ballistic_arc_lateral(false,
                    grabPoint.position,
                    ForceTime,
                    JointAnchorWorldPosition,
                    JointAnchorWorldPosition.y + YOffset,
                    out var velocity,
                    out var gravity);

                grabbable.Rigidbody.velocity = velocity;

                var elapsed = 0f;

                var targetRotation = grabbable.GetForceGrabPointRotation(HandGrabber.HandSide);
                var startHandRotation = HandGrabber.HandWorldRotation;

                while (GrabbedTarget)
                {
                    if (elapsed > ForceTime)
                    {
                        break;
                    }

                    var currentVector = JointAnchorWorldPosition - grabPoint.position;

                    currentVector.y = 0;

                    var percentTime = elapsed / ForceTime;
                    var yExtra = YOffset * (1 - percentTime);

                    if (percentTime < .3) _grabbableCollided = false;
                    else if (_grabbableCollided)
                    {
                        if (grabbable.Rigidbody.velocity.magnitude > MaximumVelocityPostCollision)
                            grabbable.Rigidbody.velocity = grabbable.Rigidbody.velocity.normalized * MaximumVelocityPostCollision;
                        ForceRelease();
                        //Debug.Log($"Collided while force grabbing.");
                        break;
                    }


                    fts.solve_ballistic_arc_lateral(
                        false,
                        grabPoint.position,
                        ForceTime - elapsed,
                        JointAnchorWorldPosition,
                        JointAnchorWorldPosition.y + yExtra,
                        out velocity, out gravity);

                    grabbable.Rigidbody.velocity = velocity;
                    grabbable.Rigidbody.AddForce(-Vector3.up * gravity, ForceMode.Acceleration);

                    if (AutoGrab && (JointAnchorWorldPosition - grabPoint.position).magnitude < AutoGrabDistance)
                    {
                        if (HandGrabber.TryAutoGrab(GrabbedTarget))
                        {
                            needsCollisionEnabled = false;
                            IsForceGrabbing = false;
                            break;
                        }
                    }

                    if (currentVector.magnitude < .1f)
                    {
                        //Debug.Log($"<.1f");
                        break;
                    }

                    var handDelta = Quaternion.Angle(HandGrabber.HandWorldRotation, startHandRotation);
                    if (Mathf.Abs(handDelta) > 20)
                    {
                        startHandRotation = HandGrabber.HandWorldRotation;
                    }

                    //todo
                    var delta = startHandRotation * Quaternion.Inverse(targetRotation);

                    delta.ToAngleAxis(out var angle, out var axis);

                    if (angle > 180.0f) angle -= 360.0f;

                    var remaining = ForceTime - elapsed;

                    if (percentTime > .3f && Mathf.Abs(angle) > 1 && remaining > .01)
                    {
                        grabbable.Rigidbody.angularVelocity = axis * (angle * Mathf.Deg2Rad) / ForceTime;
                    }
                    else
                    {
                        grabbable.Rigidbody.angularVelocity = Vector3.zero;
                    }

                    elapsed += Time.fixedDeltaTime;
                    yield return new WaitForFixedUpdate();
                }
            }
            finally
            {
                if (needsCollisionEnabled)
                {
                    HandGrabber.EnableHandCollision(grabbable);
                }

                IsHoldActive = false;
                grabbable.IsBeingForcedGrabbed = false;
                grabbable.Collided.RemoveListener(OnGrabbableCollided);
                grabbable.Grabbed.RemoveListener(OnGrabbableGrabbed);
                if (IsGrabbing)
                {
                    ForceRelease();
                }

                IsForceGrabbing = false;
            }

            if (AutoGrab && AdditionalAutoGrabTime > 0 && !grabbable.IsBeingHeld)
            {
                _additionalGrabRoutine = StartCoroutine(ContinueAutoGrab(grabbable, grabPoint));
            }
        }

        private IEnumerator ContinueAutoGrab(HVRGrabbable grabbable, Transform grabPoint)
        {
            var elapsed = 0f;
            while (grabbable && elapsed < AdditionalAutoGrabTime && !grabbable.IsBeingHeld)
            {
                if (grabbable.Rigidbody.velocity.magnitude > MaximumVelocityAutoGrab)
                    grabbable.Rigidbody.velocity *= .9f;


                if ((JointAnchorWorldPosition - grabPoint.position).magnitude < AutoGrabDistance)
                {
                    if (HandGrabber.TryAutoGrab(grabbable))
                    {
                        break;
                    }
                }

                elapsed += Time.fixedDeltaTime;
                yield return new WaitForFixedUpdate();
            }

            _additionalGrabRoutine = null;
        }

        private void OnGrabbableGrabbed(HVRGrabberBase arg0, HVRGrabbable grabbable)
        {
            //Debug.Log($"Grabbed while force grabbing.");
        }

        private void OnGrabbableCollided(HVRGrabbable g)
        {
            _grabbableCollided = true;
        }

        private void UpdateGrabIndicator()
        {
            if (!IsHovering || !GrabIndicator)
                return;

            if (HVRManager.Instance.Camera)
            {
                GrabIndicator.transform.LookAt(HVRManager.Instance.Camera);
            }

            var grabPoint = HoverTarget.GetForceGrabPoint(HandSide);
            var position = HoverTarget.transform.position;
            if (grabPoint)
            {
                position = HandGrabber.GetGrabIndicatorPosition(HoverTarget, grabPoint, true);
            }

            GrabIndicator.transform.position = position;
        }
    }

    public enum HVRForceGrabMode
    {
        GravityGloves,
        ForcePull
    }
}