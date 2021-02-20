using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using Assets.HurricaneVR.Framework.Shared.Utilities;
using HurricaneVR.Framework.Components;
using HurricaneVR.Framework.ControllerInput;
using HurricaneVR.Framework.Core.Bags;
using HurricaneVR.Framework.Core.Player;
using HurricaneVR.Framework.Core.Utils;
using HurricaneVR.Framework.Shared;
using HurricaneVR.Framework.Shared.HandPoser;
using HurricaneVR.Framework.Shared.HandPoser.Data;
using HurricaneVR.Framework.Shared.Utilities;
using HurricaneVR.Framework.Weapons;
using UnityEngine;
using Debug = UnityEngine.Debug;

namespace HurricaneVR.Framework.Core.Grabbers
{
    public class HVRHandGrabber : HVRGrabberBase
    {


        [Tooltip("HVRSocketBag used for placing and removing from sockets")]
        public HVRSocketBag SocketBag;

        [Header("HandSettings")]
        [Tooltip("Set to true if the HandModel is an IK target")]
        public bool InverseKinematics;

        [Header("Grab Settings")]

        [Tooltip("If true the hand will move to the grabbable instead of pulling the grabbable to the hand")]
        public bool HandGrabs;
        [Tooltip("Hand move speed when HandGrabs = true")]
        public float HandGrabSpeed = 5f;


        [Tooltip("If in a networked game, can someone take this an object from your hand?")]
        public bool AllowMultiplayerSwap;


        [Tooltip("Hold down or Toggle grabbing")]
        public HVRButtonTrigger GrabTrigger = HVRButtonTrigger.Active;
        [Tooltip("Left or right hand.")]
        public HVRHandSide HandSide;

        [Tooltip("If true the hand model will be cloned, collider removed, and used when parenting to the grabbable")]
        public bool CloneHandModel = true;

        [Tooltip("Vibration strength when hovering over something you can pick up.")]
        public float HapticsAmplitude = .1f;
        [Tooltip("Vibration durection when hovering over something you can pick up.")]
        public float HapticsDuration = .1f;

        [Tooltip("Ignores hand model parenting distance check.")]
        public bool IgnoreParentingDistance;
        [Tooltip("Ignores hand model parenting angle check.")]
        public bool IgnoreParentingAngle;

        [Tooltip("Angle to meet before hand model parents to the grabbable.")]
        public float ParentingMaxAngleDelta = 20f;
        [Tooltip("Distance to meet before hand model parents to the grabbable")]
        public float ParentingMaxDistance = .01f;

        [Tooltip("Settings used to pull and rotate the object into position")]
        public HVRJointSettings PullingSettings;

        [Tooltip("Layer mask to determine line of sight to the grabbable.")]
        public LayerMask RaycastLayermask;

        [Header("Components")]

        [Tooltip("The hand animator component, loads from children on startup if not supplied.")]
        public HVRHandAnimator HandAnimator;

        [Tooltip("Component that holds collider information about the hands. Auto populated from children if not set.")]
        public HVRHandPhysics HandPhysics;
        public HVRPlayerInputs Inputs;
        public HVRPhysicsPoser PhysicsPoser;
        public HVRForceGrabber ForceGrabber;
        public GameObject GrabIndicator;
        [Tooltip("Default hand pose to fall back to.")]
        public HVRHandPoser FallbackPoser;

        [Header("Required Transforms")]
        [Tooltip("Object holding the hand model.")]
        public Transform HandModel;

        [Tooltip("Configurable joints are anchored here")]
        public Transform JointAnchor;
        [Tooltip("Used to shoot ray casts at the grabbable to check if there is line of sight before grabbing.")]
        public Transform RaycastOrigin;
        [Tooltip("The transform that is handling device tracking.")]
        public Transform TrackedController;

        [Tooltip("Physics hand that will prevent the grabber from going through walls while you're holding something.")]
        public Transform InvisibleHand;

        [Tooltip("Sphere collider that checks when collisions should be re-enabled between a released grabbable and this hand.")]
        public Transform OverlapSizer;

        [Header("Dynamic Grabber")]

        [Tooltip("When dynamic solve grabbing, the velocity threshold of the grabbable to instantly grab the item")]
        public float InstantVelocityThreshold = 1f;
        [Tooltip("When dynamic solve grabbing, the angular velocity threshold of the grabbable to instantly grab the item")]
        public float InstantAngularThreshold = 1f;

        [Tooltip("How fast the physics poser hand will move towards the grabbable.")]
        public float PhysicsPoserVelocity = .75f;


        [Tooltip("Sphere collider that sizes the physics grabber check for closest collider.")]
        public Transform PhysicsGrabberSizer;




        [Header("Throw Settings")]
        [Tooltip("Factor to apply to the linear velocity of the throw.")]
        public float ReleasedVelocityFactor = 1.1f;

        [Tooltip("Factor to apply to the angular to linear calculation.")]
        public float ReleasedAngularConversionFactor = 1.0f;

        [Tooltip("Hand angular velocity must exceed this to add linear velocity based on angular velocity.")]
        public float ReleasedAngularThreshold = 1f;

        [Tooltip("Number of frames to average velocity for throwing.")]
        public int ThrowLookback = 5;

        [Tooltip("Number of frames to skip while averaging velocity.")]
        public int ThrowLookbackStart = 0;

        [Tooltip("Uses the center of mass that should match with current controller type you are using.")]
        public HVRThrowingCenterOfMass ThrowingCenterOfMass;



        [Header("Debugging")]

        [Tooltip("If enabled displays vectors involved in throwing calculation.")]
        public bool DebugThrowing;
        public HVRGrabbable PermanentGrabbable;
        public bool DrawCenterOfMass;


        public bool GrabToggleActive;

        public override bool IsHandGrabber => true;

        public HVRPhysicsHands PhysicsHands { get; private set; }
        public Transform HandModelParent { get; private set; }
        public Vector3 HandModelPosition { get; private set; }
        public Quaternion HandModelRotation { get; private set; }
        public Vector3 HandModelScale { get; private set; }

        public HVRRigidBodyOverrides RigidOverrides { get; private set; }

        //public Collider[] HandColliders { get; private set; }
        public Collider[] InvisibleHandColliders { get; private set; }

        public Dictionary<HVRGrabbable, Coroutine> OverlappingGrabbables = new Dictionary<HVRGrabbable, Coroutine>();

        public GameObject TempGrabPoint { get; internal set; }

        public HVRController Controller => HandSide == HVRHandSide.Left ? HVRInputManager.Instance.LeftController : HVRInputManager.Instance.RightController;

        public Transform HandGraphics => _handClone ? _handClone : HandModel;

        public bool IsLineGrab { get; private set; }



        public HVRTrackedController HVRTrackedController { get; private set; }

        public override Transform GrabPoint
        {
            get => base.GrabPoint;
            set
            {
                if (!value)
                {
                    PosableGrabPoint = null;
                }
                else if (GrabPoint != value)
                {
                    PosableGrabPoint = value.GetComponent<HVRPosableGrabPoint>();
                }

                base.GrabPoint = value;
            }
        }



        public HVRPosableGrabPoint PosableGrabPoint { get; private set; }



        public Quaternion PoseWorldRotation
        {
            get
            {
                if (PosableGrabPoint) return PosableGrabPoint.GetPoseWithJointRotation(HandSide);
                if (IsPhysicsPose)
                {
                    return GrabPoint.rotation * PhysicsHandRotation;
                }
                return GrabPoint.rotation;
            }
        }

        public Vector3 PoseWorldPosition
        {
            get
            {
                if (PosableGrabPoint) return PosableGrabPoint.transform.TransformPoint(PosableGrabPoint.GetPosePositionOffset(HandSide));
                if (IsPhysicsPose)
                {
                    return GrabPoint.position + PhysicsHandPosition;
                }

                return GrabPoint.position;
            }
        }




        internal Quaternion PhysicsHandRotation { get; set; }
        internal Vector3 PhysicsHandPosition { get; set; }
        internal byte[] PhysicsPoseBytes { get; private set; }



        public override Quaternion ControllerRotation => TrackedController.rotation;

        public Transform Palm => PhysicsPoser.Palm;

        public bool IsClimbing { get; private set; }

        public HVRSocket HoveredSocket;

        public bool IsPhysicsPose { get; set; }

        public Vector3 GrabAnchor { get; private set; }

        public Vector3 GrabAnchorWorld
        {
            get
            {
                if (GrabbedTarget.Rigidbody && _configurableJoint)
                {
                    return GrabbedTarget.Rigidbody.transform.TransformPoint(_configurableJoint.anchor);
                }

                if (GrabPoint)
                {
                    return GrabPoint.position;
                }
                return GrabbedTarget.transform.position;
            }
        }

        public override Vector3 JointAnchorWorldPosition => JointAnchor.position;

        public Vector3 JointAnchorWorld => transform.TransformPoint(JointAnchorLocal);

        public Vector3 JointAnchorLocal { get; private set; }

        public bool IsHoveringSocket => HoveredSocket;

        public Quaternion HandWorldRotation => transform.rotation * HandModelRotation;

        public readonly CircularBuffer<Vector3> RecentVelocities = new CircularBuffer<Vector3>(200);
        public readonly CircularBuffer<Vector3> RecentAngularVelocities = new CircularBuffer<Vector3>(200);
        public readonly CircularBuffer<Vector3> RecentPositions = new CircularBuffer<Vector3>(200);

        #region Private

        private SphereCollider _physicsGrabberCollider;
        private SphereCollider _overlapCollider;
        private readonly Collider[] _overlapColliders = new Collider[1000];
        private readonly List<Tuple<Collider, Vector3, float>> _physicsGrabPoints = new List<Tuple<Collider, Vector3, float>>();
        private readonly List<Tuple<GrabPointMeta, float>> _grabPoints = new List<Tuple<GrabPointMeta, float>>();
        private bool _movingToGrabbable;
        private bool _hasHandModelParented;
        private Quaternion _previousRotation = Quaternion.identity;
        private float _pullingTimer;
        private SkinnedMeshRenderer _mainSkin;
        private SkinnedMeshRenderer _copySkin;
        private Transform _handClone;
        private HVRHandAnimator _handCloneAnimator;
        internal ConfigurableJoint _configurableJoint;
        private Transform _handOffset;
        private Transform _fakeHand;
        private Transform _fakeHandAnchor;
        private bool _isForceAutoGrab;
        private readonly Collider[] _physicsGrabColliders = new Collider[100];
        private Vector3 _lineOffset;
        private bool _tightlyHeld;
        private bool _flipPose;
        private Quaternion _startRotation;
        private bool _primaryGrabPointGrab;
        private HVRPosableHand _posableHand;
        private HVRPosableHand _clonePosableHand;
        private bool _hasForceGrabber;
        private int _previousGrabbableLayer;

        #endregion

        protected virtual void Awake()
        {
            if (TrackedController)
                HVRTrackedController = TrackedController.GetComponent<HVRTrackedController>();

            RigidOverrides = GetComponent<HVRRigidBodyOverrides>();
        }



        protected override void Start()
        {
            base.Start();

            PhysicsHands = GetComponent<HVRPhysicsHands>();

            if (!Inputs)
            {
                Inputs = GetComponentInParent<HVRPlayerInputs>();
            }

            if (!ForceGrabber)
            {
                ForceGrabber = GetComponentInChildren<HVRForceGrabber>();
                _hasForceGrabber = ForceGrabber;
            }

            if (!HandAnimator)
            {
                if (HandModel)
                {
                    HandAnimator = HandModel.GetComponentInChildren<HVRHandAnimator>();
                }
                else
                {
                    HandAnimator = GetComponentInChildren<HVRHandAnimator>();
                }
            }

            if (!PhysicsPoser)
            {
                if (HandModel)
                {
                    PhysicsPoser = HandModel.GetComponentInChildren<HVRPhysicsPoser>();
                }
                else
                {
                    PhysicsPoser = GetComponentInChildren<HVRPhysicsPoser>();
                }
            }

            if (!HandPhysics)
            {
                HandPhysics = GetComponentInChildren<HVRHandPhysics>();
            }



            if (HandModel)
            {
                if (!HandPhysics.PhysicsHand && !InverseKinematics)
                {
                    HandPhysics.PhysicsHand = HandModel;
                    HandPhysics.SetupColliders();
                }

                _posableHand = HandModel.GetComponent<HVRPosableHand>();
                HandModelParent = HandModel.parent;
                HandModelPosition = HandModel.localPosition;
                HandModelRotation = HandModel.localRotation;
                HandModelScale = HandModel.localScale;

                if (InverseKinematics && CloneHandModel)
                {
                    Debug.Log($"CloneHandModel set to false, VRIK is enabled.");
                    CloneHandModel = false;
                }

                if (CloneHandModel)
                {
                    var handClone = Instantiate(HandModel.gameObject);
                    foreach (var col in handClone.GetComponentsInChildren<Collider>().ToArray())
                    {
                        Destroy(col);
                    }

                    _handClone = handClone.transform;
                    _handClone.parent = transform;
                    _mainSkin = HandModel.GetComponentInChildren<SkinnedMeshRenderer>();
                    _copySkin = _handClone.GetComponentInChildren<SkinnedMeshRenderer>();
                    _copySkin.enabled = false;
                    _handCloneAnimator = _handClone.GetComponentInChildren<HVRHandAnimator>();
                    _clonePosableHand = _handClone.GetComponent<HVRPosableHand>();
                }

                ResetRigidBodyProperties();

                var go = new GameObject("FakeHand");
                go.transform.parent = transform;
                go.transform.localPosition = HandModelPosition;
                go.transform.localRotation = HandModelRotation;
                _fakeHand = go.transform;

                go = new GameObject("FakeHandJointAnchor");
                go.transform.parent = _fakeHand;
                go.transform.localPosition = Vector3.zero;
                go.transform.localRotation = Quaternion.identity;
                _fakeHandAnchor = go.transform;

                go = new GameObject("HandOffset");
                go.transform.parent = transform;
                go.transform.localPosition = Vector3.zero;
                go.transform.localRotation = Quaternion.identity;
                _handOffset = go.transform;
            }

            if (InvisibleHand)
            {
                InvisibleHandColliders = InvisibleHand.gameObject.GetComponentsInChildren<Collider>().Where(e => !e.isTrigger).ToArray();
            }



            if (InvisibleHandColliders != null)
            {
                HandPhysics.IgnoreCollision(InvisibleHandColliders, true);
            }

            if (OverlapSizer)
            {
                _overlapCollider = OverlapSizer.GetComponent<SphereCollider>();
            }

            if (PhysicsGrabberSizer)
            {
                _physicsGrabberCollider = PhysicsGrabberSizer.GetComponent<SphereCollider>();
            }

            if (!SocketBag)
                SocketBag = GetComponentInChildren<HVRSocketBag>();

            if (!ThrowingCenterOfMass)
                ThrowingCenterOfMass = GetComponentInChildren<HVRThrowingCenterOfMass>();



            ResetTrackedVelocities();

        }



        public override bool IsGrabActivated
        {
            get
            {
                if (PermanentGrabbable)
                    return true;

                switch (GrabTrigger)
                {
                    case HVRButtonTrigger.Active:
                        return Inputs.GetGrabActivated(HandSide) || Inputs.GetTriggerGrabActivated(HandSide);
                    case HVRButtonTrigger.Toggle:
                        return GrabToggleActive;
                    default:
                        return false;
                }
            }
        }

        public override bool IsHoldActive
        {
            get
            {
                if (PermanentGrabbable)
                    return true;

                switch (GrabTrigger)
                {
                    case HVRButtonTrigger.Active:
                        {
                            if (!Inputs.CanTriggerGrab && IsLineGrab && Inputs.GetTriggerHoldActive(HandSide))
                            {
                                return true;
                            }
                            return Inputs.GetHoldActive(HandSide);
                        }
                    case HVRButtonTrigger.Toggle:
                        return GrabToggleActive;
                    default:
                        return false;
                }
            }
        }


        protected override void Update()
        {


            if (PerformUpdate)
            {
                CheckToggleGrab();
                CheckTriggerActivate();
            }

            UpdateGrabIndicator();
            UpdatePose();
            CheckPoseHand();
        }

        private void CheckTriggerActivate()
        {
            if (IsGrabbing)
            {
                if (Controller.TriggerButtonState.JustActivated)
                {
                    GrabbedTarget.InternalOnActivate(this);
                }
                else if (Controller.TriggerButtonState.JustDeactivated)
                {
                    GrabbedTarget.InternalOnDeactivate(this);
                }
            }
        }

        private void CheckToggleGrab()
        {
            if (GrabTrigger == HVRButtonTrigger.Toggle)
            {
                if (GrabToggleActive)
                {
                    if (Inputs.GetGrabActivated(HandSide))
                        GrabToggleActive = false;
                }
                else if (Inputs.GetGrabActivated(HandSide) || Inputs.GetTriggerGrabActivated(HandSide))
                {
                    var closest = GetClosestGrabbable();
                    if (SocketBag.ValidSockets.Count > 0 || closest)
                    {
                        GrabToggleActive = true;
                    }
                }
            }
        }

        private void UpdatePose()
        {
            if (!IsLineGrab && IsGrabbing && GrabbedTarget.Stationary && !GrabbedTarget.ParentHandModel && _hasHandModelParented)
            {
                HandModel.rotation = PoseWorldRotation;
                HandModel.position = PoseWorldPosition;
            }
        }

        protected void ResetTrackedVelocities()
        {
            for (var i = 0; i < 200; i++)
            {
                RecentVelocities.Enqueue(Vector3.zero);
                RecentAngularVelocities.Enqueue(Vector3.zero);
            }
        }

        private void DetermineGrabPoint(HVRGrabbable grabbable)
        {
            if (IsGrabbing)
                return;

            GrabPoint = GetGrabPoint(grabbable);
        }

        internal Transform GetGrabPoint(HVRGrabbable grabbable)
        {
            for (int i = 0; i < grabbable.GrabPointsMeta.Count; i++)
            {
                var grabPoint = grabbable.GrabPointsMeta[i];
                if (!grabPoint.GrabPoint)
                {
                    continue;
                }

                var angleDelta = 0f;
                var posableGrabPoint = grabPoint.PosableGrabPoint;
                Vector3 grabbableWorldAnchor;
                if (posableGrabPoint != null)
                {
                    if (HandSide == HVRHandSide.Left && !posableGrabPoint.LeftHand ||
                        HandSide == HVRHandSide.Right && !posableGrabPoint.RightHand)
                    {
                        continue;
                    }

                    var poseRotation = posableGrabPoint.GetPoseRotation(HandSide);

                    angleDelta = Quaternion.Angle(HandWorldRotation, poseRotation);
                    if (angleDelta > posableGrabPoint.AllowedAngleDifference)
                    {
                        continue;
                    }

                    grabbableWorldAnchor = grabPoint.GrabPoint.position;
                    //grabbableWorldAnchor = CalculateGrabPointWorldAnchor(grabbable, posableGrabPoint);
                }
                else
                {
                    grabbableWorldAnchor = grabPoint.GrabPoint.position;
                }

                var distance = Vector3.Distance(grabbableWorldAnchor, JointAnchorWorldPosition);
                distance += angleDelta;

                _grabPoints.Add(new Tuple<GrabPointMeta, float>(grabPoint, distance));
            }


            if (_grabPoints.Count > 0)
            {
                _grabPoints.Sort((x, y) => x.Item2.CompareTo(y.Item2));
                var temp = _grabPoints[0].Item1.GrabPoint;
                _grabPoints.Clear();
                return temp;
            }

            return null;
        }

        protected override void FixedUpdate()
        {
            if (PerformUpdate)
            {
                CheckBreakDistance();
                TrackVelocities();
                CheckSocketUnhover();
                CheckSocketHover();
                CheckSocketGrab(); //before base grab check
            }

            CheckPullingGrabbable();

            UpdateLineGrab();

            base.FixedUpdate();

            _previousRotation = transform.rotation;
        }

        protected override void CheckGrab()
        {
            if (!IsGrabActivated || !AllowGrabbing || !IsHovering)
            {
                return;
            }

            TryGrab(HoverTarget);
        }

        private void UpdateGrabIndicator()
        {
            if (!IsHovering || !GrabIndicator)
                return;

            if (HVRManager.Instance.Camera)
            {
                GrabIndicator.transform.LookAt(HVRManager.Instance.Camera);
            }

            DetermineGrabPoint(HoverTarget);

            if (PosableGrabPoint)
            {
                GrabIndicator.transform.position = GetGrabIndicatorPosition(HoverTarget, PosableGrabPoint);
            }
            else
            {
                GrabIndicator.transform.position = HoverTarget.transform.position;
            }
        }

        internal Vector3 GetGrabIndicatorPosition(HVRGrabbable grabbable, Transform grabPoint, bool useGrabPoint = false)
        {
            var posableGrabPoint = grabPoint.GetComponent<HVRPosableGrabPoint>();
            if (posableGrabPoint)
            {
                return GetGrabIndicatorPosition(grabbable, posableGrabPoint, useGrabPoint);
            }

            return grabPoint.position;
        }

        internal Vector3 GetGrabIndicatorPosition(HVRGrabbable grabbable, HVRPosableGrabPoint grabPoint, bool useGrabPoint = false)
        {
            if (grabPoint.IsLineGrab && !useGrabPoint)
            {
                var point = HVRUtilities.FindNearestPointOnLine(
                    grabPoint.LineStart.localPosition,
                    grabPoint.LineEnd.localPosition,
                    grabbable.transform.InverseTransformPoint(transform.TransformPoint(GetAnchor())));
                return grabbable.transform.TransformPoint(point);
            }

            if (grabPoint.GrabIndicatorPosition)
                return grabPoint.GrabIndicatorPosition.position;

            return grabPoint.transform.position;
        }

        protected override void OnHoverEnter(HVRGrabbable grabbable)
        {
            base.OnHoverEnter(grabbable);

            if (IsMine && !Mathf.Approximately(0f, HapticsDuration))
            {
                Controller.Vibrate(HapticsAmplitude, HapticsDuration);
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

        private void TrackVelocities()
        {
            var deltaRotation = transform.rotation * Quaternion.Inverse(_previousRotation);
            deltaRotation.ToAngleAxis(out var angle, out var axis);
            angle *= Mathf.Deg2Rad;
            var angularVelocity = axis * (angle * (1.0f / Time.fixedDeltaTime));

            RecentVelocities.Enqueue(Rigidbody.velocity);
            RecentAngularVelocities.Enqueue(angularVelocity);
            RecentPositions.Enqueue(transform.position);
        }

        private void CheckSocketUnhover()
        {
            if (!HoveredSocket)
                return;

            if (SocketBag.ClosestSocket == null || !SocketBag.ValidSockets.Contains(HoveredSocket) ||
                (SocketBag.ClosestSocket != HoveredSocket && SocketBag.ClosestSocket.IsGrabbing) || IsGrabbing)
            {
                HoveredSocket.OnHandGrabberExited();
                HoveredSocket = null;
                //Debug.Log($"socket exited");
            }
        }

        private void CheckSocketGrab()
        {
            if (GrabbedTarget)
                return;

            if (_hasForceGrabber && (ForceGrabber.IsForceGrabbing || ForceGrabber.IsAiming))
                return;

            if (HoveredSocket && HoveredSocket.GrabbedTarget && IsGrabActivated && HoveredSocket.CanGrabbableBeRemoved)
            {
                _primaryGrabPointGrab = true;
                if (TryGrab(HoveredSocket.GrabbedTarget, true))
                {
                    HoveredSocket.OnHandGrabberExited();
                    HoveredSocket = null;
                    //Debug.Log($"grabbed from socket directly");
                }
            }
        }

        private void CheckSocketHover()
        {
            if (IsGrabbing || IsHoveringSocket || !SocketBag)
                return;

            for (var i = 0; i < SocketBag.ValidSockets.Count; i++)
            {
                var socket = SocketBag.ValidSockets[i];
                if (!socket.IsGrabbing || !socket.CanInteract)
                    continue;

                HoveredSocket = socket;
                socket.OnHandGrabberEntered();
                break;
            }
        }

        private void CheckPullingGrabbable()
        {
            if (!IsGrabbing || !GrabPoint || !PullingGrabbable)
                return;

            _pullingTimer += Time.fixedDeltaTime;

            var distance = Vector3.Distance(JointAnchorWorld, GrabAnchorWorld);

            var angleDelta = Quaternion.Angle(PoseWorldRotation, HandWorldRotation);
            Vector3 worldLine;
            if (IsLineGrab && !_primaryGrabPointGrab)
            {
                worldLine = GrabbedTarget.transform.TransformDirection(PosableGrabPoint.Line.normalized) * (_flipPose ? -1f : 1f);
                angleDelta = Vector3.Angle(worldLine, transform.up);
            }

            var alreadyGrabbed = GrabbedTarget.GrabberCount > 1; //two handed grabs are difficult to rotate into position

            var angleComplete = angleDelta < GrabbedTarget.FinalJointMaxAngle || alreadyGrabbed;
            var distanceComplete = distance < ParentingMaxDistance;
            var timesUp = _pullingTimer > GrabbedTarget.FinalJointTimeout && GrabbedTarget.FinalJointQuick;

            if (angleComplete && distanceComplete || timesUp)
            {
                //Debug.Log($"before {angleDelta}");

                if (IsLineGrab && !_primaryGrabPointGrab)
                {
                    worldLine = GrabbedTarget.transform.TransformDirection(PosableGrabPoint.Line.normalized) * (_flipPose ? -1f : 1f);
                    var deltaRot = Quaternion.FromToRotation(worldLine, transform.up);
                    if (alreadyGrabbed)
                    {
                        transform.rotation = Quaternion.Inverse(deltaRot) * transform.rotation;
                    }
                    else
                    {
                        GrabbedTarget.transform.rotation = deltaRot * GrabbedTarget.transform.rotation;
                    }

                    worldLine = GrabbedTarget.transform.TransformDirection(PosableGrabPoint.Line.normalized) * (_flipPose ? -1f : 1f);
                    angleDelta = Vector3.Angle(worldLine, transform.up);
                }
                else
                {
                    var deltaRot = HandWorldRotation * Quaternion.Inverse(PoseWorldRotation);
                    if (alreadyGrabbed)
                    {
                        transform.rotation = Quaternion.Inverse(deltaRot) * transform.rotation;
                    }
                    else
                    {
                        GrabbedTarget.transform.rotation = deltaRot * GrabbedTarget.transform.rotation;
                    }

                    angleDelta = Quaternion.Angle(PoseWorldRotation, HandWorldRotation);
                }

                //Debug.Log($"final joint created {angleDelta}");
                PullingGrabbable = false;

                SetupConfigurableJoint(GrabbedTarget, true);

            }
        }

        private void CheckBreakDistance()
        {
            if (GrabbedTarget && !_movingToGrabbable)
            {
                var position = GrabbedTarget.Stationary ? TrackedController.position : JointAnchorWorldPosition;
                if (Vector3.Distance(GrabAnchorWorld, position) > GrabbedTarget.BreakDistance)
                {
                    ForceRelease();
                }
            }
        }

        private void CheckPoseHand()
        {
            if (!IsGrabbing || _hasHandModelParented || _movingToGrabbable || !GrabbedTarget || IsPhysicsPose)
                return;

            var angleDelta = 0f;
            if (GrabbedTarget.GrabType == HVRGrabType.Snap && !IgnoreParentingAngle)
            {
                if (IsLineGrab && !_primaryGrabPointGrab)
                {
                    var worldLine = GrabbedTarget.transform.TransformDirection(PosableGrabPoint.Line.normalized) * (_flipPose ? -1f : 1f);
                    angleDelta = Vector3.Angle(worldLine, transform.up);
                }
                else
                {
                    angleDelta = Quaternion.Angle(PoseWorldRotation, HandWorldRotation);
                }
            }

            var distance = 0f;
            if (!IgnoreParentingDistance && _configurableJoint)
            {
                distance = Vector3.Distance(JointAnchorWorld, GrabAnchorWorld);
            }

            if ((IgnoreParentingAngle || angleDelta <= ParentingMaxAngleDelta) &&
                (IgnoreParentingDistance || distance <= ParentingMaxDistance) ||
                GrabbedTarget.ParentHandModelImmediately ||
                GrabbedTarget.GrabberCount > 1)
            {
                if (GrabbedTarget.ParentHandModel)
                {
                    ParentHandModel(GrabPoint, PosableGrabPoint ? PosableGrabPoint.HandPoser : FallbackPoser);
                }
                else
                {
                    PoseHand();
                }
            }
        }

        private void PoseHand()
        {
            _hasHandModelParented = true;

            if (InverseKinematics)
            {
                if (PosableGrabPoint && !IsLineGrab)
                {
                    var pose = PosableGrabPoint.HandPoser.PrimaryPose.Pose.GetPose(HandSide);
                    HandModel.parent = PosableGrabPoint.transform;
                    HandModel.localRotation = pose.Rotation;
                    HandModel.localPosition = pose.Position;
                }
            }
            else
            {
                _handOffset.localPosition = Vector3.zero;
                _handOffset.localRotation = Quaternion.identity;
                HandModel.parent = _handOffset;
                if (PosableGrabPoint)
                {
                    _handOffset.localRotation = PosableGrabPoint.JointOffset;
                }
            }


            if (IsPhysicsPose)
            {
                HandAnimator?.SetCurrentPoser(null);
            }
            else
            {
                HandAnimator?.SetCurrentPoser(PosableGrabPoint ? PosableGrabPoint.HandPoser : FallbackPoser, false);
            }

        }

        private void ParentHandModel(Transform parent, HVRHandPoser poser)
        {
            if (!parent)
                return;

            if (GrabbedTarget && !GrabbedTarget.ParentHandModel)
                return;

            var worldRotation = parent.rotation;
            var worldPosition = parent.position;

            var posableGrabPoint = parent.GetComponent<HVRPosableGrabPoint>();
            if (posableGrabPoint && posableGrabPoint.VisualGrabPoint)
            {
                parent = posableGrabPoint.VisualGrabPoint;
                parent.rotation = worldRotation;
                parent.position = worldPosition;
            }

            if (CloneHandModel)
            {
                _copySkin.enabled = true;
                _handClone.transform.parent = parent;
                _handCloneAnimator?.SetCurrentPoser(poser);
                _mainSkin.enabled = false;
            }
            else
            {
                HandModel.parent = parent;
                HandAnimator?.SetCurrentPoser(poser);

                if (InverseKinematics)
                {
                    if (PosableGrabPoint)
                    {
                        var pose = PosableGrabPoint.HandPoser.PrimaryPose.Pose.GetPose(HandSide);
                        HandModel.localRotation = pose.Rotation;
                        HandModel.localPosition = pose.Position;
                    }
                }
            }

            if (IsPhysicsPose && CloneHandModel)
            {
                var pose = PhysicsPoser.Hand.CreateHandPose();
                _handClone.GetComponent<HVRPosableHand>().Pose(pose);
                _handClone.localPosition = parent.InverseTransformPoint(HandModel.position);
                _handClone.localRotation = Quaternion.Inverse(parent.rotation) * HandModel.rotation;
                ResetHand(HandModel);
            }

            _hasHandModelParented = true;

            var listener = parent.gameObject.AddComponent<HVRDestroyListener>();
            listener.Destroyed.AddListener(OnGrabPointDestroyed);
        }
        private void OnGrabPointDestroyed(HVRDestroyListener listener)
        {
            if (HandGraphics && HandGraphics.transform.parent == listener.transform)
            {
                ResetHandModel();
            }
        }

        public void OverrideHandSettings(HVRJointSettings settings)
        {
            PhysicsHands.UpdateStrength(settings);
            if (settings)
            {
                //Debug.Log($"hand - {settings.name}");
            }
            else
            {
                //Debug.Log($"hand - reset");
            }
        }

        public override bool CanHover(HVRGrabbable grabbable)
        {
            if (_hasForceGrabber && (ForceGrabber.IsForceGrabbing || ForceGrabber.IsAiming))
                return false;

            return CanGrab(grabbable);
        }

        public override bool CanGrab(HVRGrabbable grabbable)
        {
            if (!base.CanGrab(grabbable))
                return false;

            if (HoveredSocket && HoveredSocket.CanRemoveGrabbable)
                return false;

            //todo reconsider how to prevent taking items from someone elses hands in multiplayer
            //this is prone to error if someone disconnects or the grab fails on the other side

            if ((!AllowMultiplayerSwap && !grabbable.AllowMultiplayerSwap) && grabbable.HoldType != HVRHoldType.ManyHands && grabbable.AnyGrabberNotMine())
            {
                return false;
            }

            if (grabbable.PrimaryGrabber && !grabbable.PrimaryGrabber.AllowSwap)
            {
                if (grabbable.HoldType == HVRHoldType.TwoHanded && grabbable.GrabberCount > 1)
                    return false;

                if (grabbable.HoldType == HVRHoldType.OneHand && grabbable.GrabberCount > 0)
                    return false;
            }

            if (PermanentGrabbable && grabbable != PermanentGrabbable)
                return false;

            if (GrabbedTarget != null && GrabbedTarget != grabbable)
                return false;
            if (grabbable.PrimaryGrabber && grabbable.PrimaryGrabber is HVRSocket)
                return false;

            if (!grabbable.IsBeingForcedGrabbed && grabbable.RequireLineOfSight && !CheckLineOfSight(grabbable))
                return false;

            if (grabbable.RequiresGrabbable)
            {
                if (!grabbable.RequiredGrabbable.PrimaryGrabber || !grabbable.RequiredGrabbable.PrimaryGrabber.IsHandGrabber)
                    return false;
            }

            return true;
        }

        private bool CheckLineOfSight(HVRGrabbable grabbable)
        {
            var ray = new Ray();
            var anyHit = false;
            for (var i = 0; i < grabbable.Colliders.Length; i++)
            {
                var grabbableCollider = grabbable.Colliders[i];

                ray.origin = RaycastOrigin.position;
                ray.direction = grabbableCollider.bounds.center - ray.origin;

                if (Physics.Raycast(ray, out var hit, .75f, RaycastLayermask, QueryTriggerInteraction.Ignore))
                {
                    if (Equals(grabbableCollider, hit.collider))
                    {
                        anyHit = true;
                        break;
                    }
                }
            }

            for (var i = 0; i < grabbable.Triggers.Length; i++)
            {
                var grabbableCollider = grabbable.Triggers[i];

                ray.origin = RaycastOrigin.position;
                ray.direction = grabbableCollider.bounds.center - ray.origin;

                if (Physics.Raycast(ray, out var hit, .75f, RaycastLayermask, QueryTriggerInteraction.Collide))
                {
                    if (Equals(grabbableCollider, hit.collider))
                    {
                        anyHit = true;
                        break;
                    }
                }
            }

            if (!anyHit)
                return false;
            return true;
        }



        protected override void OnBeforeGrabbed(HVRGrabArgs args)
        {
            if (args.Grabbable.GrabType == HVRGrabType.Snap)
            {
                GrabPoint = null;

                if (_primaryGrabPointGrab)
                {
                    GrabPoint = args.Grabbable.GetForceGrabPoint(HandSide);
                }

                if (!GrabPoint || _isForceAutoGrab && GrabPoint == args.Grabbable.transform)
                {
                    DetermineGrabPoint(args.Grabbable);
                }


            }
            base.OnBeforeGrabbed(args);
        }

        protected override void OnGrabbed(HVRGrabArgs args)
        {
            base.OnGrabbed(args);
            var grabbable = args.Grabbable;
            if (OverlappingGrabbables.TryGetValue(grabbable, out var routine))
            {
                if (routine != null) StopCoroutine(routine);
                OverlappingGrabbables.Remove(grabbable);
            }

            DisableHandCollision(grabbable);

            _previousGrabbableLayer = args.Grabbable.gameObject.layer;
            if (UseDynamicGrab())
            {
                if (InverseKinematics)
                {
                    StartCoroutine(SolveIKPhysicsGrab());
                }
                else
                {
                    StartCoroutine(SolvePhysicsGrab());
                }

                return;
            }


            if (!GrabPoint || args.Grabbable.GrabType == HVRGrabType.Offset)
            {
                OffsetGrab(grabbable);
            }
            else
            {
                IsLineGrab = PosableGrabPoint && PosableGrabPoint.IsLineGrab;

                if (IsLineGrab)
                {
                    SetupLineGrab(grabbable);
                }

                if ((!_isForceAutoGrab) && (HandGrabs || GrabbedTarget.Stationary || GrabbedTarget.GrabberCount > 1))
                {
                    StartCoroutine(MoveGrab());
                }
                else
                {
                    GrabPointGrab(grabbable);
                }
            }
        }



        private void OffsetGrab(HVRGrabbable grabbable)
        {
            TempGrabPoint = new GameObject(name + " OffsetGrabPoint");
            TempGrabPoint.transform.position = JointAnchorWorldPosition;
            TempGrabPoint.transform.parent = GrabbedTarget.transform;
            TempGrabPoint.transform.localRotation = Quaternion.identity;
            GrabPoint = TempGrabPoint.transform;
            TempGrabPoint.transform.rotation = HandModel.rotation;

            HandAnimator.SetCurrentPoser(null);
            HandAnimator.Hand.Pose(FallbackPoser.PrimaryPose.Pose.GetPose(HandSide));
            if (grabbable.ParentHandModel)
            {
                ParentHandModel(GrabPoint, null);
            }

            Grab(grabbable);
        }

        private void SetupLineGrab(HVRGrabbable grabbable)
        {
            var testPoint = _primaryGrabPointGrab ? GrabPoint.localPosition : GrabbedTarget.transform.InverseTransformPoint(transform.TransformPoint(GetAnchor()));
            _lineOffset = HVRUtilities.FindNearestPointOnLine(PosableGrabPoint.LineStart.localPosition, PosableGrabPoint.LineEnd.localPosition, testPoint) - PosableGrabPoint.LineMid;

            _flipPose = false;
            if (PosableGrabPoint.CanLineFlip && !_primaryGrabPointGrab)
            {
                _flipPose = Vector3.Dot(grabbable.transform.TransformDirection(PosableGrabPoint.Line), transform.up) < 0;
            }
        }

        private Vector3 FindClosestPoint(HVRGrabbable grabbable)
        {
            _physicsGrabPoints.Clear();

            if (grabbable.Colliders == null || grabbable.Colliders.Length == 0)
                return grabbable.transform.position;

            foreach (var gc in grabbable.Colliders)
            {
                var point = gc.ClosestPoint(Palm.transform.position);
                if (point == Palm.transform.position || Vector3.Distance(Palm.transform.position, point) < .00001f)
                {
                    //bad colliders or infinitely small colliders will return the input to the function (this is not documented...)
                    continue;
                }
                _physicsGrabPoints.Add(new Tuple<Collider, Vector3, float>(gc, point, Vector3.Distance(point, Palm.transform.position)));
            }

            if (_physicsGrabPoints.Count == 0)
                return grabbable.transform.position;

            _physicsGrabPoints.Sort((x, y) => x.Item3.CompareTo(y.Item3));

            return _physicsGrabPoints[0].Item2;
        }

        private bool UseDynamicGrab()
        {
            if (GrabbedTarget.GrabType == HVRGrabType.Offset)
                return false;

            if (GrabbedTarget.Colliders.Length == 0)
            {
                return false;
            }

            return GrabbedTarget.GrabType == HVRGrabType.PhysicPoser || ((GrabPoint == null || GrabPoint == GrabbedTarget.transform) && GrabbedTarget.PhysicsPoserFallback);
        }

        private IEnumerator MoveGrab()
        {
            var target = GrabPoint.position;
            var linePoint = Vector3.zero;

            if (IsLineGrab && !_primaryGrabPointGrab)
            {
                linePoint = HVRUtilities.FindNearestPointOnLine(PosableGrabPoint.LineStart.localPosition, PosableGrabPoint.LineEnd.localPosition, GrabbedTarget.transform.InverseTransformPoint(transform.TransformPoint(GetAnchor())));
                target = GrabbedTarget.transform.TransformPoint(linePoint);
            }

            var time = (target - transform.position).magnitude / HandGrabSpeed;
            var elapsed = 0f;
            var start = transform.position;

            if (IsPhysicsPose)
                start = Palm.position;

            Rigidbody.detectCollisions = false;
            while (elapsed < time && GrabbedTarget)
            {
                target = IsLineGrab && !_primaryGrabPointGrab ? GrabbedTarget.transform.TransformPoint(linePoint) : GrabPoint.position;

                transform.position = Vector3.Lerp(start, target, elapsed / time);

                var targetRotation = PoseWorldRotation;

                if (!IsLineGrab || _primaryGrabPointGrab || GrabbedTarget.Stationary)
                {
                    transform.rotation = Quaternion.Slerp(HandModel.rotation, targetRotation, elapsed / time) * Quaternion.Inverse(HandModelRotation);
                }

                elapsed += Time.fixedDeltaTime;
                yield return new WaitForFixedUpdate();
            }

            Rigidbody.detectCollisions = true;

            if (!GrabbedTarget)
                yield break;

            var angleDelta = Quaternion.Angle(PoseWorldRotation, HandWorldRotation);
            if (IsLineGrab && !_primaryGrabPointGrab)
            {
                var worldLine = GrabbedTarget.transform.TransformDirection(PosableGrabPoint.Line.normalized) * (_flipPose ? -1f : 1f);
                angleDelta = Vector3.Angle(worldLine, transform.up);
            }

            //Debug.Log($"before {angleDelta}");

            if (IsLineGrab && !_primaryGrabPointGrab && !GrabbedTarget.Stationary)
            {
                var worldLine = GrabbedTarget.transform.TransformDirection(PosableGrabPoint.Line.normalized) * (_flipPose ? -1f : 1f);
                var deltaRot = Quaternion.FromToRotation(transform.up, worldLine);
                transform.rotation = deltaRot * transform.rotation;
                worldLine = GrabbedTarget.transform.TransformDirection(PosableGrabPoint.Line.normalized) * (_flipPose ? -1f : 1f);
                angleDelta = Vector3.Angle(worldLine, transform.up);
            }
            else
            {
                var deltaRot = HandWorldRotation * Quaternion.Inverse(PoseWorldRotation);
                transform.rotation = Quaternion.Inverse(deltaRot) * transform.rotation;
                angleDelta = Quaternion.Angle(PoseWorldRotation, HandWorldRotation);
            }

            //Debug.Log($"after movegrab {angleDelta}");

            GrabPointGrab(GrabbedTarget);
        }

        private void GrabPointGrab(HVRGrabbable grabbable)
        {
            Grab(grabbable);

            if (grabbable.ParentHandModel && grabbable.ParentHandModelImmediately)
            {
                ParentHandModel(GrabPoint, PosableGrabPoint ? PosableGrabPoint.HandPoser : FallbackPoser);
            }
            else if (!grabbable.ParentHandModel && grabbable.ParentHandModelImmediately)
            {
                PoseHand();
            }
        }

        public virtual void NetworkGrab(HVRGrabbable grabbable)
        {
            CommonGrab(grabbable);
        }

        public virtual void NetworkPhysicsGrab(HVRGrabbable grabbable)
        {
            IsPhysicsPose = true;
            if (grabbable.ParentHandModel)
            {
                ParentHandModel(GrabPoint.transform, null);
            }
            else
            {
                ResetHand(HandModel, true);
                PoseHand();
            }
            CommonGrab(grabbable);
        }

        protected virtual void Grab(HVRGrabbable grabbable)
        {
            CommonGrab(grabbable);
            Grabbed.Invoke(this, grabbable);
        }

        protected virtual void PhysicsGrab(HVRGrabbable grabbable)
        {
            IsPhysicsPose = true;
            if (grabbable.ParentHandModel)
            {
                ParentHandModel(GrabPoint.transform, null);
            }
            else
            {
                ResetHand(HandModel, true);
                PoseHand();
            }

            CommonGrab(grabbable);
            Grabbed.Invoke(this, grabbable);
        }

        private void CommonGrab(HVRGrabbable grabbable)
        {
            SetupGrab(grabbable);
            IsClimbing = grabbable.GetComponent<HVRClimbable>();
            if (grabbable.HandGrabbedClip)
                SFXPlayer.Instance.PlaySFX(grabbable.HandGrabbedClip, transform.position);
        }

        public void SetupGrab(HVRGrabbable grabbable)
        {
            if (grabbable.IsJointGrab)
            {


                var handMovedToGrabbable = HandGrabs && !_isForceAutoGrab && !IsPhysicsPose || grabbable.GrabberCount > 1;
                var final = grabbable.GrabType == HVRGrabType.Offset || grabbable.Stationary || (grabbable.RemainsKinematic && grabbable.Rigidbody.isKinematic) || handMovedToGrabbable;
                if (grabbable.TrackingType == HVRGrabTracking.FixedJoint && !handMovedToGrabbable)
                    final = false;

                if (final)
                {
                    if (grabbable.IsJointGrab)
                    {
                        SetupConfigurableJoint(grabbable, true);
                    }
                }
                else //needs pulling and rotating into position
                {
                    if (grabbable.IsJointGrab)
                    {
                        SetupConfigurableJoint(grabbable);

                        PullingGrabbable = true;
                        _pullingTimer = 0f;
                    }
                }

                if (!grabbable.Rigidbody.isKinematic || !grabbable.RemainsKinematic)
                {
                    grabbable.Rigidbody.isKinematic = false;
                    grabbable.Rigidbody.collisionDetectionMode = grabbable.CollisionDetection;
                }
            }

            if (GrabPoint)
            {
                grabbable.HeldGrabPoints.Add(GrabPoint);
            }
        }

        private Vector3 GetGrabAnchor()
        {
            if (IsLineGrab)
            {
                return PosableGrabPoint.LineMid;
            }

            var positionOffset = HandModelPosition;
            var rotationOffset = HandModelRotation;
            if (PosableGrabPoint)
            {
                if (PosableGrabPoint.IsJointAnchor)
                    return PosableGrabPoint.transform.localPosition;
                positionOffset = PosableGrabPoint.GetPosePositionOffset(HandSide);
                rotationOffset = PosableGrabPoint.GetPoseRotationOffset(HandSide);
            }
            else if (IsPhysicsPose)
            {
                if (InverseKinematics)
                    return GrabPoint.localPosition;
                positionOffset = PhysicsHandPosition;
                rotationOffset = PhysicsHandRotation;
            }
            else if (GrabbedTarget.GrabType == HVRGrabType.Offset)
            {
                positionOffset = HandModel.localPosition;
                rotationOffset = HandModel.localRotation;
            }

            _fakeHand.localPosition = HandModelPosition;
            _fakeHand.localRotation = HandModelRotation;
            _fakeHandAnchor.position = JointAnchorWorldPosition;
            _fakeHand.parent = GrabPoint;
            _fakeHand.localPosition = positionOffset;
            _fakeHand.localRotation = rotationOffset;
            var anchor = GrabbedTarget.Rigidbody.transform.InverseTransformPoint(_fakeHandAnchor.position);
            _fakeHand.parent = transform;


            return anchor;

        }

        private Vector3 GetAnchor()
        {
            //if (IsPhysicsPose && InverseKinematics)
            //{
            //    return Palm.localPosition;
            //}    

            if (IsLineGrab)
            {
                return Quaternion.Inverse(PosableGrabPoint.GetPoseRotationOffset(HandSide) * Quaternion.Inverse(HandModelRotation)) * -PosableGrabPoint.GetPosePositionOffset(HandSide) + HandModelPosition;
            }

            if (PosableGrabPoint && PosableGrabPoint.IsJointAnchor)
            {
                return Quaternion.Inverse(PosableGrabPoint.GetPoseRotationOffset(HandSide) * Quaternion.Inverse(HandModelRotation)) * -PosableGrabPoint.GetPosePositionOffset(HandSide);
            }

            return JointAnchor.localPosition;
        }

        public Quaternion PoseLocalRotation
        {
            get
            {
                if (PosableGrabPoint)
                    return PosableGrabPoint.transform.localRotation * PosableGrabPoint.GetJointRotationOffset(HandSide);
                return GrabPoint.localRotation;
            }
        }

        public Quaternion JointRotation
        {
            get
            {
                var poseRotation = PoseLocalRotation;

                if (IsPhysicsPose)
                {
                    poseRotation = PhysicsHandRotation;
                }
                else if (GrabbedTarget.GrabType == HVRGrabType.Offset)
                {
                    poseRotation = GrabPoint.localRotation;
                }

                return Quaternion.Inverse(GrabbedTarget.transform.rotation) * HandWorldRotation * Quaternion.Inverse(poseRotation);
            }
        }

        private void SetupConfigurableJoint(HVRGrabbable grabbable, bool final = false)
        {
            GrabAnchor = GetGrabAnchor();
            JointAnchorLocal = GetAnchor();

            var axis = Vector3.right;
            var secondaryAxis = Vector3.up;

            if (IsLineGrab)
            {
                var line = PosableGrabPoint.LineEnd.localPosition - PosableGrabPoint.LineStart.localPosition;
                axis = line.normalized;
                secondaryAxis = HVRUtilities.OrthogonalVector(axis);
            }

            if (_configurableJoint)
            {
                Destroy(_configurableJoint);
            }

            _configurableJoint = GrabbedTarget.gameObject.AddComponent<ConfigurableJoint>();
            _configurableJoint.autoConfigureConnectedAnchor = false;
            _configurableJoint.connectedBody = Rigidbody;
            _configurableJoint.configuredInWorldSpace = false;
            _configurableJoint.anchor = GrabAnchor;
            _configurableJoint.connectedAnchor = JointAnchorLocal;
            _configurableJoint.axis = axis;
            _configurableJoint.secondaryAxis = secondaryAxis;
            _configurableJoint.swapBodies = false;
            _configurableJoint.enablePreprocessing = false;

            if (IsLineGrab)
            {
                _configurableJoint.anchor = GrabAnchor + _lineOffset;
            }

            if (!GrabbedTarget.Stationary)
            {
                if (IsLineGrab && (final || !_primaryGrabPointGrab))
                {
                    var handLine = GrabbedTarget.transform.InverseTransformDirection(transform.up);
                    var poseLine = PosableGrabPoint.Line * (_flipPose ? -1f : 1f);
                    var handLocal = Quaternion.FromToRotation(poseLine, handLine);

                    if (final)
                    {
                        _startRotation = Quaternion.Inverse(Quaternion.Inverse(grabbable.transform.rotation) * transform.rotation);
                        _configurableJoint.SetTargetRotationLocal(Quaternion.Inverse(Quaternion.Inverse(grabbable.transform.rotation) * transform.rotation), _startRotation);
                    }
                    else
                    {
                        _configurableJoint.SetTargetRotationLocal(GrabbedTarget.transform.localRotation * handLocal, GrabbedTarget.transform.localRotation);
                    }
                }
                else
                {
                    _configurableJoint.SetTargetRotationLocal(GrabbedTarget.transform.localRotation * (JointRotation), GrabbedTarget.transform.localRotation);
                }
            }

            grabbable.AddJoint(_configurableJoint, this);

            HVRJointSettings pullSettings = null;

            if (grabbable.PullingSettingsOverride)
            {
                pullSettings = grabbable.PullingSettingsOverride;
            }
            else if (PullingSettings)
            {
                pullSettings = PullingSettings;
            }

            if (!final && pullSettings != null)
            {
                pullSettings.ApplySettings(_configurableJoint);
            }
            else
            {
                HVRJointSettings settings;
                if (grabbable.JointOverride)
                {
                    settings = grabbable.JointOverride;
                }
                else if (IsLineGrab)
                {
                    settings = HVRSettings.Instance.LineGrabSettings;
                }
                else if (HVRSettings.Instance.DefaultJointSettings)
                {
                    settings = HVRSettings.Instance.DefaultJointSettings;
                }
                else
                {
                    Debug.LogError("HVRGrabbable:JointOverride or HVRSettings:DefaultJointSettings must be populated.");
                    return;
                }

                settings.ApplySettings(_configurableJoint);

                if (grabbable.TrackingType == HVRGrabTracking.FixedJoint)
                {
                    _configurableJoint.xMotion = ConfigurableJointMotion.Locked;
                    _configurableJoint.yMotion = ConfigurableJointMotion.Locked;
                    _configurableJoint.zMotion = ConfigurableJointMotion.Locked;
                    _configurableJoint.angularXMotion = ConfigurableJointMotion.Locked;
                    _configurableJoint.angularYMotion = ConfigurableJointMotion.Locked;
                    _configurableJoint.angularZMotion = ConfigurableJointMotion.Locked;

                    if (grabbable.CanJointBreak)
                    {
                        _configurableJoint.breakForce = grabbable.JointBreakForce;
                        _configurableJoint.breakTorque = grabbable.JointBreakTorque;
                    }
                }

                if (IsLineGrab)
                {
                    _tightlyHeld = Inputs.GetGripHoldActive(HandSide);

                    if (!_tightlyHeld)
                    {
                        SetupLooseLineGrab();
                    }
                }
            }
        }

        private void UpdateLineGrab()
        {
            if (!IsLineGrab || PullingGrabbable || !_configurableJoint)
                return;

            if (!_tightlyHeld &&
                (GrabTrigger == HVRButtonTrigger.Active && Inputs.GetGripHoldActive(HandSide) ||
                 GrabTrigger == HVRButtonTrigger.Toggle && !Inputs.GetTriggerHoldActive(HandSide)))
            {
                _tightlyHeld = true;
                var settings = GrabbedTarget.JointOverride ?? HVRSettings.Instance.LineGrabSettings;
                settings.ApplySettings(_configurableJoint);

                _lineOffset = HVRUtilities.FindNearestPointOnLine(PosableGrabPoint.LineStart.localPosition, PosableGrabPoint.LineEnd.localPosition, GrabbedTarget.transform.InverseTransformPoint(transform.TransformPoint(GetAnchor()))) - PosableGrabPoint.LineMid;

                _configurableJoint.anchor = GrabAnchor + _lineOffset;
                _configurableJoint.SetTargetRotationLocal(Quaternion.Inverse(Quaternion.Inverse(GrabbedTarget.transform.rotation) * transform.rotation), _startRotation);
                //Debug.Log($"strong");
            }
            else if (_tightlyHeld &&
                     (GrabTrigger == HVRButtonTrigger.Active && !Inputs.GetGripHoldActive(HandSide) ||
                      GrabTrigger == HVRButtonTrigger.Toggle && Inputs.GetTriggerHoldActive(HandSide)))
            {
                _tightlyHeld = false;
                SetupLooseLineGrab();
                //Debug.Log($"weak");
            }

        }

        private void SetupLooseLineGrab()
        {
            if (PosableGrabPoint.LineCanReposition)
            {
                _configurableJoint.xMotion = ConfigurableJointMotion.Limited;
                var limit = _configurableJoint.linearLimit;
                limit.limit = PosableGrabPoint.Line.magnitude / 2f;
                _configurableJoint.linearLimit = limit;
                _configurableJoint.anchor = GrabAnchor;

                var xDrive = _configurableJoint.xDrive;
                xDrive.positionSpring = 0;
                xDrive.positionDamper = PosableGrabPoint.LooseDamper;
                xDrive.maximumForce = 100000f;
                _configurableJoint.xDrive = xDrive;
            }

            if (PosableGrabPoint.LineCanRotate)
            {
                _configurableJoint.angularXMotion = ConfigurableJointMotion.Free;


                var xDrive = _configurableJoint.angularXDrive;
                xDrive.positionSpring = 0;
                xDrive.positionDamper = PosableGrabPoint.LooseAngularDamper;
                xDrive.maximumForce = 100000f;
                _configurableJoint.angularXDrive = xDrive;
            }

        }

        protected override void OnReleased(HVRGrabbable grabbable)
        {
            base.OnReleased(grabbable);
            _primaryGrabPointGrab = false;
            _lineOffset = Vector3.zero;
            PullingGrabbable = false;

            IsLineGrab = false;
            GrabPoint = null;
            ResetHandModel();

            if (IsMine && IsPhysicsPose)
            {
                SetGrabbableLayer(grabbable, _previousGrabbableLayer);
            }

            IsPhysicsPose = false;

            HandModel.SetLayerRecursive(HVRLayers.Hand);

            if (InvisibleHand)
            {
                InvisibleHand.gameObject.SetActive(false);
            }

            var routine = StartCoroutine(CheckReleasedOverlap(grabbable));
            OverlappingGrabbables[grabbable] = routine;

            GrabToggleActive = false;

            grabbable.HeldGrabPoints.Remove(GrabPoint);

            if (TempGrabPoint)
            {
                Destroy(TempGrabPoint.gameObject);
            }

            IsClimbing = false;

            if (grabbable.Rigidbody)
            {
                var throwVelocity = ComputeThrowVelocity(grabbable, out var angularVelocity, true);
                grabbable.Rigidbody.velocity = throwVelocity;
                grabbable.Rigidbody.angularVelocity = angularVelocity;
            }

            Released.Invoke(this, grabbable);
        }

        private void SetGrabbableLayer(HVRGrabbable grabbable, int layer)
        {
            foreach (var c in grabbable.Colliders)
            {
                c.transform.gameObject.layer = layer;
            }
        }

        public Vector3 GetAverageVelocity(int seconds, int start, Vector3[] velocities = null)
        {
            return GetAverageVelocity(seconds, start, RecentVelocities, velocities);
        }

        public Vector3 GetAverageAngularVelocity(int seconds, int start, Vector3[] velocities = null)
        {
            return GetAverageVelocity(seconds, start, RecentAngularVelocities, velocities);
        }

        internal static Vector3 GetAverageVelocity(int frames, int start, CircularBuffer<Vector3> recentVelocities, Vector3[] velocities = null)
        {
            if (velocities != null)
            {
                for (int i = 0; i < velocities.Length; i++)
                {
                    velocities[i] = Vector3.zero;
                }
            }

            var sum = Vector3.zero;
            for (var i = start; i < start + frames; i++)
            {
                sum += recentVelocities[i];
                if (velocities != null && velocities.Length > i)
                {
                    velocities[i] = recentVelocities[i];
                }
            }

            if (Mathf.Approximately(frames, 0f))
                return Vector3.zero;

            var average = sum / frames;

            sum = Vector3.zero;

            for (var i = start; i < start + frames; i++)
            {
                //removing any vectors not going in the direction of the average vector
                var dot = Vector3.Dot(average.normalized, recentVelocities[i].normalized);
                if (dot < .2)
                {
                    //Debug.Log($"Filtered {average},{recentVelocities[i]},{dot}");
                    continue;
                }
                sum += recentVelocities[i];
            }

            return sum / frames;
        }



        public Vector3 ComputeThrowVelocity(HVRGrabbable grabbable, out Vector3 angularVelocity, bool isThrowing = false)
        {
            if (!grabbable.Rigidbody)
            {
                angularVelocity = Vector3.zero;
                return Vector3.zero;
            }

            var velocities = new Vector3[200];
            var angVelocities = new Vector3[200];

            var velocities2 = new Vector3[200];
            var angVelocities2 = new Vector3[200];

            var grabbableVelocity = grabbable.GetAverageVelocity(ThrowLookback, ThrowLookbackStart, velocities2);
            var grabbableAngular = grabbable.GetAverageAngularVelocity(ThrowLookback, ThrowLookbackStart, angVelocities2);

            var handVelocity = GetAverageVelocity(ThrowLookback, ThrowLookbackStart, velocities);
            var handAngularVelocity = GetAverageAngularVelocity(ThrowLookback, ThrowLookbackStart, angVelocities);

            var linearVelocity = ReleasedVelocityFactor * handVelocity + grabbableVelocity * grabbable.ReleasedVelocityFactor;
            var throwVelocity = linearVelocity;

            Vector3 centerOfMass;
            if (ThrowingCenterOfMass && ThrowingCenterOfMass.CenterOfMass)
            {
                centerOfMass = ThrowingCenterOfMass.CenterOfMass.position;
            }
            else
            {
                centerOfMass = Rigidbody.worldCenterOfMass;
            }

            //compute linear velocity from wrist rotation
            var grabbableCom = GrabPoint != null ? GrabPoint.position : grabbable.Rigidbody.worldCenterOfMass;

            //Debug.Log($"{handAngularVelocity.magnitude}");

            if (handAngularVelocity.magnitude > ReleasedAngularThreshold)
            {
                var cross = Vector3.Cross(handAngularVelocity, grabbableCom - centerOfMass) * grabbable.ReleasedAngularConversionFactor * ReleasedAngularConversionFactor;
                throwVelocity += cross;
            }

            angularVelocity = grabbableAngular * grabbable.ReleasedAngularFactor;

            if (isThrowing && DebugThrowing)
            {
                for (var i = ThrowLookbackStart; i < ThrowLookbackStart + ThrowLookback; i++)
                {
                    Debug.Log($"{velocities[i].magnitude},{angVelocities[i].magnitude}|{velocities2[i].magnitude},{angVelocities2[i].magnitude}");
                    var obj = new GameObject("average line " + i);
                    var line = obj.AddComponent<LineRenderer>();
                    line.widthCurve = new AnimationCurve(new Keyframe(0, .005f), new Keyframe(0, .005f));
                    line.SetPosition(0, RecentPositions[i]);
                    line.SetPosition(1, RecentPositions[i] + velocities[i]);
                }

                var obj2 = new GameObject("");
                var line2 = obj2.AddComponent<LineRenderer>();
                line2.widthCurve = new AnimationCurve(new Keyframe(0, .01f), new Keyframe(0, .03f));
                line2.SetPosition(0, RecentPositions[0]);
                line2.SetPosition(1, RecentPositions[0] + throwVelocity);

            }

            return throwVelocity;
        }


        private IEnumerator CheckReleasedOverlap(HVRGrabbable grabbable)
        {
            if (!OverlapSizer || !_overlapCollider)
            {
                yield break;
            }

            yield return new WaitForFixedUpdate();

            var elapsed = 0f;

            while (OverlappingGrabbables.ContainsKey(grabbable))
            {
                var count = Physics.OverlapSphereNonAlloc(OverlapSizer.transform.position, _overlapCollider.radius, _overlapColliders);
                if (count == 0) break;

                var match = false;
                for (int i = 0; i < count; i++)
                {
                    if (_overlapColliders[i].attachedRigidbody == grabbable.Rigidbody)
                    {
                        match = true;
                        break;
                    }
                }

                if (!match)
                    break;

                yield return new WaitForFixedUpdate();
                elapsed += Time.fixedDeltaTime;

                if (!grabbable.RequireOverlapClearance && elapsed > grabbable.OverlapTimeout)
                {
                    break;
                }
            }

            EnableHandCollision(grabbable);
            EnableInvisibleHandCollision(grabbable);

            OverlappingGrabbables.Remove(grabbable);
        }

        private void EnableInvisibleHandCollision(HVRGrabbable grabbable)
        {
            if (InvisibleHandColliders == null || grabbable.Colliders == null)
            {
                return;
            }

            foreach (var handCollider in InvisibleHandColliders)
            {
                foreach (var grabbableCollider in grabbable.Colliders)
                {
                    if (grabbableCollider)
                        Physics.IgnoreCollision(handCollider, grabbableCollider, false);
                }

                foreach (var grabbableCollider in grabbable.AdditionalIgnoreColliders)
                {
                    Physics.IgnoreCollision(handCollider, grabbableCollider, false);
                }
            }
        }

        private void DisableInvisibleHandCollision(HVRGrabbable grabbable, Collider except = null)
        {
            if (InvisibleHandColliders == null || grabbable.Colliders == null)
            {
                return;
            }

            foreach (var handCollider in InvisibleHandColliders)
            {
                foreach (var grabbableCollider in grabbable.Colliders)
                {
                    if (grabbableCollider && except != grabbableCollider)
                        Physics.IgnoreCollision(handCollider, grabbableCollider);
                }

                foreach (var grabbableCollider in grabbable.AdditionalIgnoreColliders)
                {
                    Physics.IgnoreCollision(handCollider, grabbableCollider);
                }
            }
        }

        internal void EnableHandCollision(HVRGrabbable grabbable)
        {
            HandPhysics.IgnoreCollision(grabbable.Colliders, false);
            HandPhysics.IgnoreCollision(grabbable.AdditionalIgnoreColliders, false);
        }

        internal void DisableHandCollision(HVRGrabbable grabbable)
        {
            if (grabbable.EnableInvisibleHand && InvisibleHand)
            {
                InvisibleHand.gameObject.SetActive(true);
                DisableInvisibleHandCollision(grabbable);
            }

            HandPhysics.IgnoreCollision(grabbable.Colliders, true);
            HandPhysics.IgnoreCollision(grabbable.AdditionalIgnoreColliders, true);
        }

        private IEnumerator SolveIKPhysicsGrab()
        {

            var layer = HandSide == HVRHandSide.Left ? HVRLayers.LeftTarget : HVRLayers.RightTarget;
            var layerMask = LayerMask.GetMask(layer.ToString());
            SetGrabbableLayer(GrabbedTarget, LayerMask.NameToLayer(layer.ToString()));

            IsPhysicsPose = true;

            TempGrabPoint = new GameObject(name + " GrabPoint");
            TempGrabPoint.transform.parent = GrabbedTarget.transform;
            TempGrabPoint.transform.position = FindClosestPoint(GrabbedTarget);
            TempGrabPoint.transform.localRotation = Quaternion.identity;
            GrabPoint = TempGrabPoint.transform;

            GrabbedTarget.Rigidbody.velocity = Vector3.zero;
            GrabbedTarget.Rigidbody.angularVelocity = Vector3.zero;

            var delta = GrabPoint.position - PhysicsPoser.Palm.position;
            var palmDelta = Quaternion.FromToRotation(PhysicsPoser.Palm.forward, delta.normalized);
            HandModel.rotation = palmDelta * HandModel.rotation;
            var offset = HandModel.position - Palm.position;

            HandModel.position = GrabPoint.position + offset;
            var pos = GrabbedTarget.transform.position;
            var rot = GrabbedTarget.transform.rotation;

            yield return new WaitForEndOfFrame();

            GrabbedTarget.transform.position = pos;
            GrabbedTarget.transform.rotation = rot;
            HandAnimator?.SetCurrentPoser(null);
            PhysicsPoser.OpenFingers();

            var gpDelta = GrabPoint.position - Palm.position;
            PhysicsPoser.Hand.transform.position += gpDelta;

            HandModel.position += gpDelta;

            if (GrabbedTarget)
            {
                PhysicsPoser.SimulateClose(layerMask);

                GrabbedTarget.Rigidbody.velocity = Vector3.zero;
                GrabbedTarget.Rigidbody.angularVelocity = Vector3.zero;
                PhysicsHandRotation = Quaternion.Inverse(GrabPoint.rotation) * HandModel.rotation;
                PhysicsHandPosition = GrabPoint.transform.InverseTransformPoint(HandModel.position);

                PhysicsPoser.SimulateClose(layerMask);
                PhysicsPoseBytes = PhysicsPoser.Hand.CreateHandPose().Serialize();
                PhysicsGrab(GrabbedTarget);

                SetGrabbableLayer(GrabbedTarget, _previousGrabbableLayer);
            }
        }

        private IEnumerator SolvePhysicsGrab()
        {
            _movingToGrabbable = true;
            var solved = false;

            var grabbedTarget = GrabbedTarget;

            var previousLayer = grabbedTarget.gameObject.layer;

            IsPhysicsPose = true;
            try
            {
                var layer = HandSide == HVRHandSide.Left ? HVRLayers.LeftTarget : HVRLayers.RightTarget;
                var layerMask = LayerMask.GetMask(layer.ToString());
                SetGrabbableLayer(grabbedTarget, LayerMask.NameToLayer(layer.ToString()));

                EnableHandCollision(GrabbedTarget);

                var hand = new GameObject("Hand");
                hand.transform.position = transform.position;
                hand.transform.rotation = transform.rotation;

                var rb = hand.AddComponent<Rigidbody>();
                rb.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;

                if (!GrabbedTarget.Rigidbody.isKinematic)
                    GrabbedTarget.Rigidbody.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;

                var tracker = hand.AddComponent<HVRCollisionMonitor>();
                rb.useGravity = false;
                HandModel.parent = hand.transform;
                hand.transform.SetLayerRecursive(HVRLayers.Hand);

                var target = new GameObject("Target");
                target.transform.position = FindClosestPoint(grabbedTarget);
                target.transform.parent = GrabbedTarget.transform;


                var delta = target.transform.position - PhysicsPoser.Palm.position;
                var palmDelta = Quaternion.FromToRotation(PhysicsPoser.Palm.forward, delta.normalized);
                hand.transform.rotation = palmDelta * hand.transform.rotation;

                var time = delta.magnitude / PhysicsPoserVelocity + .3f;
                rb.velocity = PhysicsPoserVelocity * delta.normalized;
                if (HandAnimator != null)
                    HandAnimator.SetCurrentPoser(null);


                PhysicsPoser.OpenFingers();
                var elapsed = 0f;
                var snapping = false;

                while (GrabbedTarget && elapsed < time)
                {
                    delta = target.transform.position - PhysicsPoser.Palm.position;
                    rb.velocity = delta.normalized * rb.velocity.magnitude;
                    palmDelta = Quaternion.FromToRotation(PhysicsPoser.Palm.forward, delta.normalized);
                    palmDelta.ToAngleAxis(out var angle, out var axis);
                    if (angle > 180.0f) angle -= 360.0f;

                    if (Mathf.Abs(angle) > 5)
                    {
                        rb.angularVelocity = (1f / Time.fixedDeltaTime * angle * axis * 0.01745329251994f * Mathf.Pow(1, 90f * Time.fixedDeltaTime));
                    }
                    else
                    {
                        rb.angularVelocity = Vector3.zero;
                    }


                    if (_isForceAutoGrab || snapping || grabbedTarget.Rigidbody.velocity.magnitude > InstantVelocityThreshold || grabbedTarget.Rigidbody.angularVelocity.magnitude > InstantAngularThreshold)
                    {
                        rb.velocity *= 1.2f;

                        grabbedTarget.Rigidbody.velocity *= .7f;
                        grabbedTarget.Rigidbody.angularVelocity *= .7f;


                        snapping = true;
                    }

                    if (tracker.Collided)
                    {
                        if (GrabbedTarget.Colliders.Contains(tracker.Collider))
                        {
                            //Debug.Log($"collided");
                            _movingToGrabbable = false;

                            TempGrabPoint = new GameObject(name + " PhysicsGrabPoint");
                            TempGrabPoint.transform.position = Palm.position;
                            TempGrabPoint.transform.parent = GrabbedTarget.transform;
                            TempGrabPoint.transform.localRotation = Quaternion.identity;
                            GrabPoint = TempGrabPoint.transform;

                            GrabbedTarget.Rigidbody.velocity = Vector3.zero;
                            GrabbedTarget.Rigidbody.angularVelocity = Vector3.zero;
                            PhysicsHandRotation = Quaternion.Inverse(GrabPoint.rotation) * HandModel.rotation;
                            PhysicsHandPosition = GrabPoint.transform.InverseTransformPoint(HandModel.position);

                            PhysicsPoser.SimulateClose(layerMask);
                            PhysicsPoseBytes = PhysicsPoser.Hand.CreateHandPose().Serialize();
                            PhysicsGrab(grabbedTarget);


                            solved = true;
                            break;
                        }

                        tracker.Collided = false;
                        tracker.Collider = null;
                    }

                    yield return new WaitForFixedUpdate();
                    elapsed += Time.fixedDeltaTime;
                }

                if (!solved)
                {
                    Debug.Log($"unsolved reset hand model");
                    ResetHand(HandModel);
                }

                Destroy(target);
                Destroy(hand);
            }
            finally
            {
                if (GrabbedTarget)
                {
                    SetGrabbableLayer(grabbedTarget, previousLayer);
                }

                _movingToGrabbable = false;
            }

            if (!solved && GrabbedTarget)
            {
                Debug.Log($"unsolved force release");
                ForceRelease();
            }
        }


        public bool TryAutoGrab(HVRGrabbable grabbable)
        {
            if (GrabTrigger == HVRButtonTrigger.Active && !Inputs.GetHoldActive(HandSide))
            {
                return false;
            }

            grabbable.Rigidbody.velocity = Vector3.zero;
            grabbable.Rigidbody.angularVelocity = Vector3.zero;

            GrabPoint = grabbable.GetForceGrabPoint(HandSide) ?? grabbable.transform;
            if (!PosableGrabPoint && grabbable.PhysicsPoserFallback)
                GrabPoint = null;

            _isForceAutoGrab = true;
            _primaryGrabPointGrab = true;

            try
            {
                if (TryGrab(grabbable))
                {
                    if (GrabTrigger == HVRButtonTrigger.Toggle)
                        GrabToggleActive = true;
                    return true;
                }
            }
            finally
            {
                _isForceAutoGrab = false;
            }
            return false;
        }


        private void ResetHandModel(bool maintainPose = false)
        {
            _hasHandModelParented = false;
            if (!HandGraphics)
                return;

            if (HandGraphics.parent)
            {
                var listener = HandGraphics.parent.GetComponent<HVRDestroyListener>();
                if (listener)
                    listener.Destroyed.RemoveListener(OnGrabPointDestroyed);
            }

            ResetHand(HandModel, maintainPose);
            if (_handClone)
            {
                ResetHand(_handClone, maintainPose);
            }

            if (_copySkin)
            {
                _copySkin.enabled = false;
            }

            if (_mainSkin)
            {
                _mainSkin.enabled = true;
            }
        }

        private void ResetHand(Transform hand, bool maintainPose = false)
        {
            hand.parent = HandModelParent;
            hand.localPosition = HandModelPosition;
            hand.localRotation = HandModelRotation;
            hand.localScale = HandModelScale;
            if (!maintainPose)
            {
                if (InverseKinematics)
                {
                    HandAnimator?.ResetToDefault();
                }
                else
                {
                    hand.GetComponent<HVRHandAnimator>()?.ResetToDefault();
                }

            }
        }

        private void ResetRigidBodyProperties()
        {
            this.ExecuteNextUpdate(() =>
            {
                Rigidbody.ResetCenterOfMass();
                Rigidbody.ResetInertiaTensor();

                if (RigidOverrides)
                {
                    RigidOverrides.ApplyOverrides();
                }
            });
        }

        internal byte[] GetPoseData()
        {
            if (CloneHandModel)
            {
                return _clonePosableHand.CreateHandPose().Serialize();
            }
            else
            {
                return _posableHand.CreateHandPose().Serialize();
            }
        }

        internal void PoseHand(byte[] data)
        {
            if (CloneHandModel)
            {
                _clonePosableHand.Pose(HVRHandPoseData.FromByteArray(data, HandSide), GrabbedTarget.ParentHandModel);
            }
            _posableHand.Pose(HVRHandPoseData.FromByteArray(data, HandSide), GrabbedTarget.ParentHandModel);
        }

#if UNITY_EDITOR
        private void OnDrawGizmos()
        {
            if (DrawCenterOfMass && Rigidbody)
            {
                Gizmos.color = Color.magenta;
                Gizmos.DrawWireSphere(Rigidbody.worldCenterOfMass, .03f);
            }

            //if (_configurableJoint)
            //{
            //    Gizmos.color = Color.cyan;
            //    Gizmos.DrawWireSphere(_configurableJoint.transform.TransformPoint(_configurableJoint.anchor), .02f);
            //    Gizmos.color = Color.blue;
            //    Gizmos.DrawCube(transform.TransformPoint(_configurableJoint.connectedAnchor), new Vector3(.02f, .02f, .02f));
            //}

            //if (PosableGrabPoint && (IsHovering || IsGrabbing))
            //{
            //    Gizmos.color = Color.red;
            //    Gizmos.DrawCube(PoseWorldPosition, new Vector3(.02f, .02f, .02f));
            //    //Debug.DrawLine(PoseWorldPosition, GrabPoint.position, Color.green);

            //    Gizmos.color = Color.blue;

            //    var grabbable = HoverTarget ?? GrabbedTarget;

            //    var p = Quaternion.Inverse(PosableGrabPoint.GetPoseRotationOffset(HandSide) * Quaternion.Inverse(HandModelRotation)) * -(PosableGrabPoint.GetPosePositionOffset(HandSide));

            //    Gizmos.DrawCube(transform.TransformPoint(p), new Vector3(.02f, .02f, .02f));
            //}
        }

#endif
    }
}
