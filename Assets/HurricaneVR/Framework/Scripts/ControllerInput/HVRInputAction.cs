using HurricaneVR.Framework.Core;
using HurricaneVR.Framework.Core.Grabbers;
using HurricaneVR.Framework.Shared;
using UnityEngine;

namespace HurricaneVR.Framework.ControllerInput
{
    [RequireComponent(typeof(HVRGrabbable))]
    public abstract class HVRInputAction : MonoBehaviour
    {
        public HVRGrabbable Grabbable { get; private set; }

        protected HVRHandGrabber HandGrabber { get; private set; }

        protected virtual void Awake()
        {
            Grabbable = GetComponent<HVRGrabbable>();
            Grabbable.Grabbed.AddListener(OnGrabbableGrabbed);
            Grabbable.Released.AddListener(OnGrabbableReleased);
        }

        private void OnGrabbableReleased(HVRGrabberBase arg0, HVRGrabbable arg1)
        {
            HandGrabber = null;
        }

        private void OnGrabbableGrabbed(HVRGrabberBase grabber, HVRGrabbable arg1)
        {
            if (grabber is HVRHandGrabber handGrabber)
            {
                HandGrabber = handGrabber;
            }
        }

        public void Update()
        {
            if (HandGrabber)
            {
                var controller = HVRInputManager.Instance.GetController(HandGrabber.HandSide);
                CheckInput(controller);
            }
        }

        protected abstract void CheckInput(HVRController controller);
    }
}