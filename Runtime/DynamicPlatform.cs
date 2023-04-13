using System;
using UnityEngine;

namespace Monologist.KRCC
{
    [AddComponentMenu("Physics/KRCC/Moving Platform")]
    [RequireComponent(typeof(Rigidbody))]
    public class DynamicPlatform : MonoBehaviour
    {
        public bool IsSyncUpAxis = false;

        #region MonoBehaviours

        private void OnTriggerEnter(Collider other)
        {
            var character = other.GetComponent<KinematicRigidbodyCharacterController>();
            if (character == null) return;

            character.AttachToDynamicPlatform(this);
        }

        private void OnTriggerExit(Collider other)
        {
            var character = other.GetComponent<KinematicRigidbodyCharacterController>();
            if (character == null) return;
            
            character.DetachFromMovingPlatform(this);
        }

        #endregion

        #region Sync Methods

        /// <summary>
        /// Get absolute position synced with the dynamic platform.
        /// </summary>
        /// <param name="relativePosition"></param>
        /// <returns>Synced Position.</returns>
        public Vector3 GetSyncedPosition(Vector3 relativePosition)
        {
            return transform.TransformPoint(relativePosition);
        }

        /// <summary>
        /// Get absolute rotation with the dynamic platform.
        /// </summary>
        /// <param name="relativeRotation"></param>
        /// <param name="upAxis">Character up axis.</param>
        /// <returns>Synced Rotation.</returns>
        public Quaternion GetSyncedRotation(Quaternion relativeRotation, Vector3 upAxis)
        {
            Quaternion result = transform.rotation * relativeRotation;

            if (!IsSyncUpAxis)
            {
                Vector3 up = result * Vector3.up;
                result = Quaternion.FromToRotation(up, upAxis) * result;
            }
            
            return result;
        }
        
        /// <summary>
        /// Get relative position with the dynamic platform.
        /// </summary>
        /// <param name="worldPosition"></param>
        /// <returns>Relative position with the dynamic platform.</returns>
        public Vector3 GetRelativePosition(Vector3 worldPosition)
        {
            return transform.InverseTransformPoint(worldPosition);
        }
        
        /// <summary>
        /// Get relative rotation with the dynamic platform.
        /// </summary>
        /// <param name="worldRotation"></param>
        /// <returns>Relative position with the dynamic platform.</returns>
        public Quaternion GetRelativeRotation(Quaternion worldRotation)
        {
            return Quaternion.Inverse(transform.rotation) * worldRotation;
        }

        #endregion
    }
}