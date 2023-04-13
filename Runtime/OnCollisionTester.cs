using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class OnCollisionTester : MonoBehaviour
{
    private void OnCollisionEnter(Collision collision)
    {
        Debug.Log(collision.collider.name + " Detected");
    }
}
