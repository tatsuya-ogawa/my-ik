using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Serialization;
using UnityEngine.UIElements;

public class ObjectDynamics : MonoBehaviour
{
    public Vector3 offset;

    public float scale = 1;

    // Start is called before the first frame update
    void Start()
    {
    }

    // Update is called once per frame
    void Update()
    {
        var theta = Time.time;
        var ph = Time.time;
        var z = Mathf.Sin(ph) * scale;
        var x = Mathf.Sin(theta) * Mathf.Cos(ph) * scale;
        var y = Mathf.Cos(theta) * Mathf.Cos(ph) * scale;
        var o = gameObject;
        var rotateAngle = (Time.time * 100) % 360;
        o.transform.localPosition = new Vector3(x, y, z) + offset;
        o.transform.localEulerAngles = new Vector3(rotateAngle, rotateAngle, rotateAngle);
    }
}