using UnityEngine;

[System.Serializable] 
public class IKNode
{
    public int id;
    public Transform transform;
    public Vector3 axis;
    public bool useRotationMin;
    [SerializeField, Range(-180, 180)] public float rotationMin;
    public bool useRotationMax;
    [SerializeField, Range(-180, 180)] public float rotationMax;
}
public class IKNodeForCalculation
{
    public Vector3 position;
    public Quaternion rotation;
    public Vector3 axis;
    public bool useRotationMin;
    [SerializeField, Range(-180, 180)] public float rotationMin;
    public bool useRotationMax;
    [SerializeField, Range(-180, 180)] public float rotationMax;
}