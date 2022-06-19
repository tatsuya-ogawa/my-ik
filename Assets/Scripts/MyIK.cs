using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using UnityEngine.Serialization;

[Serializable]
public class MyIK : MonoBehaviour
{
    public MySolver.Node[] nodes;
    public Transform target;
    private MySolver _solver = new MySolver();

    // Start is called before the first frame update
    void Start()
    {
    }

    private float[] _angles;
    public int iterationCount = 100;

    // Update is called once per frame
    void Update()
    {
        _angles = _solver.Solve(this.nodes, target.position,
             Quaternion.FromToRotation(Vector3.forward, target.position - nodes[0].transform.position)*target.rotation,
            iterationCount);
    }

    private void FixedUpdate()
    {
        if (_angles == null) return;
        for (var i = 0; i < nodes.Length; i++)
        {
            nodes[i].transform.localRotation = Quaternion.AngleAxis(_angles[i], nodes[i].axis);
        }
    }
}