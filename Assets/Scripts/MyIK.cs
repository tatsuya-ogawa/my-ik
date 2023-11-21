using UnityEngine;
using System;

interface ISolver
{
    Quaternion[] Solve(IKNode[] nodes, Vector3 targetPosition, Quaternion? targetRotation, int iterationLimit);
}

[Serializable]
public class MyIK : MonoBehaviour
{
    public IKNode[] nodes;
    public Transform target;
    private ISolver _solver = new MyCCDIKSolver();

    // Start is called before the first frame update
    void Start()
    {
    }

    private Quaternion[] _angles;
    public int iterationCount = 1;

    // Update is called once per frame
    void Update()
    {
        _angles = _solver.Solve(this.nodes, target.position,
            Quaternion.FromToRotation(Vector3.forward, target.position - nodes[0].transform.position) *
            target.rotation,
            iterationCount);
    }

    private void FixedUpdate()
    {
        if (_angles == null) return;
        for (var i = 0; i < nodes.Length; i++)
        {
            nodes[i].transform.localRotation = _angles[i];
        }
    }
}