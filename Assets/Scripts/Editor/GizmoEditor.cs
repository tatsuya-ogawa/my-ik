using UnityEditor;
using UnityEngine;

namespace Editor
{
    [CustomEditor(typeof(MyIK))]
    public class GizmoEditor : UnityEditor.Editor
    {
        void OnSceneGUI()
        {
            Debug.Log("OnSceneGUI");
            var ik = (MyIK) target;
            EditorGUI.BeginChangeCheck();
            Handles.color = Color.cyan;
            foreach (var node in ik.nodes)
            {
                Handles.DrawLine(node.transform.position, node.transform.position + node.transform.rotation*node.axis,2.0f);
                var up = Quaternion.AngleAxis(225,node.axis) * Vector3.Normalize(Vector3.Cross(node.axis, new Vector3(1.0f, 1.0f, 1.0f)));
                var relativeRotation = node.transform.rotation*Quaternion.Inverse(node.transform.localRotation);
                if (node.useRotationMin)
                {
                    Handles.DrawLine(node.transform.position, node.transform.position + relativeRotation*Quaternion.AngleAxis(node.rotationMin,node.axis)*up,1.0f);
                }

                if (node.useRotationMax)
                {
                    Handles.DrawLine(node.transform.position, node.transform.position + relativeRotation*Quaternion.AngleAxis(node.rotationMax,node.axis)*up,1.0f);
                }

                if (node.useRotationMin && node.useRotationMax)
                {
                    Handles.DrawWireArc(node.transform.position, relativeRotation*node.axis ,relativeRotation*Quaternion.AngleAxis(node.rotationMin,node.axis)*up,node.rotationMax-node.rotationMin,1.0f);
                }
            }
            if (EditorGUI.EndChangeCheck())
            {
            }
        }

        private void OnEnable()
        {
            Debug.Log("OnEnable");
         
        }
    }
}