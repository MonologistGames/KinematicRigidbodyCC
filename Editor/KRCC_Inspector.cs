using Monologist.KRCC;
using UnityEditor;
using UnityEngine;
using UnityEngine.UIElements;

[CustomEditor(typeof(KinematicRigidbodyCharacterController))]
public class KRCC_Inspector : Editor
{
    public VisualTreeAsset visualTreeAsset;
    public override VisualElement CreateInspectorGUI()
    {
        VisualElement tree = new VisualElement();
        visualTreeAsset.CloneTree(tree);

        return tree;
    }
}
