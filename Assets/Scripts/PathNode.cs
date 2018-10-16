using UnityEngine;

public class PathNode : MonoBehaviour
{
    public Color m_PointColor;

    [Header("Radius")]
    public Color m_RadiusColor;

    public float m_Radius;

    private void OnDrawGizmos()
    {
        Gizmos.color = m_PointColor;
        Gizmos.DrawSphere(transform.position, 0.2f);

        Gizmos.color = m_RadiusColor;
        Gizmos.DrawSphere(transform.position, m_Radius);
    }
}
