using System;
using UnityEngine;

[Serializable]
public class Path
{
    public Transform[] m_Nodes;

    public float m_Radius;

    public bool m_CanRestart;

    public bool m_CanComeBack;

    private int m_PathDirection = 1;

    private int m_CurrentNode;

    public Vector3? GetNode()
    {
        if (m_Nodes == null || m_Nodes.Length == 0)
            return null;

        return m_Nodes[m_CurrentNode].position;
    }

    public void NextNode()
    {
        if (m_CanRestart)
        {
            m_CurrentNode = ++m_CurrentNode % m_Nodes.Length;
        }
        else if (m_CanComeBack)
        {
            m_CurrentNode += m_PathDirection;

            if (m_CurrentNode >= m_Nodes.Length || m_CurrentNode < 0)
            {
                m_PathDirection *= -1;
                m_CurrentNode += m_PathDirection;
            }
        }
        else
        {
            if (m_CurrentNode < m_Nodes.Length - 1)
                m_CurrentNode++;
        }
    }
}
