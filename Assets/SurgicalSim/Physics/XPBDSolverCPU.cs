// XPBDSolverCPU.cs - 直接移植自 Habrador/Ten-Minute-Physics-Unity
// 純 CPU Gauss-Seidel XPBD，已驗證穩定的參考實現
// 體積梯度使用官方 volIdOrder = {[1,3,2],[0,2,3],[0,3,1],[0,1,2]}
using System;
using System.Collections.Generic;
using UnityEngine;

namespace SurgicalSim.Physics
{
    public class XPBDSolverCPU : IDisposable
    {
        // ── 公開參數 ─────────────────────────────────────────
        public int     NumSubSteps      = 10;
        public float   EdgeCompliance   = 2.0f;   // 官方默認=2（柔軟），0=無限硬
        public float   VolumeCompliance = 0.0f;    // 0=不可壓縮
        public float   Damping          = 1.0f;    // 1.0=無阻尼（官方默認）
        public Vector3 Gravity          = new Vector3(0f, -9.81f, 0f);
        public float   GroundY          = 0.0f;

        // ── 粒子數據（直接操作，不走 GPU buffer）────────────
        Vector3[] _pos;
        Vector3[] _prevPos;
        Vector3[] _vel;
        float[]   _invMass;

        // ── Tet 數據 ────────────────────────────────────────
        int[] _tetIds;
        float[] _restVolumes;
        bool[] _tetActive;
        int _numTets;

        // ── 邊數據 ──────────────────────────────────────────
        int[] _edgeIds;
        float[] _restEdgeLengths;
        int _numEdges;

        int _numParticles;

        // 官方 volIdOrder（完全一致）
        static readonly int[][] volIdOrder = {
            new[] {1, 3, 2},
            new[] {0, 2, 3},
            new[] {0, 3, 1},
            new[] {0, 1, 2}
        };

        // 梯度臨時存儲（避免每幀 alloc）
        readonly Vector3[] _grads = new Vector3[4];

        public XPBDSolverCPU() { }

        public void Init(Core.TetMeshData data)
        {
            _numParticles = data.NumParticles;
            _numTets      = data.NumTets;

            // 複製位置數據
            _pos     = new Vector3[_numParticles];
            _prevPos = new Vector3[_numParticles];
            _vel     = new Vector3[_numParticles];
            _invMass = new float[_numParticles];
            Array.Copy(data.Positions, _pos, _numParticles);
            Array.Copy(data.PrevPositions, _prevPos, _numParticles);

            _tetIds      = new int[_numTets * 4];
            Array.Copy(data.TetIds, _tetIds, _numTets * 4);
            _restVolumes = new float[_numTets];
            _tetActive   = new bool[_numTets];

            // 計算 rest volumes 和 invMass（與官方完全一致）
            for (int i = 0; i < _numTets; i++)
            {
                _restVolumes[i] = GetTetVolume(i);
                _tetActive[i]   = data.TetActive[i];

                float vol = _restVolumes[i];
                float pInvMass = vol > 0f ? 1f / (vol / 4f) : 0f;

                _invMass[_tetIds[4*i+0]] += pInvMass;
                _invMass[_tetIds[4*i+1]] += pInvMass;
                _invMass[_tetIds[4*i+2]] += pInvMass;
                _invMass[_tetIds[4*i+3]] += pInvMass;
            }

            // 建邊
            BuildEdges(data);

            Debug.Log($"[XPBDSolverCPU] 初始化 | P:{_numParticles} T:{_numTets} E:{_numEdges}");
            Debug.Log($"[XPBDSolverCPU] invMass 範圍: min={MinNonZero(_invMass):F4} max={MaxVal(_invMass):F4}");

            // 診斷初始 rest volumes
            int degCount = 0;
            for (int i = 0; i < _numTets; i++)
                if (_restVolumes[i] <= 0f) degCount++;
            if (degCount > 0)
                Debug.LogWarning($"[XPBDSolverCPU] ⚠ {degCount}/{_numTets} 退化tet (restVol<=0)");
        }

        public void Step(float dt)
        {
            float sdt = dt / NumSubSteps;

            for (int step = 0; step < NumSubSteps; step++)
            {
                PreSolve(sdt);
                SolveEdges(EdgeCompliance, sdt);
                SolveVolumes(VolumeCompliance, sdt);
                HandleGroundCollision();
                PostSolve(sdt);
            }
        }

        // ─── PreSolve：積分（完全照搬官方）──────────────────
        void PreSolve(float dt)
        {
            for (int i = 0; i < _numParticles; i++)
            {
                if (_invMass[i] == 0f) continue;
                _vel[i] += dt * Gravity;
                _prevPos[i] = _pos[i];
                _pos[i] += dt * _vel[i];
            }
        }

        // ─── SolveEdges（完全照搬官方）─────────────────────
        void SolveEdges(float compliance, float dt)
        {
            float alpha = compliance / (dt * dt);

            for (int i = 0; i < _numEdges; i++)
            {
                int id0 = _edgeIds[2*i];
                int id1 = _edgeIds[2*i+1];

                float w0 = _invMass[id0];
                float w1 = _invMass[id1];
                float wTot = w0 + w1;
                if (wTot == 0f) continue;

                Vector3 diff = _pos[id0] - _pos[id1];
                float l = diff.magnitude;
                if (l == 0f) continue;

                Vector3 gradC = diff / l;
                float C = l - _restEdgeLengths[i];
                float lambda = -C / (wTot + alpha);

                _pos[id0] +=  lambda * w0 * gradC;
                _pos[id1] += -lambda * w1 * gradC;
            }
        }

        // ─── SolveVolumes（完全照搬官方，含 volIdOrder）──────
        void SolveVolumes(float compliance, float dt)
        {
            float alpha = compliance / (dt * dt);

            for (int i = 0; i < _numTets; i++)
            {
                if (!_tetActive[i]) continue;

                float wTimesGrad = 0f;

                for (int j = 0; j < 4; j++)
                {
                    int idThis = _tetIds[4*i + j];
                    int id0 = _tetIds[4*i + volIdOrder[j][0]];
                    int id1 = _tetIds[4*i + volIdOrder[j][1]];
                    int id2 = _tetIds[4*i + volIdOrder[j][2]];

                    Vector3 gradC = Vector3.Cross(
                        _pos[id1] - _pos[id0],
                        _pos[id2] - _pos[id0]
                    ) / 6f;

                    _grads[j] = gradC;
                    wTimesGrad += _invMass[idThis] * Vector3.SqrMagnitude(gradC);
                }

                if (wTimesGrad == 0f) continue;

                float vol = GetTetVolume(i);
                float C = vol - _restVolumes[i];
                float lambda = -C / (wTimesGrad + alpha);

                for (int j = 0; j < 4; j++)
                {
                    int id = _tetIds[4*i + j];
                    _pos[id] += lambda * _invMass[id] * _grads[j];
                }
            }
        }

        // ─── 地面碰撞（照搬官方）─────────────────────────
        void HandleGroundCollision()
        {
            for (int i = 0; i < _numParticles; i++)
            {
                if (_pos[i].y < GroundY)
                {
                    _pos[i] = _prevPos[i];
                    _pos[i].y = GroundY;
                }
            }
        }

        // ─── PostSolve（速度更新）─────────────────────────
        void PostSolve(float dt)
        {
            float oneOverDt = 1f / dt;
            for (int i = 0; i < _numParticles; i++)
            {
                if (_invMass[i] == 0f) continue;
                _vel[i] = (_pos[i] - _prevPos[i]) * oneOverDt;
            }
        }

        // ─── 公開接口 ────────────────────────────────────
        public void ReadbackPositions(Core.TetMeshData data)
        {
            Array.Copy(_pos, data.Positions, _numParticles);
        }

        public Vector3[] GetPositions() => _pos;

        public void UploadTetActiveBuffer(bool[] tetActive)
        {
            Array.Copy(tetActive, _tetActive, _numTets);
        }

        public void Dispose() { }

        // ─── 內部工具 ────────────────────────────────────
        float GetTetVolume(int nr)
        {
            Vector3 a = _pos[_tetIds[4*nr+0]];
            Vector3 b = _pos[_tetIds[4*nr+1]];
            Vector3 c = _pos[_tetIds[4*nr+2]];
            Vector3 d = _pos[_tetIds[4*nr+3]];
            return Vector3.Dot(Vector3.Cross(b-a, c-a), d-a) / 6f;
        }

        void BuildEdges(Core.TetMeshData data)
        {
            var edgeSet = new HashSet<long>();
            var eidList = new List<int>();
            var rlList  = new List<float>();
            int[,] pairs = {{0,1},{0,2},{0,3},{1,2},{1,3},{2,3}};
            for (int t = 0; t < _numTets; t++)
                for (int p = 0; p < 6; p++)
                {
                    int a = _tetIds[t*4+pairs[p,0]], b = _tetIds[t*4+pairs[p,1]];
                    if (a > b) { int tmp=a; a=b; b=tmp; }
                    long key = (long)a * 200000 + b;
                    if (edgeSet.Add(key))
                    {
                        eidList.Add(a); eidList.Add(b);
                        rlList.Add((_pos[a] - _pos[b]).magnitude);
                    }
                }
            _edgeIds = eidList.ToArray();
            _restEdgeLengths = rlList.ToArray();
            _numEdges = _edgeIds.Length / 2;
        }

        static float MinNonZero(float[] a) { float m=float.MaxValue; foreach(var v in a) if(v>0&&v<m) m=v; return m; }
        static float MaxVal(float[] a) { float m=0; foreach(var v in a) if(v>m) m=v; return m; }
    }
}
