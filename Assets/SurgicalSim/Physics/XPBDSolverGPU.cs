// XPBDSolverGPU.cs — NeoHookean 弹性 XPBD 求解器
// 参考: FantasyVR/neohookean_XPBD + Peng Yu EG2025 论文
// Graph Coloring Gauss-Seidel + NeoHookean (deviatoric + hydrostatic)
using System;
using System.Collections.Generic;
using UnityEngine;

namespace SurgicalSim.Physics
{
    public class XPBDSolverGPU : IDisposable
    {
        // ── 公开参数 ─────────────────────────────────────────
        public int     NumSubSteps      = 10;
        public float   EdgeCompliance   = 2.0f;
        public float   YoungsModulus    = 1e5f;   // Pa
        public float   PoissonsRatio    = 0.45f;
        public float   Damping          = 0.05f;  // 论文默认
        public float   Density          = 1000f;  // kg/m^3
        public Vector3 Gravity          = new Vector3(0f, -9.81f, 0f);
        public float   GroundY          = 0.0f;

        // Lamé 参数（从 Young's / Poisson's 计算）
        float _lameLambda, _lameMu;

        // ── GPU 资源 ─────────────────────────────────────────
        readonly ComputeShader _cs;
        int _kIntegrate, _kSolveEdges, _kSolveNHDev, _kSolveNHHyd,
            _kPostSolve, _kGroundCollision, _kToolCollision,
            _kSolveSurfaceToolContacts;

        ComputeBuffer _bufPos, _bufPrevPos, _bufVel, _bufInvMass;
        ComputeBuffer _bufTetIds, _bufRestVol, _bufTetActive;
        ComputeBuffer _bufEdgeIds, _bufRestLen, _bufEdgeActive;
        ComputeBuffer _bufColorFlat, _bufEdgeColorFlat;
        ComputeBuffer _bufSurfaceTriIds, _bufSurfaceTriColorFlat;
        // NeoHookean 专用
        ComputeBuffer _bufInvRestMatrix;    // float4[numT*3]
        ComputeBuffer _bufAlphaDeviatoric;  // float[numT]
        ComputeBuffer _bufAlphaHydrostatic; // float[numT]

        int[] _groupOff, _groupCnt; int _numColors;
        int[] _edgeGroupOff, _edgeGroupCnt; int _numEdgeColors;

        int _numP, _numT, _numE, _numSurfaceTris;
        const int TH = 64;
        Vector4[] _readBuf;

        List<int>[] _edgeToTets;
        int[] _surfaceGroupOff, _surfaceGroupCnt; int _numSurfaceColors;

        public float ToolContactDistance = 0.01f;
        public float ToolContactCompliance = 1e-7f;
        public int ToolContactIterations = 2;
        public int ToolContactCouplingPasses = 2;

        [System.Runtime.InteropServices.StructLayout(
            System.Runtime.InteropServices.LayoutKind.Sequential)]
        struct UInt4 { public uint x, y, z, w; }
        [System.Runtime.InteropServices.StructLayout(
            System.Runtime.InteropServices.LayoutKind.Sequential)]
        struct UInt2 { public uint x, y; }
        [System.Runtime.InteropServices.StructLayout(
            System.Runtime.InteropServices.LayoutKind.Sequential)]
        struct UInt3 { public uint x, y, z; }

        public XPBDSolverGPU(ComputeShader cs)
        {
            _cs = cs ?? throw new ArgumentNullException(nameof(cs));
            _kIntegrate      = _cs.FindKernel("CSIntegrate");
            _kSolveEdges     = _cs.FindKernel("CSSolveEdges");
            _kSolveNHDev     = _cs.FindKernel("CSSolveNeoHookeanDev");
            _kSolveNHHyd     = _cs.FindKernel("CSSolveNeoHookeanHyd");
            _kPostSolve      = _cs.FindKernel("CSPostSolve");
            _kGroundCollision = _cs.FindKernel("CSGroundCollision");
            _kToolCollision   = _cs.FindKernel("CSToolCollision");
            _kSolveSurfaceToolContacts = _cs.FindKernel("CSSolveSurfaceToolContacts");
        }

        // ══════════════════════════════════════════════════════
        // Init — 计算 B^{-1}, Lamé 参数, restVolume, invMass
        // 参考: FantasyVR init_phy + init_alpha
        // ══════════════════════════════════════════════════════
        public void Init(Core.TetMeshData data)
        {
            _numP = data.NumParticles;
            _numT = data.NumTets;
            _numSurfaceTris = data.NumSurfaceTris;

            // Lamé 参数
            float E = YoungsModulus;
            float nu = PoissonsRatio;
            _lameLambda = E * nu / ((1f + nu) * (1f - 2f * nu));
            _lameMu     = E / (2f * (1f + nu));

            Vector3[] restPos = data.RestPositions;
            Vector3[] curPos  = data.Positions;

            // ── Per-tet: B^{-1}, restVol, alpha ──────────────
            float[] mass        = new float[_numP];  // 先累加质量
            float[] invMass     = new float[_numP];
            float[] restVolumes = new float[_numT];
            Vector4[] invRestMat = new Vector4[_numT * 3]; // 3 rows per tet
            float[] alphaDev    = new float[_numT];
            float[] alphaHyd    = new float[_numT];
            const float minVolForConstraint = 1e-10f;

            for (int i = 0; i < _numT; i++)
            {
                if (!data.TetActive[i]) continue;
                int id0 = data.TetIds[4*i], id1 = data.TetIds[4*i+1];
                int id2 = data.TetIds[4*i+2], id3 = data.TetIds[4*i+3];

                Vector3 e1 = restPos[id1] - restPos[id0];
                Vector3 e2 = restPos[id2] - restPos[id0];
                Vector3 e3 = restPos[id3] - restPos[id0];

                float det = Vector3.Dot(Vector3.Cross(e1, e2), e3);
                float vol = Mathf.Abs(det) / 6f;
                restVolumes[i] = vol;

                float avgMass = Density * vol / 4f;
                mass[id0] += avgMass;
                mass[id1] += avgMass;
                mass[id2] += avgMass;
                mass[id3] += avgMass;

                // Very small child tets are topological bookkeeping for a
                // zero-width cut, not stable Neo-Hookean elements. Edge
                // constraints still keep them connected without huge gradients.
                if (Mathf.Abs(det) < 1e-12f || vol < minVolForConstraint)
                {
                    invRestMat[i*3+0] = Vector4.zero;
                    invRestMat[i*3+1] = Vector4.zero;
                    invRestMat[i*3+2] = Vector4.zero;
                    alphaDev[i] = 0f;
                    alphaHyd[i] = 0f;
                    continue;
                }

                float invDet = 1f / det;
                Vector3 r0 = Vector3.Cross(e2, e3) * invDet;
                Vector3 r1 = Vector3.Cross(e3, e1) * invDet;
                Vector3 r2 = Vector3.Cross(e1, e2) * invDet;

                invRestMat[i*3+0] = new Vector4(r0.x, r0.y, r0.z, 0);
                invRestMat[i*3+1] = new Vector4(r1.x, r1.y, r1.z, 0);
                invRestMat[i*3+2] = new Vector4(r2.x, r2.y, r2.z, 0);

                // 累加质量 (参考 FantasyVR: mass[a] += density * V / 4)
                // alpha = 1/(h^2 * mu * V)
                float h = Time.fixedDeltaTime;  // ~0.02
                float inv_h2 = 1f / (h * h);

                // ★ 跳过极小体积 tet 的约束（防止 alpha 爆炸）
                if (vol < minVolForConstraint)
                {
                    alphaDev[i] = 0f;
                    alphaHyd[i] = 0f;
                }
                else
                {
                    alphaDev[i] = (_lameMu > 0f) ? inv_h2 / (_lameMu * vol) : 0f;
                    alphaHyd[i] = (_lameLambda > 0f) ? inv_h2 / (_lameLambda * vol) : 0f;

                    // ★ 钳位 alpha 防止数值爆炸
                    float maxAlpha = 1e8f;
                    alphaDev[i] = Mathf.Min(alphaDev[i], maxAlpha);
                    alphaHyd[i] = Mathf.Min(alphaHyd[i], maxAlpha);
                }
            }

            // mass → invMass
            // ★ Step 3 (cut debris fix): compute the average MASS
            // (not invMass) using the median-ish "robust" mean, then
            // enforce a hard mass floor at avgMass / 100 to absolutely
            // bound the worst-case inverse mass. Averaging invMass
            // directly is non-robust: a single 1e-11 m^3 sliver tet
            // produces an invMass of ~4e+8 which then drives
            // fallbackInvMass into the 1e+7 range and lets the
            // fallbackInvMass*10 clamp accept invMasses up to 1e+8.
            // The observed [SepDiag InvMass max=3.478E+08] is exactly
            // that failure mode.
            float sumParticleMass = 0f;
            int countParticleMass = 0;
            for (int i = 0; i < _numP; i++)
            {
                if (data.InvMass[i] == 0f) continue;
                if (mass[i] > 1e-12f)
                {
                    sumParticleMass += mass[i];
                    countParticleMass++;
                }
            }
            float avgParticleMass = countParticleMass > 0 ? sumParticleMass / countParticleMass : 1f;
            // ★ Step 4 (XPBD conditioning): tighten the mass floor to
            // avgMass * 0.1. With the previous 0.01 factor the invMass
            // ratio max/min could still reach ~10000:1, which the
            // GPU XPBD constraint solver cannot handle gracefully --
            // heavy particles get accelerated into light neighbours
            // and the cut surface ends up with the "tassel / pin-hole"
            // pattern observed in the screenshot. 0.1 caps the ratio
            // at ~100:1, well inside the well-conditioned regime, at
            // the cost of treating sliver-incident particles as if
            // they were slightly heavier than their (post-cut) tets
            // would assign. This is acceptable because sliver tets are
            // physical artefacts of the cutting algorithm anyway, not
            // a faithful sampling of the material distribution.
            float minReasonableMass = avgParticleMass * 0.1f;

            int massFloorHits = 0;
            for (int i = 0; i < _numP; i++)
            {
                if (data.InvMass[i] == 0f)
                {
                    invMass[i] = 0f;
                    continue;
                }
                float m = mass[i];
                if (m < minReasonableMass)
                {
                    m = minReasonableMass;
                    massFloorHits++;
                }
                invMass[i] = 1f / m;
            }
            if (massFloorHits > 0)
            {
                Debug.Log($"[XPBDSolverGPU] mass floor applied to {massFloorHits} particles " +
                          $"(avgParticleMass={avgParticleMass:G4}, floor={minReasonableMass:G4})");
            }

            // ── 建边 ─────────────────────────────────────────
            BuildEdges(data, restPos, out int[] edgeIds, out float[] restLens);
            _numE = edgeIds.Length / 2;

            // ── Graph Coloring ───────────────────────────────
            BuildTetColorGroups(data, out int[] colorFlat,
                out _groupOff, out _groupCnt, out _numColors);
            BuildEdgeColorGroups(edgeIds, _numE, _numP,
                out int[] edgeColorFlat,
                out _edgeGroupOff, out _edgeGroupCnt, out _numEdgeColors);
            BuildSurfaceTriColorGroups(data.SurfaceTriIds, _numSurfaceTris, _numP,
                out int[] surfaceTriColorFlat,
                out _surfaceGroupOff, out _surfaceGroupCnt, out _numSurfaceColors);

            // ── 上传 GPU Buffer ──────────────────────────────
            _bufPos      = MkV4(ToV4(curPos, _numP));
            _bufPrevPos  = MkV4(ToV4(data.PrevPositions, _numP));
            _bufVel      = MkV4(ToV4(data.Velocities, _numP)); // 使用实际速度,不清零!
            _bufInvMass  = MkF(invMass);

            _bufTetIds   = MkU4(ToU4(data.TetIds, _numT));
            _bufRestVol  = MkF(restVolumes);
            int[] actI = new int[_numT];
            for (int i = 0; i < _numT; i++) actI[i] = data.TetActive[i] ? 1 : 0;
            _bufTetActive = MkI(actI);

            // NeoHookean 专用 buffer
            _bufInvRestMatrix    = MkV4(invRestMat);
            _bufAlphaDeviatoric  = MkF(alphaDev);
            _bufAlphaHydrostatic = MkF(alphaHyd);

            _bufEdgeIds  = MkU2(ToU2(edgeIds, _numE));
            _bufRestLen  = MkF(restLens);
            int[] edgeActI = new int[_numE];
            for (int i = 0; i < _numE; i++) edgeActI[i] = 1;
            _bufEdgeActive = MkI(edgeActI);

            _bufColorFlat     = MkI(colorFlat);
            _bufEdgeColorFlat = MkI(edgeColorFlat);
            if (_numSurfaceTris > 0)
            {
                _bufSurfaceTriIds = MkU3(ToU3(data.SurfaceTriIds, _numSurfaceTris));
                _bufSurfaceTriColorFlat = MkI(surfaceTriColorFlat);
            }

            _readBuf = new Vector4[_numP];
            BindAll();

            // CPU 端状态
            _edgeIdsFlat = edgeIds;
            _edgeActive = new bool[_numE];
            for (int i = 0; i < _numE; i++) _edgeActive[i] = true;
            _tetActive = new bool[_numT];
            for (int i = 0; i < _numT; i++) _tetActive[i] = data.TetActive[i];

            // ── 诊断信息 ─────────────────────────────────────
            int pinnedCount = 0, zeroMassCount = 0, degenTets = 0;
            float minInvM = float.MaxValue, maxInvM = 0f;
            float minVol = float.MaxValue, maxVol = 0f;
            float minAlphaD = float.MaxValue, maxAlphaD = 0f;
            float maxBnorm = 0f;

            for (int i = 0; i < _numP; i++)
            {
                if (invMass[i] == 0f) pinnedCount++;
                if (mass[i] < 1e-12f && invMass[i] == 0f && data.InvMass[i] != 0f) zeroMassCount++;
                if (invMass[i] > 0f) { minInvM = Mathf.Min(minInvM, invMass[i]); maxInvM = Mathf.Max(maxInvM, invMass[i]); }
            }
            for (int i = 0; i < _numT; i++)
            {
                if (restVolumes[i] < 1e-15f) { degenTets++; continue; }
                minVol = Mathf.Min(minVol, restVolumes[i]);
                maxVol = Mathf.Max(maxVol, restVolumes[i]);
                if (alphaDev[i] > 0f) { minAlphaD = Mathf.Min(minAlphaD, alphaDev[i]); maxAlphaD = Mathf.Max(maxAlphaD, alphaDev[i]); }
                // B matrix norm
                Vector3 br0 = new Vector3(invRestMat[i*3].x, invRestMat[i*3].y, invRestMat[i*3].z);
                Vector3 br1 = new Vector3(invRestMat[i*3+1].x, invRestMat[i*3+1].y, invRestMat[i*3+1].z);
                Vector3 br2 = new Vector3(invRestMat[i*3+2].x, invRestMat[i*3+2].y, invRestMat[i*3+2].z);
                float bn = br0.magnitude + br1.magnitude + br2.magnitude;
                maxBnorm = Mathf.Max(maxBnorm, bn);
            }

            Debug.Log($"[XPBDSolverGPU] NeoHookean Init | P:{_numP} T:{_numT} E:{_numE} SurfaceTris:{_numSurfaceTris} | " +
                      $"Colors:{_numColors} EdgeColors:{_numEdgeColors} SurfaceColors:{_numSurfaceColors}");
            Debug.Log($"[XPBDSolverGPU] Lamé: λ={_lameLambda:G4} μ={_lameMu:G4} | " +
                      $"E={YoungsModulus:G4} ν={PoissonsRatio}");
            Debug.Log($"[XPBDSolverGPU] InvMass: [{minInvM:G4}, {maxInvM:G4}] | " +
                      $"Pinned: {pinnedCount} | ZeroMass(bug): {zeroMassCount}");
            Debug.Log($"[XPBDSolverGPU] RestVol: [{minVol:G4}, {maxVol:G4}] | " +
                      $"DegenTets: {degenTets}/{_numT}");
            Debug.Log($"[XPBDSolverGPU] AlphaDev: [{minAlphaD:G4}, {maxAlphaD:G4}] | " +
                      $"MaxBnorm: {maxBnorm:G4}");
        }

        // ══════════════════════════════════════════════════════
        // Step — 每帧调用
        // ══════════════════════════════════════════════════════
        public void Step(float dt)
        {
            if (_bufPos == null || _bufInvMass == null) return;
            BindAll();

            float sdt = dt / NumSubSteps;
            float sdt2 = sdt * sdt;

            _cs.SetFloat("_Dt",       sdt);
            _cs.SetFloat("_GravityX", Gravity.x);
            _cs.SetFloat("_GravityY", Gravity.y);
            _cs.SetFloat("_GravityZ", Gravity.z);
            _cs.SetFloat("_EdgeAlpha", EdgeCompliance / sdt2);
            _cs.SetFloat("_Damping",  Damping);
            _cs.SetFloat("_Omega",    0.2f);   // SOR 松弛: GS 每个约束只应用 20% 校正
            _cs.SetFloat("_GroundY",  GroundY);
            _cs.SetInt("_NumParticles", _numP);
            _cs.SetInt("_NumTets",      _numT);
            _cs.SetInt("_NumEdges",     _numE);
            _cs.SetInt("_NumSurfaceTris", _numSurfaceTris);
            _cs.SetFloat("_ToolContactDistance", ToolContactDistance);
            _cs.SetFloat("_ToolContactAlpha", ToolContactCompliance / sdt2);

            // GS (图着色) 比 Jacobi 收敛快得多
            // FantasyVR 用 Jacobi 需要 10 次, GS 只需 1 次
            int constraintIter = 1;

            for (int sub = 0; sub < NumSubSteps; sub++)
            {
                // 1. 积分 (semi-euler)
                Go(_kIntegrate, _numP);

                // 2. 约束投影 — 迭代多次 (关键！)
                for (int iter = 0; iter < constraintIter; iter++)
                {
                    // 2a. 边约束
                    for (int c = 0; c < _numEdgeColors; c++)
                    {
                        _cs.SetInt("_GroupOffset", _edgeGroupOff[c]);
                        _cs.SetInt("_GroupCount",  _edgeGroupCnt[c]);
                        Go(_kSolveEdges, _edgeGroupCnt[c]);
                    }

                    // 2b. NeoHookean: Deviatoric + Hydrostatic
                    for (int c = 0; c < _numColors; c++)
                    {
                        _cs.SetInt("_GroupOffset", _groupOff[c]);
                        _cs.SetInt("_GroupCount",  _groupCnt[c]);
                        Go(_kSolveNHDev, _groupCnt[c]);
                        Go(_kSolveNHHyd, _groupCnt[c]);
                    }
                }

                // 3. 夹爪碰撞（第一次 — 弹性约束后）
                SolveToolContacts();

                // 4. 地面碰撞
                Go(_kGroundCollision, _numP);

                // 5. 内部约束和接触交替求解，让局部压陷能扩散到体内，同时保持最终不穿模
                int couplingPasses = Mathf.Max(1, ToolContactCouplingPasses);
                for (int pass = 0; pass < couplingPasses; pass++)
                {
                    SolveInternalOnce();
                    SolveToolContacts();
                }

                // 6. 更新速度 + 阻尼
                Go(_kPostSolve, _numP);
            }
        }

        // ══════════════════════════════════════════════════════
        // 读回位置
        // ══════════════════════════════════════════════════════
        void SolveInternalOnce()
        {
            for (int c = 0; c < _numEdgeColors; c++)
            {
                _cs.SetInt("_GroupOffset", _edgeGroupOff[c]);
                _cs.SetInt("_GroupCount",  _edgeGroupCnt[c]);
                Go(_kSolveEdges, _edgeGroupCnt[c]);
            }

            for (int c = 0; c < _numColors; c++)
            {
                _cs.SetInt("_GroupOffset", _groupOff[c]);
                _cs.SetInt("_GroupCount",  _groupCnt[c]);
                Go(_kSolveNHDev, _groupCnt[c]);
                Go(_kSolveNHHyd, _groupCnt[c]);
            }
        }

        void SolveToolContacts()
        {
            if (_numSurfaceTris <= 0 || _numSurfaceColors <= 0 ||
                _bufSurfaceTriIds == null || _bufSurfaceTriColorFlat == null)
                return;

            int iterations = Mathf.Max(1, ToolContactIterations);
            for (int iter = 0; iter < iterations; iter++)
            {
                for (int c = 0; c < _numSurfaceColors; c++)
                {
                    _cs.SetInt("_SurfaceGroupOffset", _surfaceGroupOff[c]);
                    _cs.SetInt("_SurfaceGroupCount",  _surfaceGroupCnt[c]);
                    Go(_kSolveSurfaceToolContacts, _surfaceGroupCnt[c]);
                }
            }
        }

        public void ReadbackPositions(Core.TetMeshData data)
        {
            _bufPos.GetData(_readBuf);
            for (int i = 0; i < _numP; i++)
                data.Positions[i] = new Vector3(_readBuf[i].x, _readBuf[i].y, _readBuf[i].z);
        }

        /// <summary>
        /// 回读全部物理状态 (位置+速度+prevPositions)
        /// 在 Dispose+Init 之前调用, 确保 CPU 端数据是最新的
        /// </summary>
        public void ReadbackAll(Core.TetMeshData data)
        {
            // 位置
            _bufPos.GetData(_readBuf);
            for (int i = 0; i < _numP; i++)
                data.Positions[i] = new Vector3(_readBuf[i].x, _readBuf[i].y, _readBuf[i].z);

            // 速度
            var velBuf = new Vector4[_numP];
            _bufVel.GetData(velBuf);
            for (int i = 0; i < _numP; i++)
                data.Velocities[i] = new Vector3(velBuf[i].x, velBuf[i].y, velBuf[i].z);

            // 上一帧位置
            var prevBuf = new Vector4[_numP];
            _bufPrevPos.GetData(prevBuf);
            for (int i = 0; i < _numP; i++)
                data.PrevPositions[i] = new Vector3(prevBuf[i].x, prevBuf[i].y, prevBuf[i].z);
        }

        public void UploadPositionsAndVelocities(Core.TetMeshData data)
        {
            var posV4  = ToV4(data.Positions, _numP);
            var velV4  = ToV4(data.Velocities, _numP);
            var prevV4 = ToV4(data.PrevPositions, _numP);
            _bufPos.SetData(posV4);
            _bufVel.SetData(velV4);
            _bufPrevPos.SetData(prevV4);
        }

        public ComputeBuffer GetPositionsBuffer() => _bufPos;

        /// <summary>
        /// 上传 InvMass 到 GPU（用于夹取时锁定/解锁粒子）
        /// 将 data.InvMass 直接写入 GPU buffer，无需完整 Dispose+Init
        /// </summary>
        public void UploadInvMass(Core.TetMeshData data)
        {
            if (_bufInvMass == null) return;
            var invM = new float[_numP];
            int count = Mathf.Min(_numP, data.NumParticles);
            for (int i = 0; i < count; i++)
                invM[i] = data.InvMass[i];
            _bufInvMass.SetData(invM);
        }

        /// <summary>
        /// 设置胶囊体碰撞参数（每帧调用，在 Step 之前）
        /// 3个胶囊体: 上颚 + 下颚 + 杆身
        /// 参考 SOFA collision pipeline
        /// </summary>
        public void SetCapsuleCollisionParams(
            Vector3 cap0A, Vector3 cap0B, float cap0R,
            Vector3 cap1A, Vector3 cap1B, float cap1R,
            Vector3 cap2A, Vector3 cap2B, float cap2R,
            Vector3 prevCap0A, Vector3 prevCap0B,
            Vector3 prevCap1A, Vector3 prevCap1B,
            Vector3 prevCap2A, Vector3 prevCap2B,
            int numCapsules)
        {
            _cs.SetVector("_Capsule0A", new Vector4(cap0A.x, cap0A.y, cap0A.z, cap0R));
            _cs.SetVector("_Capsule0B", new Vector4(cap0B.x, cap0B.y, cap0B.z, 0f));
            _cs.SetVector("_Capsule1A", new Vector4(cap1A.x, cap1A.y, cap1A.z, cap1R));
            _cs.SetVector("_Capsule1B", new Vector4(cap1B.x, cap1B.y, cap1B.z, 0f));
            _cs.SetVector("_Capsule2A", new Vector4(cap2A.x, cap2A.y, cap2A.z, cap2R));
            _cs.SetVector("_Capsule2B", new Vector4(cap2B.x, cap2B.y, cap2B.z, 0f));
            _cs.SetVector("_PrevCapsule0A", new Vector4(prevCap0A.x, prevCap0A.y, prevCap0A.z, cap0R));
            _cs.SetVector("_PrevCapsule0B", new Vector4(prevCap0B.x, prevCap0B.y, prevCap0B.z, 0f));
            _cs.SetVector("_PrevCapsule1A", new Vector4(prevCap1A.x, prevCap1A.y, prevCap1A.z, cap1R));
            _cs.SetVector("_PrevCapsule1B", new Vector4(prevCap1B.x, prevCap1B.y, prevCap1B.z, 0f));
            _cs.SetVector("_PrevCapsule2A", new Vector4(prevCap2A.x, prevCap2A.y, prevCap2A.z, cap2R));
            _cs.SetVector("_PrevCapsule2B", new Vector4(prevCap2B.x, prevCap2B.y, prevCap2B.z, 0f));
            _cs.SetVector("_ToolBBoxMin", Vector4.zero);
            _cs.SetVector("_ToolBBoxMax", new Vector4(0f, 0f, 0f, numCapsules));
        }

        public void UploadTetActiveBuffer(bool[] ta)
        {
            var a = new int[_numT];
            for (int i = 0; i < _numT; i++) a[i] = ta[i] ? 1 : 0;
            _bufTetActive.SetData(a);

            if (_edgeToTets != null)
            {
                var ea = new int[_numE];
                for (int e = 0; e < _numE; e++)
                {
                    bool active = false;
                    foreach (int t in _edgeToTets[e])
                    {
                        if (ta[t]) { active = true; break; }
                    }
                    ea[e] = active ? 1 : 0;
                }
                _bufEdgeActive.SetData(ea);
            }
        }

        /// <summary>
        /// 增量更新 TetActive — 不做 full reinit, 只上传 active 标记
        /// 切割时调用此方法, 比 Dispose+Init 快 50-100x
        /// </summary>
        public void UpdateTetActive(Core.TetMeshData data)
        {
            int n = Mathf.Min(_numT, data.NumTets);
            bool[] ta = new bool[_numT];
            for (int i = 0; i < n; i++) ta[i] = data.TetActive[i];
            UploadTetActiveBuffer(ta);
        }

        public void Dispose()
        {
            _bufPos?.Release(); _bufPrevPos?.Release(); _bufVel?.Release();
            _bufInvMass?.Release(); _bufTetIds?.Release(); _bufRestVol?.Release();
            _bufTetActive?.Release(); _bufEdgeIds?.Release(); _bufRestLen?.Release();
            _bufEdgeActive?.Release();
            _bufColorFlat?.Release(); _bufEdgeColorFlat?.Release();
            _bufSurfaceTriIds?.Release(); _bufSurfaceTriColorFlat?.Release();
            _bufInvRestMatrix?.Release();
            _bufAlphaDeviatoric?.Release(); _bufAlphaHydrostatic?.Release();
        }

        // ── 切割支持 API ─────────────────────────────────────
        public int[] EdgeIds => _edgeIdsFlat;
        public int NumEdges => _numE;
        public int NumParticles => _numP;
        public bool[] EdgeActive => _edgeActive;
        public bool[] TetActive => _tetActive;

        int[] _edgeIdsFlat;
        bool[] _edgeActive;
        bool[] _tetActive;

        public void DisableEdges(HashSet<int> edgeIndices, bool[] tetActiveOverride = null)
        {
            if (edgeIndices == null || edgeIndices.Count == 0) return;
            foreach (int e in edgeIndices)
                if (e >= 0 && e < _numE) _edgeActive[e] = false;

            var eaBuf = new int[_numE];
            for (int i = 0; i < _numE; i++) eaBuf[i] = _edgeActive[i] ? 1 : 0;
            _bufEdgeActive.SetData(eaBuf);

            if (_edgeToTets != null)
            {
                var tetDisabledCount = new int[_numT];
                for (int e = 0; e < _numE; e++)
                {
                    if (!_edgeActive[e])
                        foreach (int t in _edgeToTets[e])
                            tetDisabledCount[t]++;
                }
                for (int t = 0; t < _numT; t++)
                {
                    if (tetDisabledCount[t] >= 3)
                        _tetActive[t] = false;
                }
                if (tetActiveOverride != null)
                    for (int t = 0; t < _numT; t++)
                        if (!tetActiveOverride[t]) _tetActive[t] = false;

                var taBuf = new int[_numT];
                for (int t = 0; t < _numT; t++) taBuf[t] = _tetActive[t] ? 1 : 0;
                _bufTetActive.SetData(taBuf);
            }
        }

        public void DisableEdgesOnly(HashSet<int> edgeIndices)
        {
            if (edgeIndices == null || edgeIndices.Count == 0) return;
            foreach (int e in edgeIndices)
                if (e >= 0 && e < _numE) _edgeActive[e] = false;

            var eaBuf = new int[_numE];
            for (int i = 0; i < _numE; i++) eaBuf[i] = _edgeActive[i] ? 1 : 0;
            _bufEdgeActive.SetData(eaBuf);
        }

        // ── 内部方法 ─────────────────────────────────────────

        void BindAll()
        {
            int[] ks = { _kIntegrate, _kSolveEdges, _kSolveNHDev,
                         _kSolveNHHyd, _kPostSolve, _kGroundCollision };
            foreach (int k in ks)
            {
                _cs.SetBuffer(k, "_Positions",     _bufPos);
                _cs.SetBuffer(k, "_PrevPositions", _bufPrevPos);
                _cs.SetBuffer(k, "_Velocities",    _bufVel);
                _cs.SetBuffer(k, "_InvMasses",     _bufInvMass);
                _cs.SetBuffer(k, "_TetIds",        _bufTetIds);
                _cs.SetBuffer(k, "_RestVolumes",   _bufRestVol);
                _cs.SetBuffer(k, "_TetActive",     _bufTetActive);
                _cs.SetBuffer(k, "_EdgeIds",       _bufEdgeIds);
                _cs.SetBuffer(k, "_RestLengths",   _bufRestLen);
                _cs.SetBuffer(k, "_ColorGroupFlat",    _bufColorFlat);
                _cs.SetBuffer(k, "_EdgeColorGroupFlat",_bufEdgeColorFlat);
            }
            _cs.SetBuffer(_kSolveEdges, "_EdgeActive", _bufEdgeActive);

            // NeoHookean buffers — only needed by NH kernels
            int[] nhKernels = { _kSolveNHDev, _kSolveNHHyd };
            foreach (int k in nhKernels)
            {
                _cs.SetBuffer(k, "_InvRestMatrix",    _bufInvRestMatrix);
                _cs.SetBuffer(k, "_AlphaDeviatoric",  _bufAlphaDeviatoric);
                _cs.SetBuffer(k, "_AlphaHydrostatic", _bufAlphaHydrostatic);
            }

            // Tool collision — 需要 _Positions, _PrevPositions(速度修正), _InvMasses
            _cs.SetBuffer(_kToolCollision, "_Positions", _bufPos);
            _cs.SetBuffer(_kToolCollision, "_PrevPositions", _bufPrevPos);
            _cs.SetBuffer(_kToolCollision, "_InvMasses", _bufInvMass);

            if (_bufSurfaceTriIds != null && _bufSurfaceTriColorFlat != null)
            {
                _cs.SetBuffer(_kSolveSurfaceToolContacts, "_Positions", _bufPos);
                _cs.SetBuffer(_kSolveSurfaceToolContacts, "_PrevPositions", _bufPrevPos);
                _cs.SetBuffer(_kSolveSurfaceToolContacts, "_InvMasses", _bufInvMass);
                _cs.SetBuffer(_kSolveSurfaceToolContacts, "_SurfaceTriIds", _bufSurfaceTriIds);
                _cs.SetBuffer(_kSolveSurfaceToolContacts, "_SurfaceTriColorGroupFlat", _bufSurfaceTriColorFlat);
            }
        }

        void Go(int k, int n)
        {
            int g = (n + TH - 1) / TH;
            if (g > 0) _cs.Dispatch(k, g, 1, 1);
        }

        void BuildEdges(Core.TetMeshData data, Vector3[] pos,
            out int[] edgeIds, out float[] restLengths)
        {
            var edgeSet = new Dictionary<long, int>();
            var eidList = new List<int>();
            var rlList  = new List<float>();
            var e2tList = new List<List<int>>();

            int[,] pairs = {{0,1},{0,2},{0,3},{1,2},{1,3},{2,3}};
            for (int t = 0; t < _numT; t++)
            {
                if (!data.TetActive[t]) continue;
                for (int p2 = 0; p2 < 6; p2++)
                {
                    int a = data.TetIds[t*4+pairs[p2,0]], b = data.TetIds[t*4+pairs[p2,1]];
                    if (a > b) { int tmp=a; a=b; b=tmp; }
                    long key = (long)a * 200000 + b;
                    if (!edgeSet.ContainsKey(key))
                    {
                        int ei = eidList.Count / 2;
                        edgeSet[key] = ei;
                        eidList.Add(a); eidList.Add(b);
                        rlList.Add((pos[a] - pos[b]).magnitude);
                        e2tList.Add(new List<int> { t });
                    }
                    else
                    {
                        e2tList[edgeSet[key]].Add(t);
                    }
                }
            }
            edgeIds = eidList.ToArray();
            restLengths = rlList.ToArray();
            _edgeToTets = new List<int>[e2tList.Count];
            for (int i = 0; i < e2tList.Count; i++)
                _edgeToTets[i] = e2tList[i];
        }

        void BuildTetColorGroups(Core.TetMeshData data,
            out int[] flat, out int[] offsets, out int[] counts, out int numColors)
        {
            var groups = Core.GraphColoring.Compute(data.TetIds, data.NumTets, data.NumParticles, data.TetActive);
            numColors = groups.Count;
            offsets = new int[numColors]; counts = new int[numColors];
            int total = 0;
            for (int i = 0; i < numColors; i++) total += groups[i].Length;
            flat = new int[total];
            int off = 0;
            for (int i = 0; i < numColors; i++)
            {
                offsets[i] = off; counts[i] = groups[i].Length;
                Array.Copy(groups[i], 0, flat, off, groups[i].Length);
                off += groups[i].Length;
            }
        }

        void BuildEdgeColorGroups(int[] edgeIds, int numEdges, int numParticles,
            out int[] flat, out int[] offsets, out int[] counts, out int numColors)
        {
            int[] color = new int[numEdges];
            Array.Fill(color, -1);
            var vertToEdges = new List<int>[numParticles];
            for (int i = 0; i < numParticles; i++) vertToEdges[i] = new List<int>(8);
            for (int e = 0; e < numEdges; e++)
            {
                vertToEdges[edgeIds[e*2]].Add(e);
                vertToEdges[edgeIds[e*2+1]].Add(e);
            }
            int maxColor = 0;
            for (int e = 0; e < numEdges; e++)
            {
                int a = edgeIds[e*2], b = edgeIds[e*2+1];
                var used = new HashSet<int>();
                foreach (int n in vertToEdges[a]) if (color[n] >= 0) used.Add(color[n]);
                foreach (int n in vertToEdges[b]) if (color[n] >= 0) used.Add(color[n]);
                int c2 = 0; while (used.Contains(c2)) c2++;
                color[e] = c2;
                if (c2 > maxColor) maxColor = c2;
            }
            numColors = maxColor + 1;
            var groups = new List<int>[numColors];
            for (int i = 0; i < numColors; i++) groups[i] = new List<int>();
            for (int e = 0; e < numEdges; e++) groups[color[e]].Add(e);
            offsets = new int[numColors]; counts = new int[numColors];
            int total = 0;
            for (int i = 0; i < numColors; i++) total += groups[i].Count;
            flat = new int[total]; int off = 0;
            for (int i = 0; i < numColors; i++)
            {
                offsets[i] = off; counts[i] = groups[i].Count;
                groups[i].CopyTo(flat, off);
                off += groups[i].Count;
            }
        }

        // ── Buffer helpers ──────────────────────────────────
        void BuildSurfaceTriColorGroups(int[] triIds, int numTris, int numParticles,
            out int[] flat, out int[] offsets, out int[] counts, out int numColors)
        {
            if (triIds == null || numTris <= 0)
            {
                flat = Array.Empty<int>();
                offsets = Array.Empty<int>();
                counts = Array.Empty<int>();
                numColors = 0;
                return;
            }

            int[] color = new int[numTris];
            Array.Fill(color, -1);
            var vertToTris = new List<int>[numParticles];
            for (int i = 0; i < numParticles; i++) vertToTris[i] = new List<int>(8);

            for (int t = 0; t < numTris; t++)
            {
                vertToTris[triIds[t * 3 + 0]].Add(t);
                vertToTris[triIds[t * 3 + 1]].Add(t);
                vertToTris[triIds[t * 3 + 2]].Add(t);
            }

            int maxColor = 0;
            for (int t = 0; t < numTris; t++)
            {
                int a = triIds[t * 3 + 0];
                int b = triIds[t * 3 + 1];
                int c = triIds[t * 3 + 2];
                var used = new HashSet<int>();
                foreach (int n in vertToTris[a]) if (color[n] >= 0) used.Add(color[n]);
                foreach (int n in vertToTris[b]) if (color[n] >= 0) used.Add(color[n]);
                foreach (int n in vertToTris[c]) if (color[n] >= 0) used.Add(color[n]);
                int chosen = 0; while (used.Contains(chosen)) chosen++;
                color[t] = chosen;
                if (chosen > maxColor) maxColor = chosen;
            }

            numColors = maxColor + 1;
            var groups = new List<int>[numColors];
            for (int i = 0; i < numColors; i++) groups[i] = new List<int>();
            for (int t = 0; t < numTris; t++) groups[color[t]].Add(t);

            offsets = new int[numColors];
            counts = new int[numColors];
            int total = 0;
            for (int i = 0; i < numColors; i++) total += groups[i].Count;
            flat = new int[total];

            int off = 0;
            for (int i = 0; i < numColors; i++)
            {
                offsets[i] = off;
                counts[i] = groups[i].Count;
                groups[i].CopyTo(flat, off);
                off += groups[i].Count;
            }
        }

        ComputeBuffer MkV4(Vector4[] d) { var b=new ComputeBuffer(d.Length,16); b.SetData(d); return b; }
        ComputeBuffer MkF(float[] d)    { var b=new ComputeBuffer(d.Length,4);  b.SetData(d); return b; }
        ComputeBuffer MkI(int[] d)      { var b=new ComputeBuffer(d.Length,4);  b.SetData(d); return b; }
        ComputeBuffer MkU4(UInt4[] d)   { var b=new ComputeBuffer(d.Length,16); b.SetData(d); return b; }
        ComputeBuffer MkU2(UInt2[] d)   { var b=new ComputeBuffer(d.Length,8);  b.SetData(d); return b; }
        ComputeBuffer MkU3(UInt3[] d)   { var b=new ComputeBuffer(d.Length,12); b.SetData(d); return b; }

        static Vector4[] ToV4(Vector3[] v, int count) {
            var r=new Vector4[count];
            for(int i=0;i<count;i++) r[i]=new Vector4(v[i].x,v[i].y,v[i].z,0);
            return r;
        }
        static UInt4[] ToU4(int[] f,int n) {
            var a=new UInt4[n];
            for(int i=0;i<n;i++) a[i]=new UInt4{x=(uint)f[i*4],y=(uint)f[i*4+1],z=(uint)f[i*4+2],w=(uint)f[i*4+3]};
            return a;
        }
        static UInt2[] ToU2(int[] f,int n) {
            var a=new UInt2[n];
            for(int i=0;i<n;i++) a[i]=new UInt2{x=(uint)f[i*2],y=(uint)f[i*2+1]};
            return a;
        }
        static UInt3[] ToU3(int[] f,int n) {
            var a=new UInt3[n];
            for(int i=0;i<n;i++) a[i]=new UInt3{x=(uint)f[i*3],y=(uint)f[i*3+1],z=(uint)f[i*3+2]};
            return a;
        }
    }
}
