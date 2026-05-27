// TetMeshData.cs
// 四面體網格核心數據結構
// 兼容 Matthias Müller Ten-Minute-Physics Blender TetPlugin 導出的 JSON 格式
// 參考: https://github.com/Habrador/Ten-Minute-Physics-Unity (MIT License)
// 數據佈局為平坦數組 (flat array)，便於直接上傳 GPU ComputeBuffer

using UnityEngine;

namespace SurgicalSim.Core
{
    /// <summary>
    /// JSON 反序列化容器（對應 Blender TetPlugin 導出格式）
    /// verts: [x0,y0,z0, x1,y1,z1, ...]  平坦頂點坐標
    /// tetIds: [i0,i1,i2,i3, i0,i1,i2,i3, ...] 四面體頂點索引（每4個一組）
    /// tetSurfaceTriIds: [i0,i1,i2, ...] 邊界表面三角形索引（每3個一組）
    /// </summary>
    [System.Serializable]
    public class TetMeshJson
    {
        public float[] verts;             // 頂點坐標（平坦）
        public int[]   tetIds;            // 四面體索引（平坦，每4個一組）
        public int[]   tetSurfaceTriIds;  // 邊界三角形索引（平坦，每3個一組）
        public int[]   tetEdgeIds;        // 用於 debug 顯示的邊（可選）
    }

    /// <summary>
    /// 運行時四面體網格數據
    /// 所有數組均為 GPU-friendly 的平坦佈局
    /// </summary>
    public class TetMeshData
    {
        // ── 拓撲（初始化後不變）──────────────────────────────
        public int   NumParticles  { get; private set; }
        public int   NumTets       { get; private set; }
        public int   NumSurfaceTris { get; private set; }

        /// <summary>四面體頂點索引，平坦數組，長度 = NumTets * 4</summary>
        public int[] TetIds        { get; private set; }

        /// <summary>邊界三角形頂點索引，平坦數組，長度 = NumSurfaceTris * 3</summary>
        public int[] SurfaceTriIds { get; private set; }

        // ── 粒子狀態（每幀由 XPBD Solver 更新）──────────────
        /// <summary>當前粒子位置 [numParticles]</summary>
        public Vector3[] Positions    { get; set; }

        /// <summary>靜止位置（初始化後不變）[numParticles]</summary>
        public Vector3[] RestPositions { get; set; }

        /// <summary>上一幀位置，用於 XPBD 速度計算 [numParticles]</summary>
        public Vector3[] PrevPositions { get; set; }

        /// <summary>速度 [numParticles]</summary>
        public Vector3[] Velocities    { get; set; }

        /// <summary>質量倒數 [numParticles]。0 = 固定點（無限質量）</summary>
        public float[] InvMass         { get; set; }

        // ── 切割狀態 ─────────────────────────────────────────
        /// <summary>四面體是否存活 [numTets]。false = 已被切除</summary>
        public bool[] TetActive        { get; set; }

        // ── 約束預計算（初始化後不變）────────────────────────
        /// <summary>各四面體的靜止體積 [numTets]</summary>
        public float[] RestVolumes     { get; private set; }

        // ── 圖著色分組（GPU 並行求解用）──────────────────────
        /// <summary>顏色組，每個元素是該組包含的 tet 索引列表</summary>
        public int[][] ColorGroups     { get; set; }

        // ─────────────────────────────────────────────────────
        /// <summary>
        /// 從 JSON 數據初始化網格（CPU 側）
        /// </summary>
        public void InitFromJson(TetMeshJson json, float massDensity = 1000f)
        {
            int vertCount = json.verts.Length / 3;
            int tetCount  = json.tetIds.Length / 4;

            NumParticles   = vertCount;
            NumTets        = tetCount;
            NumSurfaceTris = json.tetSurfaceTriIds != null
                           ? json.tetSurfaceTriIds.Length / 3
                           : 0;

            // 拷貝索引數組
            TetIds        = json.tetIds;
            SurfaceTriIds = json.tetSurfaceTriIds ?? new int[0];

            // 初始化粒子數組
            Positions     = new Vector3[vertCount];
            RestPositions = new Vector3[vertCount];
            PrevPositions = new Vector3[vertCount];
            Velocities    = new Vector3[vertCount];
            InvMass       = new float[vertCount];

            // 解析頂點坐標（Unity 坐標系：Y 軸朝上）
            for (int i = 0; i < vertCount; i++)
            {
                float x = json.verts[i * 3 + 0];
                float y = json.verts[i * 3 + 1];
                float z = json.verts[i * 3 + 2];
                Positions[i]     = new Vector3(x, y, z);
                RestPositions[i] = new Vector3(x, y, z);
                PrevPositions[i] = new Vector3(x, y, z);
            }

            // 初始化四面體狀態
            TetActive    = new bool[tetCount];
            RestVolumes  = new float[tetCount];

            float totalVol = 0f;
            for (int t = 0; t < tetCount; t++)
            {
                TetActive[t]   = true;
                RestVolumes[t] = Mathf.Abs(ComputeTetVolume(t)); // Abs：兼容Delaunay可能的負體積
                totalVol      += RestVolumes[t];
            }

            // 計算每個粒子的質量：把每個 tet 的質量分配到4個頂點
            float[] mass = new float[vertCount];
            float totalMass = massDensity * totalVol;
            float massPerTet = totalMass / tetCount;

            for (int t = 0; t < tetCount; t++)
            {
                float m = massPerTet / 4f;
                mass[TetIds[t*4+0]] += m;
                mass[TetIds[t*4+1]] += m;
                mass[TetIds[t*4+2]] += m;
                mass[TetIds[t*4+3]] += m;
            }

            for (int i = 0; i < vertCount; i++)
                InvMass[i] = (mass[i] > 0f) ? 1f / mass[i] : 0f;

            Debug.Log($"[TetMeshData] 初始化完成: {NumParticles} 粒子, " +
                      $"{NumTets} 四面體, {NumSurfaceTris} 表面三角形, " +
                      $"總體積={totalVol * 1e6f:F1} cm³"); // 1m³=1e6cm³
        }

        /// <summary>計算第 t 個四面體的有符號體積</summary>
        public float ComputeTetVolume(int t)
        {
            Vector3 p0 = Positions[TetIds[t*4+0]];
            Vector3 p1 = Positions[TetIds[t*4+1]];
            Vector3 p2 = Positions[TetIds[t*4+2]];
            Vector3 p3 = Positions[TetIds[t*4+3]];
            return Vector3.Dot(Vector3.Cross(p1 - p0, p2 - p0), p3 - p0) / 6f;
        }

        /// <summary>計算第 t 個四面體的中心位置</summary>
        public Vector3 TetCenter(int t)
        {
            return (Positions[TetIds[t*4+0]] + Positions[TetIds[t*4+1]] +
                    Positions[TetIds[t*4+2]] + Positions[TetIds[t*4+3]]) * 0.25f;
        }

        /// <summary>固定指定粒子（設 invMass = 0）</summary>
        public void PinParticle(int i)   => InvMass[i] = 0f;

        /// <summary>判斷粒子是否被固定</summary>
        public bool IsPinned(int i)      => InvMass[i] == 0f;

        // ── 動態拓撲修改（VNA vertex splitting 用）────────────

        /// <summary>更新四面體索引（vertex splitting 後）</summary>
        public void SetTetIds(int[] newIds)
        {
            TetIds = newIds;
        }

        public void SetSurfaceTriIds(int[] newIds)
        {
            SurfaceTriIds = newIds ?? new int[0];
            NumSurfaceTris = SurfaceTriIds.Length / 3;
        }

        /// <summary>更新粒子數量（vertex splitting 後）</summary>
        public void SetNumParticles(int n)
        {
            NumParticles = n;
        }

        /// <summary>設置靜止位置（vertex splitting 後需要更新）</summary>
        public void SetRestPositions(Vector3[] rp)
        {
            RestPositions = rp;
        }

        // ── 顶点分裂支持 ─────────────────────────────────────

        int _capacity = 0; // 粒子数组实际容量

        public void EnsureCapacity(int minCapacity)
        {
            if (_capacity >= minCapacity) return;
            _capacity = minCapacity;
            Positions     = ResizeVec3(Positions, _capacity);
            RestPositions = ResizeVec3(RestPositions, _capacity);
            PrevPositions = ResizeVec3(PrevPositions, _capacity);
            Velocities    = ResizeVec3(Velocities, _capacity);
            InvMass       = ResizeFloat(InvMass, _capacity);
        }

        public int AddParticle(Vector3 pos, Vector3 vel, float invMass)
        {
            int newIdx = NumParticles;
            if (newIdx >= Positions.Length)
            {
                int newCap = Mathf.Max(Positions.Length * 3 / 2, newIdx + 1);
                EnsureCapacity(newCap);
            }
            Positions[newIdx]     = pos;
            RestPositions[newIdx] = pos;
            PrevPositions[newIdx] = pos;
            Velocities[newIdx]    = vel;
            InvMass[newIdx]       = invMass;
            NumParticles = newIdx + 1;
            return newIdx;
        }

        // ── 四面体细分支持 ────────────────────────────────────

        int _tetCapacity = 0;

        /// <summary>确保 tet 数组有足够容量</summary>
        public void EnsureTetCapacity(int minCapacity)
        {
            if (_tetCapacity >= minCapacity) return;
            _tetCapacity = minCapacity;
            TetIds     = ResizeInt(TetIds, _tetCapacity * 4);
            TetActive  = ResizeBool(TetActive, _tetCapacity);
            RestVolumes = ResizeFloat(RestVolumes, _tetCapacity);
        }

        /// <summary>添加一个新四面体，返回其索引</summary>
        public int AddTet(int v0, int v1, int v2, int v3)
        {
            int newIdx = NumTets;
            if (newIdx >= _tetCapacity)
            {
                int newCap = Mathf.Max(_tetCapacity * 3 / 2, newIdx + 100);
                EnsureTetCapacity(newCap);
            }
            int b = newIdx * 4;
            TetIds[b] = v0; TetIds[b+1] = v1; TetIds[b+2] = v2; TetIds[b+3] = v3;
            TetActive[newIdx] = true;

            // 用 RestPositions 计算静止体积
            Vector3 p0 = RestPositions[v0], p1 = RestPositions[v1];
            Vector3 p2 = RestPositions[v2], p3 = RestPositions[v3];
            float vol = Mathf.Abs(Vector3.Dot(Vector3.Cross(p1 - p0, p2 - p0), p3 - p0)) / 6f;
            RestVolumes[newIdx] = vol;

            NumTets = newIdx + 1;
            return newIdx;
        }

        /// <summary>更新 RestVolumes（外部可写）</summary>
        public void DeactivateTet(int t)
        {
            if (t >= 0 && t < NumTets) TetActive[t] = false;
        }

        public void SetRestVolumes(float[] rv) { RestVolumes = rv; }

        // ── 工具方法 ─────────────────────────────────────────

        static Vector3[] ResizeVec3(Vector3[] arr, int newSize)
        {
            if (arr.Length >= newSize) return arr;
            var newArr = new Vector3[newSize];
            System.Array.Copy(arr, newArr, arr.Length);
            return newArr;
        }

        static float[] ResizeFloat(float[] arr, int newSize)
        {
            if (arr.Length >= newSize) return arr;
            var newArr = new float[newSize];
            System.Array.Copy(arr, newArr, arr.Length);
            return newArr;
        }

        static int[] ResizeInt(int[] arr, int newSize)
        {
            if (arr.Length >= newSize) return arr;
            var newArr = new int[newSize];
            System.Array.Copy(arr, newArr, arr.Length);
            return newArr;
        }

        static bool[] ResizeBool(bool[] arr, int newSize)
        {
            if (arr.Length >= newSize) return arr;
            var newArr = new bool[newSize];
            System.Array.Copy(arr, newArr, arr.Length);
            return newArr;
        }
    }
}
