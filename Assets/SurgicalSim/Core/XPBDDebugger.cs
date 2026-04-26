// XPBDDebugger.cs
// 診斷工具：在每個求解步驟後檢查位置是否有 NaN/Infinity
// 掛在 SoftBody 同一個 GameObject 上即可
// 只在 Unity Editor 模式下運行，不影響 Build

using UnityEngine;
using SurgicalSim.Core;
using SurgicalSim.Physics;

namespace SurgicalSim
{
#if UNITY_EDITOR
    [RequireComponent(typeof(SoftBody))]
    public class XPBDDebugger : MonoBehaviour
    {
        [Header("診斷設置")]
        [Tooltip("每 N 幀採樣一次（避免每幀都讀回 GPU 數據）")]
        public int sampleEveryNFrames = 5;

        [Tooltip("前 N 幀強制每幀採樣（捕獲初始爆炸）")]
        public int alwaysSampleFirstNFrames = 20;

        TetMeshData _data;
        int _frameCount = 0;
        bool _explosionDetected = false;

        void Start()
        {
            var softBody = GetComponent<SoftBody>();
            // 等網格加載後拿到數據引用
            StartCoroutine(WaitForData(softBody));
        }

        System.Collections.IEnumerator WaitForData(SoftBody softBody)
        {
            // 等待 SoftBody 初始化完成
            yield return new WaitForSeconds(0.5f);

            // 用反射拿到私有 _data 字段
            var field = typeof(SoftBody).GetField("_data",
                System.Reflection.BindingFlags.NonPublic |
                System.Reflection.BindingFlags.Instance);
            _data = field?.GetValue(softBody) as TetMeshData;

            if (_data == null)
                Debug.LogWarning("[XPBDDebugger] 無法獲取 _data，診斷停止");
            else
                Debug.Log("[XPBDDebugger] 診斷已啟動，監控粒子位置");
        }

        void LateUpdate()
        {
            if (_data == null || _explosionDetected) return;
            _frameCount++;

            bool shouldSample = (_frameCount <= alwaysSampleFirstNFrames)
                             || (_frameCount % sampleEveryNFrames == 0);
            if (!shouldSample) return;

            // 檢查位置
            int nanCount  = 0;
            int infCount  = 0;
            float maxPos  = 0f;
            Vector3 maxPosVec = Vector3.zero;

            var pos = _data.Positions;
            for (int i = 0; i < _data.NumParticles; i++)
            {
                float x = pos[i].x, y = pos[i].y, z = pos[i].z;

                if (float.IsNaN(x) || float.IsNaN(y) || float.IsNaN(z))
                {
                    nanCount++;
                }
                else if (float.IsInfinity(x) || float.IsInfinity(y) || float.IsInfinity(z))
                {
                    infCount++;
                }
                else
                {
                    float mag = Mathf.Abs(x) + Mathf.Abs(y) + Mathf.Abs(z);
                    if (mag > maxPos) { maxPos = mag; maxPosVec = pos[i]; }
                }
            }

            if (nanCount > 0 || infCount > 0 || maxPos > 100f)
            {
                _explosionDetected = true;
                Debug.LogError($"[XPBDDebugger] ⚠️ 爆炸檢測！Frame={_frameCount} | " +
                               $"NaN={nanCount} | Inf={infCount} | " +
                               $"MaxPos={maxPos:F1} 位於 {maxPosVec}");
                Debug.LogError("[XPBDDebugger] 建議：在 SoftBody 組件上暫停物理（pausePhysics=true）再截圖");
            }
            else
            {
                if (_frameCount <= alwaysSampleFirstNFrames || _frameCount % 30 == 0)
                    Debug.Log($"[XPBDDebugger] Frame={_frameCount} | " +
                              $"MaxPos={maxPos:F3}m | 無 NaN/Inf ✓");
            }
        }
    }
#endif
}
