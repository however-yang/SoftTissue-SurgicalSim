// AutoCutTest.cs — 自动化切割测试脚本
// 功能：点击按钮后自动驱动 SurgicalTool 按预设轨迹进行切割
//
// 支持 4 种测试:
//   1. 直线切割  (Linear)
//   2. 直角转弯  (RightAngle)
//   3. S 形切割  (SCurve)
//   4. 复杂综合  (Complex)
//
// ⚠ 注意：小棍初始位置靠近固定端（肝脏背面），必须先向前（+Z）移动
//         一段距离离开固定端，再横向切割，否则会切到固定端附近导致撕裂。
//
// 使用方法：
//   1. 将此脚本挂载到场景中的任意 GameObject（建议挂到 CuttingToolV3 所在对象）
//   2. 在 Inspector 中将 SurgicalTool 字段绑定到场景中的刀具对象
//   3. 点击 GUI 按钮即可运行对应测试
//   4. 测试过程中按 ESC 可中止当前测试

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using SurgicalSim.Cutting;

namespace SurgicalSim.CuttingV3
{
    public class AutoCutTest : MonoBehaviour
    {
        // ── 外部绑定 ─────────────────────────────────────────
        [Header("绑定")]
        [Tooltip("场景中的 SurgicalTool 组件")]
        public SurgicalTool surgicalTool;

        // ── 参数 ─────────────────────────────────────────────
        [Header("轨迹参数")]
        [Tooltip("测试开始前先向 +Z 方向前进的距离（脱离固定端）")]
        public float forwardClearance = 0.35f;

        [Tooltip("切割移动速度（世界单位/秒）")]
        public float cutSpeed = 0.12f;

        [Tooltip("每次切割段的长度")]
        public float segmentLength = 0.45f;

        [Tooltip("切割前等待物理稳定的帧数")]
        public int settleFrames = 30;

        [Tooltip("两段切割之间的停顿时间（秒）")]
        public float pauseBetweenSegments = 0.25f;

        // ── 状态 ─────────────────────────────────────────────
        bool _isRunning = false;
        string _currentTestName = "";
        string _statusText = "初始化中...";
        Coroutine _currentCoroutine = null;

        // 初始位置（测试开始时记录）
        Vector3 _savedPosition;

        // ── 自动初始化 ───────────────────────────────────────
        void Awake()
        {
            TryAutoFindSurgicalTool();
        }

        /// <summary>
        /// 在整个场景中搜索 SurgicalTool 组件，并绑定第一个找到的。
        /// 如果 Inspector 中已手动指定则跳过。
        /// </summary>
        void TryAutoFindSurgicalTool()
        {
            if (surgicalTool != null) return; // 已手动绑定，直接用

            // 全场景搜索（包含 inactive 对象）
            SurgicalTool[] found = FindObjectsByType<SurgicalTool>(FindObjectsInactive.Include, FindObjectsSortMode.None);
            if (found.Length == 0)
            {
                // 旧版 Unity 兼容写法
#pragma warning disable CS0618
                found = FindObjectsOfType<SurgicalTool>(true);
#pragma warning restore CS0618
            }

            if (found.Length > 0)
            {
                surgicalTool = found[0];
                _statusText = $"已自动绑定: {surgicalTool.gameObject.name}";
                Debug.Log($"[AutoCutTest] 自动绑定 SurgicalTool: {surgicalTool.gameObject.name}");
            }
            else
            {
                _statusText = "⚠ 未找到 SurgicalTool！\n请手动拖拽绑定或确认场景中存在 SurgicalTool 组件。";
                Debug.LogWarning("[AutoCutTest] 场景中未找到 SurgicalTool 组件，请手动绑定！");
            }
        }

        // ── GUI ──────────────────────────────────────────────
        void OnGUI()
        {
            float panelX = Screen.width - 230f;
            float panelY = 10f;
            float btnW = 210f;
            float btnH = 36f;
            float gap = 5f;

            // 根据绑定状态显示不同颜色的标题框
            bool toolReady = surgicalTool != null;
            GUI.Box(new Rect(panelX - 10, panelY - 5, 230, toolReady ? 275 : 175), "AutoCutTest");

            // 绑定状态行
            GUIStyle bindStyle = new GUIStyle(GUI.skin.label)
            {
                wordWrap = true, fontSize = 10,
                normal = { textColor = toolReady ? Color.green : Color.red }
            };
            string bindInfo = toolReady
                ? $"✓ 刀具: {surgicalTool.gameObject.name}"
                : "✗ 未绑定 SurgicalTool";
            GUI.Label(new Rect(panelX, panelY + 10, btnW, 20), bindInfo, bindStyle);

            // 状态文字
            GUIStyle statusStyle = new GUIStyle(GUI.skin.label) { wordWrap = true, fontSize = 10 };
            GUI.Label(new Rect(panelX, panelY + 28, btnW, 32), _statusText, statusStyle);

            float y = panelY + 62;

            if (!toolReady)
            {
                // 只显示重新搜索按钮
                if (GUI.Button(new Rect(panelX, y, btnW, btnH), "🔍 重新搜索 SurgicalTool"))
                    TryAutoFindSurgicalTool();
                return;
            }

            if (!_isRunning)
            {
                if (GUI.Button(new Rect(panelX, y, btnW, btnH), "▶ 1. 直线切割 (Linear)"))
                    StartTest(TestLinearCut(), "直线切割");
                y += btnH + gap;

                if (GUI.Button(new Rect(panelX, y, btnW, btnH), "▶ 2. 直角转弯 (RightAngle)"))
                    StartTest(TestRightAngleCut(), "直角转弯切割");
                y += btnH + gap;

                if (GUI.Button(new Rect(panelX, y, btnW, btnH), "▶ 3. S 形切割 (SCurve)"))
                    StartTest(TestSCurveCut(), "S 形切割");
                y += btnH + gap;

                if (GUI.Button(new Rect(panelX, y, btnW, btnH), "▶ 4. 复杂切割 (Complex)"))
                    StartTest(TestComplexCut(), "复杂综合切割");
                y += btnH + gap;

                if (GUI.Button(new Rect(panelX, y, btnW, 26), "🔍 重新搜索刀具"))
                    TryAutoFindSurgicalTool();
            }
            else
            {
                GUIStyle runningStyle = new GUIStyle(GUI.skin.label)
                {
                    normal = { textColor = Color.yellow },
                    fontStyle = FontStyle.Bold,
                    fontSize = 12
                };
                GUI.Label(new Rect(panelX, y, btnW, btnH), $"▶ {_currentTestName}", runningStyle);
                y += btnH + gap;

                if (GUI.Button(new Rect(panelX, y, btnW, btnH), "■ 停止 (ESC)"))
                    StopCurrentTest();
            }
        }

        void Update()
        {
            // 如果还没找到，每秒尝试一次（应对场景加载顺序问题）
            if (surgicalTool == null && Time.frameCount % 60 == 0)
                TryAutoFindSurgicalTool();

            if (_isRunning && Input.GetKeyDown(KeyCode.Escape))
                StopCurrentTest();
        }

        // ── 控制 ─────────────────────────────────────────────
        void StartTest(IEnumerator testRoutine, string name)
        {
            if (_isRunning) return;
            if (surgicalTool == null)
            {
                _statusText = "错误: 未绑定 SurgicalTool！";
                return;
            }
            _currentTestName = name;
            _isRunning = true;
            _savedPosition = GetToolPos();
            _currentCoroutine = StartCoroutine(RunTestWithWrapper(testRoutine, name));
        }

        void StopCurrentTest()
        {
            if (_currentCoroutine != null)
            {
                StopCoroutine(_currentCoroutine);
                _currentCoroutine = null;
            }
            _isRunning = false;
            _statusText = "测试已中止（ESC）";
            Debug.Log("[AutoCutTest] 测试中止");
        }

        IEnumerator RunTestWithWrapper(IEnumerator routine, string name)
        {
            _statusText = $"[{name}] 正在运行...";
            Debug.Log($"[AutoCutTest] 开始: {name}");
            yield return StartCoroutine(routine);
            _isRunning = false;
            _statusText = $"[{name}] 完成 ✓";
            Debug.Log($"[AutoCutTest] 完成: {name}");
        }

        // ── 工具位置操作 ─────────────────────────────────────
        // SurgicalTool 的位置通过其内部 _toolPos 控制，外部只能通过模拟按键输入。
        // 为了自动化测试，我们直接操控 Transform.position，
        // 并通过反射访问 _toolPos（或添加一个 SetPosition 方法）。
        // 这里用反射访问私有字段 _toolPos。

        System.Reflection.FieldInfo _toolPosField = null;

        void EnsureReflection()
        {
            if (_toolPosField == null)
                _toolPosField = typeof(SurgicalTool).GetField("_toolPos",
                    System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
        }

        Vector3 GetToolPos()
        {
            EnsureReflection();
            if (_toolPosField != null) return (Vector3)_toolPosField.GetValue(surgicalTool);
            return surgicalTool.transform.position;
        }

        void SetToolPos(Vector3 pos)
        {
            EnsureReflection();
            if (_toolPosField != null)
            {
                _toolPosField.SetValue(surgicalTool, pos);
            }
        }

        // ── 基础移动工具 ─────────────────────────────────────

        /// <summary>在 totalTime 秒内将刀从当前位置匀速移动 delta 向量</summary>
        IEnumerator MoveToolBy(Vector3 delta, float totalTime)
        {
            Vector3 start = GetToolPos();
            Vector3 end = start + delta;
            float elapsed = 0f;
            while (elapsed < totalTime)
            {
                elapsed += Time.deltaTime;
                float t = Mathf.Clamp01(elapsed / totalTime);
                SetToolPos(Vector3.Lerp(start, end, t));
                yield return null;
            }
            SetToolPos(end);
        }

        /// <summary>等待 N 帧（物理稳定）</summary>
        IEnumerator WaitFrames(int n)
        {
            for (int i = 0; i < n; i++) yield return null;
        }

        /// <summary>前置步骤：先向 +Z 方向移动 forwardClearance，脱离肝脏固定端</summary>
        IEnumerator MoveForwardFromFixedEnd()
        {
            _statusText += "\n[1/5] 前移脱离固定端...";
            float moveTime = forwardClearance / cutSpeed;
            yield return StartCoroutine(MoveToolBy(Vector3.forward * forwardClearance, moveTime));
            // 稳定几帧
            yield return StartCoroutine(WaitFrames(settleFrames));
        }

        // ══════════════════════════════════════════════════════
        // 测试 1：直线切割
        // 轨迹：+Z clearance → +X 直线切割
        // ══════════════════════════════════════════════════════
        IEnumerator TestLinearCut()
        {
            // Step 1: 前移脱离固定端
            yield return StartCoroutine(MoveForwardFromFixedEnd());

            // Step 2: 向 -Y (向下) 进入肝脏
            _statusText = "[直线切割] 进刀...";
            float entryTime = 0.15f / cutSpeed;
            yield return StartCoroutine(MoveToolBy(Vector3.down * 0.15f, entryTime));
            yield return StartCoroutine(WaitFrames(10));

            // Step 3: 向 +X 直线切割
            _statusText = "[直线切割] 切割中...";
            float cutTime = segmentLength / cutSpeed;
            yield return StartCoroutine(MoveToolBy(Vector3.right * segmentLength, cutTime));

            // Step 4: 出刀
            _statusText = "[直线切割] 出刀...";
            yield return StartCoroutine(WaitFrames(30));
        }

        // ══════════════════════════════════════════════════════
        // 测试 2：直角转弯切割
        // 轨迹：+Z clearance → 进刀 → +X → 停顿 → +Z → 出刀
        // 这是最容易暴露 bridge vertex 问题的测试
        // ══════════════════════════════════════════════════════
        IEnumerator TestRightAngleCut()
        {
            yield return StartCoroutine(MoveForwardFromFixedEnd());

            // 进刀
            _statusText = "[直角转弯] 进刀...";
            yield return StartCoroutine(MoveToolBy(Vector3.down * 0.15f, 0.15f / cutSpeed));
            yield return StartCoroutine(WaitFrames(10));

            // 第一段：+X
            _statusText = "[直角转弯] 第一段 (+X)...";
            float segTime = segmentLength / cutSpeed;
            yield return StartCoroutine(MoveToolBy(Vector3.right * segmentLength, segTime));

            // 转弯停顿
            yield return new WaitForSeconds(pauseBetweenSegments);

            // 第二段：+Z（直角转弯）
            _statusText = "[直角转弯] 转弯第二段 (+Z)...";
            yield return StartCoroutine(MoveToolBy(Vector3.forward * segmentLength, segTime));

            // 出刀稳定
            yield return StartCoroutine(WaitFrames(30));
        }

        // ══════════════════════════════════════════════════════
        // 测试 3：S 形切割
        // 轨迹：+Z clearance → 进刀 → +X弯+Z → -X弯+Z → 出刀
        // ══════════════════════════════════════════════════════
        IEnumerator TestSCurveCut()
        {
            yield return StartCoroutine(MoveForwardFromFixedEnd());

            // 进刀
            _statusText = "[S形] 进刀...";
            yield return StartCoroutine(MoveToolBy(Vector3.down * 0.15f, 0.15f / cutSpeed));
            yield return StartCoroutine(WaitFrames(10));

            float halfSeg = segmentLength * 0.5f;
            float quarterSeg = segmentLength * 0.25f;
            float halfTime = halfSeg / cutSpeed;
            float quarterTime = quarterSeg / cutSpeed;

            // S 形第一弯：+X 同时 +Z
            _statusText = "[S形] 第一弯...";
            yield return StartCoroutine(MoveToolSmoothCurve(
                new Vector3[] {
                    Vector3.zero,
                    new Vector3(halfSeg, 0, quarterSeg),
                    new Vector3(halfSeg, 0, halfSeg)
                },
                halfTime * 2f
            ));

            yield return new WaitForSeconds(pauseBetweenSegments * 0.5f);

            // S 形第二弯：-X 同时 +Z
            _statusText = "[S形] 第二弯...";
            yield return StartCoroutine(MoveToolSmoothCurve(
                new Vector3[] {
                    Vector3.zero,
                    new Vector3(-halfSeg, 0, quarterSeg),
                    new Vector3(-halfSeg, 0, halfSeg)
                },
                halfTime * 2f
            ));

            yield return StartCoroutine(WaitFrames(30));
        }

        /// <summary>沿 Catmull-Rom 曲线平滑移动</summary>
        IEnumerator MoveToolSmoothCurve(Vector3[] relativeWaypoints, float totalTime)
        {
            if (relativeWaypoints == null || relativeWaypoints.Length < 2) yield break;
            Vector3 startPos = GetToolPos();
            float elapsed = 0f;

            // Build cumulative distances for time parameterization
            float[] cumDist = new float[relativeWaypoints.Length];
            cumDist[0] = 0f;
            for (int i = 1; i < relativeWaypoints.Length; i++)
                cumDist[i] = cumDist[i - 1] + (relativeWaypoints[i] - relativeWaypoints[i - 1]).magnitude;
            float totalDist = cumDist[cumDist.Length - 1];
            if (totalDist < 1e-6f) yield break;

            while (elapsed < totalTime)
            {
                elapsed += Time.deltaTime;
                float u = Mathf.Clamp01(elapsed / totalTime) * totalDist;

                // Find segment
                int seg = 0;
                for (int i = 1; i < cumDist.Length; i++)
                {
                    if (cumDist[i] >= u) { seg = i - 1; break; }
                    seg = i - 1;
                }
                float segFrac = 0f;
                float segLen = cumDist[seg + 1] - cumDist[seg];
                if (segLen > 1e-8f) segFrac = (u - cumDist[seg]) / segLen;

                Vector3 localPos = Vector3.Lerp(relativeWaypoints[seg], relativeWaypoints[seg + 1], segFrac);
                SetToolPos(startPos + localPos);
                yield return null;
            }
            // Ensure final position
            SetToolPos(startPos + relativeWaypoints[relativeWaypoints.Length - 1]);
        }

        // ══════════════════════════════════════════════════════
        // 测试 4：复杂综合切割
        // 轨迹：直线 → 直角 → S 弯 → 回转
        // ══════════════════════════════════════════════════════
        IEnumerator TestComplexCut()
        {
            yield return StartCoroutine(MoveForwardFromFixedEnd());

            // 进刀
            _statusText = "[复杂] 进刀...";
            yield return StartCoroutine(MoveToolBy(Vector3.down * 0.12f, 0.12f / cutSpeed));
            yield return StartCoroutine(WaitFrames(10));

            float t1 = segmentLength * 0.4f / cutSpeed;
            float t2 = segmentLength * 0.3f / cutSpeed;

            // 段 1：直线 +X
            _statusText = "[复杂] 直线段...";
            yield return StartCoroutine(MoveToolBy(Vector3.right * segmentLength * 0.4f, t1));
            yield return new WaitForSeconds(pauseBetweenSegments);

            // 段 2：直角 +Z
            _statusText = "[复杂] 直角段...";
            yield return StartCoroutine(MoveToolBy(Vector3.forward * segmentLength * 0.3f, t2));
            yield return new WaitForSeconds(pauseBetweenSegments);

            // 段 3：S 弯
            _statusText = "[复杂] S弯段...";
            float hs = segmentLength * 0.3f;
            yield return StartCoroutine(MoveToolSmoothCurve(
                new Vector3[] {
                    Vector3.zero,
                    new Vector3(-hs * 0.5f, 0, hs * 0.5f),
                    new Vector3(0, 0, hs)
                },
                hs / cutSpeed * 1.5f
            ));
            yield return new WaitForSeconds(pauseBetweenSegments);

            // 段 4：-X 收刀
            _statusText = "[复杂] 收刀段...";
            yield return StartCoroutine(MoveToolBy(Vector3.left * segmentLength * 0.3f, t2));

            yield return StartCoroutine(WaitFrames(30));
            _statusText = "[复杂] 完成 ✓";
        }

        void OnDisable()
        {
            if (_currentCoroutine != null)
                StopCoroutine(_currentCoroutine);
            _isRunning = false;
        }
    }
}
