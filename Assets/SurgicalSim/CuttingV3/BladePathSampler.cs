using System.Collections.Generic;
using UnityEngine;

namespace SurgicalSim.CuttingV3
{
    /// <summary>
    /// Step 2 of CuttingV3.1: smooths the blade trajectory before it reaches
    /// the cutter. The raw per-frame (BladeA, BladeB) samples carry a few
    /// millimetres of high-frequency jitter from the input device and the
    /// physics integration; feeding them directly to a plane-sweep cutter
    /// produces a different cutting plane every frame and visible "scale
    /// flake" bumps on the cut surface.
    ///
    /// We keep a 4-sample ring buffer and emit Catmull-Rom interpolated
    /// ribbon segments between consecutive samples. The Catmull-Rom basis
    /// guarantees C1 continuity of the tangent direction across segment
    /// boundaries, so cutting planes computed from successive emitted
    /// ribbon quads rotate smoothly instead of stepping.
    ///
    /// The sampler has a one-sample latency (it waits for the fourth sample
    /// before it can compute a tangent for the P1->P2 segment). At a 60 Hz
    /// update rate this is &lt;17 ms which is well below the surgeon's
    /// perception threshold for haptic interaction.
    /// </summary>
    internal sealed class BladePathSampler
    {
        public struct RibbonQuad
        {
            public Vector3 A0, B0, A1, B1;
        }

        struct Sample { public Vector3 A; public Vector3 B; }

        readonly Sample[] _ring = new Sample[4];
        readonly Queue<RibbonQuad> _emitted = new Queue<RibbonQuad>(16);

        readonly int _substeps;
        readonly float _minMoveSqr;
        int _count;

        public BladePathSampler(int substeps = 3, float minMove = 0.0015f)
        {
            _substeps = Mathf.Max(1, substeps);
            _minMoveSqr = minMove * minMove;
        }

        public bool HasPending => _emitted.Count > 0;
        public int PendingCount => _emitted.Count;

        public void Reset()
        {
            _count = 0;
            _emitted.Clear();
        }

        /// <summary>
        /// Push the latest blade endpoint pair. Adds at most one new
        /// smoothed ribbon segment to the internal queue. Returns true if
        /// the sample was accepted (i.e. the blade actually moved enough
        /// to be a new sample); samples that are too close to the previous
        /// one are dropped to keep the spline shape stable at low speeds.
        /// </summary>
        public bool Push(Vector3 bladeA, Vector3 bladeB)
        {
            if (_count > 0)
            {
                var prev = _ring[(_count - 1) & 3];
                float dA = (bladeA - prev.A).sqrMagnitude;
                float dB = (bladeB - prev.B).sqrMagnitude;
                if (dA < _minMoveSqr && dB < _minMoveSqr) return false;
            }

            _ring[_count & 3] = new Sample { A = bladeA, B = bladeB };
            _count++;
            EmitNew();
            return true;
        }

        public bool TryDequeue(out RibbonQuad quad)
        {
            if (_emitted.Count == 0) { quad = default; return false; }
            quad = _emitted.Dequeue();
            return true;
        }

        /// <summary>
        /// Flush the last linear segment when the blade leaves the mesh,
        /// so the trailing portion of the stroke still gets cut even
        /// though we never received a "post" sample for Catmull-Rom.
        /// </summary>
        public void Flush()
        {
            if (_count < 2) return;
            var p1 = _ring[(_count - 2) & 3];
            var p2 = _ring[(_count - 1) & 3];
            _emitted.Enqueue(new RibbonQuad { A0 = p1.A, B0 = p1.B, A1 = p2.A, B1 = p2.B });
        }

        void EmitNew()
        {
            if (_count < 2) return;

            if (_count == 2)
            {
                // Bootstrap: only a single segment is known; emit it as a
                // straight ribbon while we wait for more context.
                var p0 = _ring[0];
                var p1 = _ring[1];
                _emitted.Enqueue(new RibbonQuad { A0 = p0.A, B0 = p0.B, A1 = p1.A, B1 = p1.B });
                return;
            }

            if (_count == 3)
            {
                // Three samples: emit the [P1, P2] segment with a mirrored
                // "ghost" P3 = 2*P2 - P1 so we can still feed Catmull-Rom.
                var p0 = _ring[0];
                var p1 = _ring[1];
                var p2 = _ring[2];
                Vector3 p3A = p2.A + (p2.A - p1.A);
                Vector3 p3B = p2.B + (p2.B - p1.B);
                EmitCatmullRom(p0.A, p1.A, p2.A, p3A, p0.B, p1.B, p2.B, p3B);
                return;
            }

            // Steady state: the four most recent samples form a Catmull-Rom
            // window; emit the middle [P1, P2] segment.
            var q0 = _ring[(_count - 4) & 3];
            var q1 = _ring[(_count - 3) & 3];
            var q2 = _ring[(_count - 2) & 3];
            var q3 = _ring[(_count - 1) & 3];
            EmitCatmullRom(q0.A, q1.A, q2.A, q3.A, q0.B, q1.B, q2.B, q3.B);
        }

        void EmitCatmullRom(
            Vector3 a0, Vector3 a1, Vector3 a2, Vector3 a3,
            Vector3 b0, Vector3 b1, Vector3 b2, Vector3 b3)
        {
            Vector3 prevA = a1, prevB = b1;
            for (int s = 1; s <= _substeps; s++)
            {
                float t = (float)s / _substeps;
                Vector3 curA = CR(a0, a1, a2, a3, t);
                Vector3 curB = CR(b0, b1, b2, b3, t);
                _emitted.Enqueue(new RibbonQuad { A0 = prevA, B0 = prevB, A1 = curA, B1 = curB });
                prevA = curA;
                prevB = curB;
            }
        }

        static Vector3 CR(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3, float t)
        {
            float t2 = t * t;
            float t3 = t2 * t;
            return 0.5f * (
                (2f * p1) +
                (-p0 + p2) * t +
                (2f * p0 - 5f * p1 + 4f * p2 - p3) * t2 +
                (-p0 + 3f * p1 - 3f * p2 + p3) * t3);
        }
    }
}
