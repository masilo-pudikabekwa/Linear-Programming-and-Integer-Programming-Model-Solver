using System;
using Linear_Programming_and_Integer_Programming_Model_Solver.Core;

namespace Linear_Programming_and_Integer_Programming_Model_Solver.Analysis;

/// <summary>
/// Compares a primal optimal result against its dual's optimal result.
/// Weak duality (primal objective bounded by dual objective) always holds for
/// feasible solutions; strong duality is the special case where, AT the optimum
/// of both, the two objective values are exactly equal.
/// </summary>
public class DualityVerifier
{
    private const double Tolerance = 1e-6;

    public (bool isStrong, string explanation) Verify(SolutionResult primalResult, SolutionResult dualResult)
    {
        if (primalResult == null) throw new ArgumentNullException(nameof(primalResult));
        if (dualResult == null) throw new ArgumentNullException(nameof(dualResult));

        if (!primalResult.IsFeasible || !dualResult.IsBounded)
        {
            return (false,
                "Primal is infeasible and/or dual is unbounded - consistent with LP duality theory " +
                "(primal infeasible implies dual is unbounded or infeasible, and vice versa). " +
                "No finite objective values to compare.");
        }

        if (!dualResult.IsFeasible || !primalResult.IsBounded)
        {
            return (false,
                "Dual is infeasible and/or primal is unbounded - consistent with LP duality theory. " +
                "No finite objective values to compare.");
        }

        if (!primalResult.IsOptimal || !dualResult.IsOptimal)
        {
            return (false,
                "One or both problems did not reach a confirmed optimal solution; duality cannot be verified.");
        }

        double primalObj = primalResult.ObjectiveValue;
        double dualObj = dualResult.ObjectiveValue;
        double gap = Math.Abs(primalObj - dualObj);

        if (gap < Tolerance)
        {
            return (true,
                $"Strong duality holds: primal optimal objective ({primalObj:F4}) equals dual optimal " +
                $"objective ({dualObj:F4}). Both solutions are confirmed globally optimal.");
        }

        return (false,
            $"Strong duality does NOT appear to hold: primal objective = {primalObj:F4}, dual objective = " +
            $"{dualObj:F4} (gap = {gap:F4}). A non-zero gap here, at claimed optimality, usually points to a " +
            $"bug in the duality conversion, an ApplyChange edit, or the solve itself - re-check sign " +
            $"conventions on whichever constraint/coefficient was last modified.");
    }
}
