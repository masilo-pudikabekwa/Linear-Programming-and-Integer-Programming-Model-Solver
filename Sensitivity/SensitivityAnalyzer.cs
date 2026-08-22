using System;
using System.Linq;
using Linear_Programming_and_Integer_Programming_Model_Solver.Algorithms;
using Linear_Programming_and_Integer_Programming_Model_Solver.Core;

namespace Linear_Programming_and_Integer_Programming_Model_Solver.Analysis;

public class SensitivityAnalyzer
{
    private readonly Tableau _tableau;
    private readonly LPModel _originalModel;
    private const double Eps = 1e-9;

    public SensitivityAnalyzer(Tableau optimalTableau, LPModel originalModel)
    {
        _tableau = optimalTableau ?? throw new ArgumentNullException(nameof(optimalTableau));
        _originalModel = originalModel ?? throw new ArgumentNullException(nameof(originalModel));

        if (_tableau.BasisInverse.GetLength(0) == 0 || _tableau.CB.Length == 0)
            throw new InvalidOperationException(
                "Tableau.BasisInverse/CB are not populated. Sensitivity analysis requires the FINAL " +
                "tableau of an optimal solve (see PrimalSimplex.Solve, which populates both).");
    }

    // ------------------------------------------------------------------
    // Small internal helpers
    // ------------------------------------------------------------------

    private int NumRows => _tableau.BasicVariableIndices.Length; // m
    private int TotalCols => _tableau.Matrix.GetLength(1) - 1;   // excludes RHS column
    private int RhsCol => _tableau.Matrix.GetLength(1) - 1;

    /// +1 for Max, -1 for Min. PrimalSimplex negates a Min model's
    /// objective internally, so every internal cost/objective value is Sign
    /// times the value in the caller's original units, and Sign is self-inverse
    /// (multiplying by it again converts back).
    private double Sign => _originalModel.Objective == ObjectiveType.Max ? 1.0 : -1.0;

    private double ToInternal(double externalValue) => Sign * externalValue;

    /// Converts an internal-form (internalLower, internalUpper) interval to
    /// external units. For Max models this is a no-op; for Min models multiplying by
    /// -1 flips the interval's direction, so bounds are swapped accordingly.
    private (double lower, double upper) ToExternalRange(double internalLower, double internalUpper)
    {
        if (Sign > 0) return (internalLower, internalUpper);
        return (-internalUpper, -internalLower);
    }

    /// <summary>Row index in the basis for this column, or null if non-basic.</summary>
    private int? RowOf(int columnIndex)
    {
        var basis = _tableau.BasicVariableIndices;
        for (int r = 0; r < basis.Length; r++)
            if (basis[r] == columnIndex) return r;
        return null;
    }

    private void RequireNonBasic(int varIndex)
    {
        if (RowOf(varIndex) != null)
            throw new InvalidOperationException($"Variable {varIndex} is currently basic; use the basic-variable overload instead.");
    }

    private void RequireBasic(int varIndex)
    {
        if (RowOf(varIndex) == null)
            throw new InvalidOperationException($"Variable {varIndex} is currently non-basic; use the non-basic-variable overload instead.");
    }

    /// Current column j, rows 1..m only (= y_j = B^-1 * A_j at the current basis).
    private double[] Column(int col)
    {
        var y = new double[NumRows];
        for (int r = 0; r < NumRows; r++) y[r] = _tableau.Matrix[r + 1, col];
        return y;
    }

    /// Current basic solution x_B, row-aligned with BasicVariableIndices.
    private double[] CurrentXB()
    {
        var xB = new double[NumRows];
        for (int r = 0; r < NumRows; r++) xB[r] = _tableau.Matrix[r + 1, RhsCol];
        return xB;
    }

    private static double Dot(double[] a, double[] b)
    {
        double sum = 0;
        for (int i = 0; i < a.Length; i++) sum += a[i] * b[i];
        return sum;
    }

    /// Internal z_j = CB . y_j for column j, using the CURRENT tableau column.
    private double ZValue(int col) => Dot(_tableau.CB, Column(col));

    /// Internal z for an arbitrary (possibly hypothetical) column vector.
    private double ZValueForVector(double[] column)
    {
        var y = MultiplyBasisInverse(column);
        return Dot(_tableau.CB, y);
    }

    private double[] MultiplyBasisInverse(double[] vector)
    {
        int m = NumRows;
        var result = new double[m];
        for (int r = 0; r < m; r++)
        {
            double sum = 0;
            for (int k = 0; k < m; k++) sum += _tableau.BasisInverse[r, k] * vector[k];
            result[r] = sum;
        }
        return result;
    }

    /// Internal reduced cost of column j, read straight off row 0 — this is
    /// exactly z_j - c_j by construction of the tableau's objective row.
    private double ReducedCost(int col) => _tableau.Matrix[0, col];

    /// Internal c_j for column j, backed out as z_j - reducedCost_j. Works
    /// uniformly for structural, slack/surplus, and artificial columns without
    /// needing to special-case column type.
    private double CurrentCInternal(int col) => ZValue(col) - ReducedCost(col);

    /// Maps x_B back onto a full-length structural-variable vector (slack/
    /// surplus/artificial columns are dropped; non-basic structural vars are 0).
    private double[] StructuralValuesFromXB(double[] xB)
    {
        var values = new double[_originalModel.VariableCount];
        for (int r = 0; r < xB.Length; r++)
        {
            int col = _tableau.BasicVariableIndices[r];
            if (col < _originalModel.VariableCount) values[col] = xB[r];
        }
        return values;
    }

    private double ExternalObjective(double[] xB) => Sign * Dot(_tableau.CB, xB);

    private static string Fmt(double v) =>
        double.IsPositiveInfinity(v) ? "+inf" : double.IsNegativeInfinity(v) ? "-inf" :
        double.IsNaN(v) ? "n/a" : v.ToString("F4");

    // ------------------------------------------------------------------
    // 1. Non-basic variable objective-coefficient ranging
    // ------------------------------------------------------------------

    /// Range of c_j for a currently NON-basic variable such that the current basis
    /// stays optimal. Changing c_j only affects that column's own reduced cost (it
    /// doesn't touch z_j, which depends on CB of the basic columns and A_j, neither
    /// of which involve c_j). Condition: reduced cost = z_j - c_j &gt;= 0 => c_j &lt;= z_j.
    /// c_j can decrease without limit (only makes the variable less attractive).
    public (double lower, double upper) RangeNonBasicVariable(int varIndex)
    {
        RequireNonBasic(varIndex);
        double z = ZValue(varIndex);
        return ToExternalRange(double.NegativeInfinity, z);
    }

    public SolutionResult ApplyChangeNonBasicVariable(int varIndex, double newValue)
    {
        var (lower, upper) = RangeNonBasicVariable(varIndex);
        bool staysOptimal = newValue >= lower - Eps && newValue <= upper + Eps;

        if (staysOptimal)
        {
            var xB = CurrentXB();
            return new SolutionResult
            {
                IsFeasible = true,
                IsBounded = true,
                IsOptimal = true,
                ObjectiveValue = ExternalObjective(xB),
                VariableValues = StructuralValuesFromXB(xB),
                FinalTableau = _tableau,
                ErrorMessage = $"c_{varIndex} = {newValue} is within the optimal range " +
                                $"[{Fmt(lower)}, {Fmt(upper)}]; basis unchanged (variable stays non-basic at 0)."
            };
        }

        return ReSolveWithModifiedObjectiveCoefficient(varIndex, newValue,
            $"c_{varIndex} = {newValue} falls outside the optimal range [{Fmt(lower)}, {Fmt(upper)}]; re-solved.");
    }

    // ------------------------------------------------------------------
    // 2. Basic variable objective-coefficient ranging
    // ------------------------------------------------------------------


    /// Range of c_j for a currently BASIC variable such that every non-basic
    /// reduced cost stays optimal. Changing c_{B_r} by delta shifts every
    /// non-basic reduced cost by delta * alpha_{r,k}, where alpha_{r,k} is row r of
    /// y_k. Need (z_k - c_k) + delta * alpha_{r,k} &gt;= 0 for every non-basic k.

    public (double lower, double upper) RangeBasicVariable(int varIndex)
    {
        RequireBasic(varIndex);
        int r = RowOf(varIndex)!.Value;

        double deltaLower = double.NegativeInfinity;
        double deltaUpper = double.PositiveInfinity;

        for (int k = 0; k < TotalCols; k++)
        {
            if (RowOf(k) != null) continue; // only non-basic columns constrain this
            double alpha = _tableau.Matrix[r + 1, k];
            if (Math.Abs(alpha) < Eps) continue;

            double reducedCost = ReducedCost(k); // >= 0 currently (optimal tableau)
            double bound = -reducedCost / alpha;

            if (alpha > 0) deltaLower = Math.Max(deltaLower, bound);
            else deltaUpper = Math.Min(deltaUpper, bound);
        }

        double cInternal = CurrentCInternal(varIndex);
        double internalLower = double.IsNegativeInfinity(deltaLower) ? double.NegativeInfinity : cInternal + deltaLower;
        double internalUpper = double.IsPositiveInfinity(deltaUpper) ? double.PositiveInfinity : cInternal + deltaUpper;
        return ToExternalRange(internalLower, internalUpper);
    }

    public SolutionResult ApplyChangeBasicVariable(int varIndex, double newValue)
    {
        var (lower, upper) = RangeBasicVariable(varIndex);
        bool staysOptimal = newValue >= lower - Eps && newValue <= upper + Eps;

        if (staysOptimal)
        {
            int r = RowOf(varIndex)!.Value;
            var xB = CurrentXB();
            double cInternal = CurrentCInternal(varIndex);
            double deltaInternal = ToInternal(newValue) - cInternal;
            double newExternalObjective = ExternalObjective(xB) + Sign * deltaInternal * xB[r];

            return new SolutionResult
            {
                IsFeasible = true,
                IsBounded = true,
                IsOptimal = true,
                ObjectiveValue = newExternalObjective,
                VariableValues = StructuralValuesFromXB(xB),
                FinalTableau = _tableau,
                ErrorMessage = $"c_{varIndex} = {newValue} is within the optimal range " +
                                $"[{Fmt(lower)}, {Fmt(upper)}]; basis unchanged, objective updated directly."
            };
        }

        return ReSolveWithModifiedObjectiveCoefficient(varIndex, newValue,
            $"c_{varIndex} = {newValue} falls outside the optimal range [{Fmt(lower)}, {Fmt(upper)}]; re-solved.");
    }

    // ------------------------------------------------------------------
    // 3. RHS ranging
    // ------------------------------------------------------------------

    /// Range of b_i such that the current basis stays FEASIBLE (reduced costs are
    /// untouched by RHS changes - only feasibility of x_B can break).
    /// x_B(delta) = x_B + delta * beta_i, where beta_i is column i of B^-1.
    /// Need x_B(delta) &gt;= 0 for every basic row. RHS values are never sign-flipped
    /// by the Min/Max convention (only the objective row is), so no external/internal
    /// conversion is needed here.

    public (double lower, double upper) RangeRHS(int constraintIndex)
    {
        var xB = CurrentXB();
        int m = NumRows;
        var beta = new double[m];
        for (int r = 0; r < m; r++) beta[r] = _tableau.BasisInverse[r, constraintIndex];

        double deltaLower = double.NegativeInfinity;
        double deltaUpper = double.PositiveInfinity;

        for (int r = 0; r < m; r++)
        {
            if (Math.Abs(beta[r]) < Eps) continue;
            double bound = -xB[r] / beta[r];
            if (beta[r] > 0) deltaLower = Math.Max(deltaLower, bound);
            else deltaUpper = Math.Min(deltaUpper, bound);
        }

        double current = _originalModel.Constraints[constraintIndex].RHS;
        double lower = double.IsNegativeInfinity(deltaLower) ? double.NegativeInfinity : current + deltaLower;
        double upper = double.IsPositiveInfinity(deltaUpper) ? double.PositiveInfinity : current + deltaUpper;
        return (lower, upper);
    }

    public SolutionResult ApplyChangeRHS(int constraintIndex, double newRHS)
    {
        var (lower, upper) = RangeRHS(constraintIndex);
        bool staysFeasible = newRHS >= lower - Eps && newRHS <= upper + Eps;

        if (staysFeasible)
        {
            var bFull = _originalModel.Constraints.Select(c => c.RHS).ToArray();
            bFull[constraintIndex] = newRHS;
            var newXB = MultiplyBasisInverse(bFull);

            return new SolutionResult
            {
                IsFeasible = true,
                IsBounded = true,
                IsOptimal = true,
                ObjectiveValue = ExternalObjective(newXB),
                VariableValues = StructuralValuesFromXB(newXB),
                FinalTableau = _tableau,
                ErrorMessage = $"b_{constraintIndex} = {newRHS} is within the feasible range " +
                                $"[{Fmt(lower)}, {Fmt(upper)}]; basis unchanged, values updated via B^-1 * b."
            };
        }

        var modifiedModel = _originalModel.Clone();
        modifiedModel.Constraints[constraintIndex].RHS = newRHS;
        var result = new PrimalSimplex().Solve(modifiedModel);
        result.ErrorMessage = $"b_{constraintIndex} = {newRHS} falls outside the feasible range " +
                                $"[{Fmt(lower)}, {Fmt(upper)}]; re-solved from scratch. " + result.ErrorMessage;
        return result;
    }

    // ------------------------------------------------------------------
    // 4. Non-basic column ranging (whole constraint column, uniformly scaled)
    // ------------------------------------------------------------------


    /// Ranges a uniform scale factor t applied to the ENTIRE constraint column of a
    /// non-basic variable j (A_j(t) = t * A_j_current) - the standard well-posed way
    /// to give a single interval for a change that potentially touches every row of
    /// the column at once. y_j(t) = t * y_j(1), so reduced cost(t) = t*z_j - c_j is
    /// linear in t.

    public (double lower, double upper) RangeNonBasicColumn(int varIndex)
    {
        RequireNonBasic(varIndex);
        double z = ZValue(varIndex);        // z_j at t = 1 (internal)
        double cInternal = CurrentCInternal(varIndex);

        if (Math.Abs(z) < Eps)
        {
            // Reduced cost is constant in t: either always optimal or never.
            return cInternal <= Eps
                ? (double.NegativeInfinity, double.PositiveInfinity)
                : (double.NaN, double.NaN);
        }

        // t is a dimensionless scale factor on constraint coefficients, not an
        // objective coefficient, so it does NOT go through the Min/Max sign flip.
        return z > 0 ? (cInternal / z, double.PositiveInfinity)
                      : (double.NegativeInfinity, cInternal / z);
    }

    public SolutionResult ApplyChangeNonBasicColumn(int varIndex, double[] newColumn)
    {
        RequireNonBasic(varIndex);

        double z = ZValueForVector(newColumn);
        double cInternal = CurrentCInternal(varIndex);
        double reducedCost = z - cInternal;

        if (reducedCost >= -Eps)
        {
            var xB = CurrentXB();
            return new SolutionResult
            {
                IsFeasible = true,
                IsBounded = true,
                IsOptimal = true,
                ObjectiveValue = ExternalObjective(xB),
                VariableValues = StructuralValuesFromXB(xB),
                FinalTableau = _tableau,
                ErrorMessage = $"New column for variable {varIndex} still has reduced cost " +
                                $"{Fmt(reducedCost)} >= 0; basis remains optimal (variable stays non-basic at 0)."
            };
        }

        if (varIndex >= _originalModel.VariableCount)
            throw new InvalidOperationException(
                "Cannot rebuild the LPModel for a slack/surplus column change - only structural " +
                "decision-variable columns can be re-solved from the original model.");

        var modifiedModel = _originalModel.Clone();
        for (int i = 0; i < modifiedModel.Constraints.Count; i++)
            modifiedModel.Constraints[i].Coefficients[varIndex] = newColumn[i];

        var result = new PrimalSimplex().Solve(modifiedModel);
        result.ErrorMessage = $"New column makes variable {varIndex} attractive (reduced cost " +
                                $"{Fmt(reducedCost)} < 0); re-solved. " + result.ErrorMessage;
        return result;
    }

    // ------------------------------------------------------------------
    // 5. Adding a new activity (variable)
    // ------------------------------------------------------------------

    public SolutionResult AddNewActivity(double objCoeff, double[] constraintColumn)
    {
        double z = ZValueForVector(constraintColumn);
        double reducedCost = z - ToInternal(objCoeff);

        if (reducedCost >= -Eps)
        {
            var xB = CurrentXB();
            var values = StructuralValuesFromXB(xB).Append(0.0).ToArray(); // new var enters at 0
            return new SolutionResult
            {
                IsFeasible = true,
                IsBounded = true,
                IsOptimal = true,
                ObjectiveValue = ExternalObjective(xB),
                VariableValues = values,
                FinalTableau = _tableau,
                ErrorMessage = $"New activity has reduced cost {Fmt(reducedCost)} >= 0, so it would not " +
                                $"improve the optimum; not added to the basis."
            };
        }

        var modifiedModel = _originalModel.Clone();
        modifiedModel.ObjectiveCoefficients = modifiedModel.ObjectiveCoefficients.Append(objCoeff).ToArray();
        for (int i = 0; i < modifiedModel.Constraints.Count; i++)
            modifiedModel.Constraints[i].Coefficients =
                modifiedModel.Constraints[i].Coefficients.Append(constraintColumn[i]).ToArray();
        modifiedModel.Restrictions = modifiedModel.Restrictions.Append(SignRestriction.Positive).ToArray();

        var result = new PrimalSimplex().Solve(modifiedModel);
        result.ErrorMessage = $"New activity has reduced cost {Fmt(reducedCost)} < 0, so it is profitable to " +
                                $"introduce; re-solved. " + result.ErrorMessage;
        return result;
    }

    // ------------------------------------------------------------------
    // 6. Adding a new constraint
    // ------------------------------------------------------------------

    public SolutionResult AddNewConstraint(Constraint newConstraint)
    {
        var xB = CurrentXB();
        var currentValues = StructuralValuesFromXB(xB);

        double lhs = 0;
        for (int j = 0; j < Math.Min(currentValues.Length, newConstraint.Coefficients.Length); j++)
            lhs += newConstraint.Coefficients[j] * currentValues[j];

        bool alreadySatisfied = newConstraint.Relation switch
        {
            RelationType.LessThanOrEqualTo => lhs <= newConstraint.RHS + 1e-6,
            RelationType.GreaterThanOrEqualTo => lhs >= newConstraint.RHS - 1e-6,
            RelationType.EqualTo => Math.Abs(lhs - newConstraint.RHS) < 1e-6,
            _ => false
        };

        if (alreadySatisfied)
        {
            return new SolutionResult
            {
                IsFeasible = true,
                IsBounded = true,
                IsOptimal = true,
                ObjectiveValue = ExternalObjective(xB),
                VariableValues = currentValues,
                FinalTableau = _tableau,
                ErrorMessage = $"Current optimal solution already satisfies the new constraint " +
                                $"(LHS = {Fmt(lhs)}); solution unchanged."
            };
        }

        // The current solution violates the new constraint, which means the current
        // basis is no longer feasible. A dual-simplex re-optimization would avoid a
        // full restart, but no DualSimplex exists in the shared codebase, so this
        // re-solves the augmented model from scratch via PrimalSimplex - correct,
        // just not the fastest path.
        var modifiedModel = _originalModel.Clone();
        modifiedModel.Constraints.Add(newConstraint.Clone());
        var result = new PrimalSimplex().Solve(modifiedModel);
        result.ErrorMessage = $"Current optimal solution violates the new constraint (LHS = {Fmt(lhs)}); " +
                                $"re-solved from scratch to restore feasibility. " + result.ErrorMessage;
        return result;
    }

    // ------------------------------------------------------------------
    // 7. Shadow prices
    // ------------------------------------------------------------------

    /// Shadow prices y = CB . B^-1, one per original constraint, converted
    /// to the caller's external Min/Max units (see Sign).
    public double[] ShadowPrices()
    {
        int m = NumRows;
        var prices = new double[m];
        for (int i = 0; i < m; i++)
        {
            double sum = 0;
            for (int r = 0; r < m; r++) sum += _tableau.CB[r] * _tableau.BasisInverse[r, i];
            prices[i] = Sign * sum;
        }
        return prices;
    }

    // ------------------------------------------------------------------
    // Internal helpers
    // ------------------------------------------------------------------

    private SolutionResult ReSolveWithModifiedObjectiveCoefficient(int varIndex, double newValue, string reasonMessage)
    {
        if (varIndex >= _originalModel.VariableCount)
            throw new InvalidOperationException(
                "Cannot rebuild the LPModel for a slack/surplus objective coefficient - only structural " +
                "decision-variable coefficients can be re-solved from the original model.");

        var modifiedModel = _originalModel.Clone();
        modifiedModel.ObjectiveCoefficients[varIndex] = newValue;
        var result = new PrimalSimplex().Solve(modifiedModel);
        result.ErrorMessage = reasonMessage + " " + result.ErrorMessage;
        return result;
    }
}
