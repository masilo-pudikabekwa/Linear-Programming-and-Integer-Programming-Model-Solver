using System;
using System.Collections.Generic;
using System.Linq;
using Linear_Programming_and_Integer_Programming_Model_Solver.Core;

namespace Linear_Programming_and_Integer_Programming_Model_Solver.Analysis;


/// Converts a primal <see cref="LPModel"/> into its dual <see cref="LPModel"/>,
/// handling all three constraint relations (&lt;=, &gt;=, =) and both objective
/// directions — not just the textbook "max, all &lt;=" case.
///
/// Sign-restriction table used (standard LP duality theory, primal assumed x &gt;= 0):
///   Primal MAX                | Primal MIN
///   &lt;= constraint -&gt; y &gt;= 0   | &gt;= constraint -&gt; y &gt;= 0
///   &gt;= constraint -&gt; y &lt;= 0   | &lt;= constraint -&gt; y &lt;= 0
///   =  constraint -&gt; y free   | =  constraint -&gt; y free
/// Dual constraints: A^T y &gt;= c (dual is Min, when primal is Max)
///                    A^T y &lt;= c (dual is Max, when primal is Min)
///
/// PrimalSimplex only ever works with x &gt;= 0 columns (see BuildInitialTableau),
/// so a dual variable that should be &lt;= 0 or free is represented via the usual
/// substitution, keeping the returned LPModel solvable by the same PrimalSimplex
/// everyone else uses:
///   y &lt;= 0  =&gt;  y = -y',              y' &gt;= 0   (one column, negated)
///   y free  =&gt;  y = y+ - y-,           y+, y- &gt;= 0  (two columns)

public class DualityConverter
{
    private enum DualVarKind { NonNegative, NonPositive, Free }

    /// Required signature per spec.
    public LPModel ToDual(LPModel primal) => ToDual(primal, out _);


    /// Same conversion, additionally returning a human-readable label per dual
    /// variable column — useful for display, and for mapping a split free-variable
    /// pair (y+, y-) back to a single economic shadow price (y = y+ - y-).

    public LPModel ToDual(LPModel primal, out List<string> dualVariableLabels)
    {
        if (primal == null) throw new ArgumentNullException(nameof(primal));

        bool primalIsMax = primal.Objective == ObjectiveType.Max;
        var dualObjective = primalIsMax ? ObjectiveType.Min : ObjectiveType.Max;

        int m = primal.Constraints.Count;
        var kinds = new DualVarKind[m];
        for (int i = 0; i < m; i++)
        {
            var rel = primal.Constraints[i].Relation;
            kinds[i] = (primalIsMax, rel) switch
            {
                (true, RelationType.LessThanOrEqualTo) => DualVarKind.NonNegative,
                (true, RelationType.GreaterThanOrEqualTo) => DualVarKind.NonPositive,
                (true, RelationType.EqualTo) => DualVarKind.Free,
                (false, RelationType.GreaterThanOrEqualTo) => DualVarKind.NonNegative,
                (false, RelationType.LessThanOrEqualTo) => DualVarKind.NonPositive,
                (false, RelationType.EqualTo) => DualVarKind.Free,
                _ => DualVarKind.Free
            };
        }

        // Build dual columns. Each primal constraint contributes 1 dual column
        // (NonNegative / NonPositive) or 2 (Free, split into y+/y-), and records
        // the sign multiplier(s) needed everywhere that constraint's row feeds
        // into the dual (both the dual's objective coefficient and every entry
        // of every dual constraint that touches this y).
        var objCoeffs = new List<double>();
        var labels = new List<string>();
        var blueprint = new List<(double sign, int constraintIndex)>[m];

        for (int i = 0; i < m; i++)
        {
            double b = primal.Constraints[i].RHS;
            switch (kinds[i])
            {
                case DualVarKind.NonNegative:
                    blueprint[i] = new List<(double, int)> { (1.0, i) };
                    objCoeffs.Add(b);
                    labels.Add($"y{i + 1}");
                    break;

                case DualVarKind.NonPositive:
                    blueprint[i] = new List<(double, int)> { (-1.0, i) };
                    objCoeffs.Add(-b);
                    labels.Add($"y{i + 1}' (y{i + 1} = -y{i + 1}')");
                    break;

                case DualVarKind.Free:
                    blueprint[i] = new List<(double, int)> { (1.0, i), (-1.0, i) };
                    objCoeffs.Add(b);
                    objCoeffs.Add(-b);
                    labels.Add($"y{i + 1}+");
                    labels.Add($"y{i + 1}- (y{i + 1} = y{i + 1}+ - y{i + 1}-)");
                    break;
            }
        }

        // Dual constraints: one per primal variable j, RHS = that variable's
        // objective coefficient c_j.
        var dualRelation = primalIsMax ? RelationType.GreaterThanOrEqualTo : RelationType.LessThanOrEqualTo;
        var dualConstraints = new List<Constraint>();
        for (int j = 0; j < primal.VariableCount; j++)
        {
            var coeffs = new List<double>();
            for (int i = 0; i < m; i++)
            {
                double aij = primal.Constraints[i].Coefficients[j];
                foreach (var (sign, _) in blueprint[i])
                    coeffs.Add(sign * aij);
            }
            dualConstraints.Add(new Constraint(coeffs.ToArray(), dualRelation, primal.ObjectiveCoefficients[j]));
        }

        dualVariableLabels = labels;

        // Every column of the returned model is already a substituted, non-negative
        // variable (see class summary), so Restrictions is uniformly Positive here —
        // that's what makes this model directly solvable by PrimalSimplex as-is.
        return new LPModel
        {
            Objective = dualObjective,
            ObjectiveCoefficients = objCoeffs.ToArray(),
            Constraints = dualConstraints,
            Restrictions = Enumerable.Repeat(SignRestriction.Positive, objCoeffs.Count).ToArray()
        };
    }
}
