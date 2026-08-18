using Linear_Programming_and_Integer_Programming_Model_Solver.Core;

namespace Linear_Programming_and_Integer_Programming_Model_Solver.Algorithms;

// Standard tableau Simplex (Big-M method). Implements IAlgorithm so Member 2's
// Branch & Bound can call Solve() per tree node without knowing anything about
// pivoting internals.
public class PrimalSimplex : IAlgorithm
{
    private const int MaxIterations = 1000; // cycling / bug safety valve
    private const double Epsilon = 1e-9;
    private const double BigM = 1_000_000;

    public SolutionResult Solve(LPModel model)
    {
        var result = new SolutionResult();
        var tableau = SimplexUtilities.BuildInitialTableau(model, BigM);

        // Iteration 0 = the initial canonical tableau, before any pivot — the spec
        // wants the canonical form displayed, and this is exactly that.
        result.Iterations.Add(SimplexUtilities.Snapshot(tableau, 0));

        int iteration = 0;
        while (!SimplexUtilities.IsOptimal(tableau))
        {
            if (++iteration > MaxIterations)
            {
                result.IsFeasible = false;
                result.ErrorMessage = "Simplex did not converge within the iteration limit";
                return result;
            }

            int pivotCol = SimplexUtilities.FindPivotColumn(tableau);
            int pivotRow = SimplexUtilities.FindPivotRow(tableau, pivotCol);

            if (pivotRow == -1)
            {
                // No positive entry in the ratio-test column: feasible region is
                // unbounded in the direction of improvement.
                result.IsBounded = false;
                result.ErrorMessage = "Model is unbounded";
                return result;
            }

            SimplexUtilities.Pivot(tableau, pivotRow, pivotCol);
            tableau.IterationNumber = iteration;
            result.Iterations.Add(SimplexUtilities.Snapshot(tableau, iteration));

        }

        // Any artificial variable still basic with a positive value means the
        // ORIGINAL (pre-Big-M) model has no feasible solution — the Big-M penalty
        // couldn't fully drive it out.
        int rhsCol = tableau.Matrix.GetLength(1) - 1;
        for (int r = 0; r < tableau.BasicVariableIndices.Length; r++)
        {
            int basicCol = tableau.BasicVariableIndices[r];
            double value = tableau.Matrix[r + 1, rhsCol];
            if (IsArtificialColumn(model, basicCol) && value > Epsilon)
            {
                result.IsFeasible = false;
                result.ErrorMessage = "Model is infeasible (artificial variable remains in basis)";
                return result;
            }
        }

        // Populate BasisInverse/CB on the final tableau — Member 3's sensitivity
        // analysis reads these two fields directly off SolutionResult.FinalTableau,
        // regardless of which simplex variant produced the result (see workload
        // breakdown §0/§3). Trick: the columns that were basic in the INITIAL tableau
        // (iteration 0 — slack/artificial columns, which start as an identity matrix)
        // land on exactly B^-1 in the FINAL tableau, by standard tableau algebra
        // (Tableau_final = B^-1 * Tableau_initial, and B^-1 * identity_column = that
        // column of B^-1). So B^-1 can be read straight off the final tableau instead
        // of tracked separately through every pivot.
        var initialBasis = result.Iterations[0].BasicVariableIndices;
        int m = tableau.BasicVariableIndices.Length;
        var basisInverse = new double[m, m];
        for (int row = 0; row < m; row++)
        {
            for (int col = 0; col < m; col++)
            {
                basisInverse[row, col] = tableau.Matrix[row + 1, initialBasis[col]];
            }
        }
        tableau.BasisInverse = basisInverse;

        var costVector = SimplexUtilities.BuildCostVector(model, BigM);
        tableau.CB = tableau.BasicVariableIndices.Select(j => costVector[j]).ToArray();

        result.IsOptimal = true;
        result.FinalTableau = tableau;

        // Row 0's RHS holds the current max-form Z after every pivot (verified by hand
        // on a 1-variable example: Z - c^Tx = 0 evolves so matrix[0,RHS] == Z once
        // optimal). Un-negate for a Min model since we maximized its negated objective.
        double sign = model.Objective == ObjectiveType.Max ? 1.0 : -1.0;
        result.ObjectiveValue = sign * tableau.Matrix[0, rhsCol];

        result.VariableValues = new double[model.VariableCount];
        for (int r = 0; r < tableau.BasicVariableIndices.Length; r++)
        {
            int basicCol = tableau.BasicVariableIndices[r];
            if (basicCol < model.VariableCount)
            {
                result.VariableValues[basicCol] = tableau.Matrix[r + 1, rhsCol];
            }
        }
        // Non-basic structural variables are implicitly 0 — VariableValues defaults
        // to 0 for every index never touched above.

        return result;

    }

    // Recomputes whether a column index is an artificial-variable column by replaying
    // the same layout SimplexUtilities.BuildInitialTableau used. Works, but duplicates
    // knowledge of the column layout in two places — flagged as a cleanup candidate
    // (e.g. tag column types on the Tableau itself) rather than fixed unasked.
    private static bool IsArtificialColumn(LPModel model, int col)
    {
        int cursor = model.VariableCount;
        foreach (var c in model.Constraints)
        {
            switch (c.Relation)
            {
                case RelationType.LessThanOrEqualTo:
                    if (col == cursor) return false; //slack
                    cursor += 1;
                    break;

                case RelationType.GreaterThanOrEqualTo:
                    if (col == cursor) return false; //surplus
                    if (col == cursor + 1) return true; //artificial
                    cursor += 2;
                    break;

                case RelationType.EqualTo:
                    if (col == cursor) return true; //artificial
                    cursor += 1;
                    break;
            }    
        }
        return false;
    }

}