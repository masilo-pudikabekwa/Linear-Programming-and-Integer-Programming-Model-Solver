using Linear_Programming_and_Integer_Programming_Model_Solver.Core;

namespace Linear_Programming_and_Integer_Programming_Model_Solver.Algorithms;

// Static helpers shared by PrimalSimplex (and reusable by Member 2's B&B Simplex,
// which solves each node via PrimalSimplex rather than reimplementing pivoting).
// Internally every model is solved as MAX: BuildInitialTableau negates a Min
// objective's coefficients up front, and the caller un-negates the final objective
// value. This means FindPivotColumn/IsOptimal never need to branch on Max vs Min.
public static class SimplexUtilities
{
    
    private const double Epsilon = 1e-9;

    // Big-M tableau construction: adds slack for <=, surplus+artificial for >=,
    // artificial for =, then eliminates the artificial columns out of the objective
    // row (Gauss-Jordan) so the returned tableau is already in canonical form
    // w.r.t. its initial (all-slack/artificial) basis.
    public static Tableau BuildInitialTableau(LPModel model, double bigM = 1_000_000)
    {
        int varCount = model.VariableCount;
        int constraintCount = model.Constraints.Count;

        int extraCols = model.Constraints.Sum(c => c.Relation switch
        {
            RelationType.LessThanOrEqualTo => 1,        // slack
            RelationType.GreaterThanOrEqualTo => 2,     // surplus + artificial
            RelationType.EqualTo => 1,                  // artificial
            _ => 0
        });

        int totalCols = varCount + extraCols + 1; // +1 for RHS
        int totalRows = constraintCount + 1; // +1 for objective row

        var matrix = new double[totalRows, totalCols];
        var basicVars = new int[constraintCount];

        // Objective row (row 0), reduced-cost form: "Z - c^T x = 0" rearranged so
        // structural variables start at -c_j (for max — a Min model is negated here
        // via 'sign' so the rest of the algorithm never has to know the difference).
        double sign = model.Objective == ObjectiveType.Max ? 1.0 : -1.0;
        for (int j = 0; j < varCount; j++)
        {
            matrix[0, j] = -sign * model.ObjectiveCoefficients[j];
        }

        int colCursor = varCount;
        var artificials = new List<(int col, int row)>();

        for (int i = 0; i < constraintCount; i++)
        {
            var constraint = model.Constraints[i];
            int row = i + 1;

            for (int j = 0; j < varCount; j++)
            {
                matrix[row, j] = constraint.Coefficients[j];
            }

            switch (constraint.Relation) 
            {
                case RelationType.LessThanOrEqualTo:
                    matrix[row, colCursor] = 1; // slack
                    basicVars[i] = colCursor;
                    colCursor++;
                    break;
                
                case RelationType.GreaterThanOrEqualTo:
                    matrix[row, colCursor] = -1; // surplus
                    colCursor++;
                    matrix[row, colCursor] = 1; // artificial
                    basicVars[i] = colCursor;
                    artificials.Add((colCursor, row));
                    colCursor++;
                    break;

                case RelationType.EqualTo:
                    matrix[row, colCursor] = 1; // artificial
                    basicVars[i] = colCursor;
                    artificials.Add((colCursor, row));
                    colCursor++;
                    break;
            }

            matrix[row, totalCols - 1] = constraint.RHS; // RHS
        }

        // Penalize artificials with Big-M, then eliminate them from the objective row
        // (they start basic, so row 0 must read 0 in their columns to be canonical).
        foreach (var (col, _) in artificials)
        {
            matrix[0, col] = bigM;
        }

        foreach (var (col, row) in artificials)
        {
            for (int j = 0; j < totalCols; j++)
            {
                matrix[0, j] -= bigM * matrix[row, j];
            }  
        }

        return new Tableau
        {
            Matrix = matrix,
            BasicVariableIndices = basicVars,
            IterationNumber = 0
        };
    }

    // Entering column: most negative entry in the objective row (excluding RHS).
    // Returns -1 when the tableau is already optimal.
    public static int FindPivotColumn(Tableau tableau)
    {
        int totalCols = tableau.Matrix.GetLength(1);
        int best = -1;
        double bestValue = -Epsilon;

        for (int j = 0; j < totalCols - 1; j++) 
        {
            if (tableau.Matrix[0, j] < bestValue)
            {
                bestValue = tableau.Matrix[0, j];
                best = j;
            }
        }
        return best;
    }

    // Minimum ratio test. Returns -1 when no row has a positive coefficient in the
    // entering column, i.e. the model is unbounded in that direction.
    public static int FindPivotRow(Tableau tableau, int pivotCol)
    {
        int rows = tableau.Matrix.GetLength(0);
        int rhsCol = tableau.Matrix.GetLength(1) - 1;

        int best = -1;
        double bestRatio = double.PositiveInfinity;

        for (int r = 1; r < rows; r++)
        {
            double coeff = tableau.Matrix[r, pivotCol];
            if (coeff > Epsilon)
            {
                double ratio = tableau.Matrix[r, rhsCol] / coeff;

                if (ratio < bestRatio - Epsilon)
                {
                    bestRatio = ratio;
                    best = r;
                }
            }
        }
        return best;
    }

    // Standard Gauss-Jordan pivot: normalize the pivot row, eliminate the pivot
    // column from every other row, then update which variable is basic in this row.
    public static void Pivot(Tableau tableau, int pivotRow, int pivotCol) 
    {
        int rows = tableau.Matrix.GetLength(0);
        int cols = tableau.Matrix.GetLength(1);
        double pivotValue = tableau.Matrix[pivotRow, pivotCol];

        if (Math.Abs(pivotValue) < Epsilon)
        {
            throw new InvalidOperationException("Pivot element is (near) zero — degenerate tableau.");
        }

        for (int j = 0; j < cols; j++)
        {
            tableau.Matrix[pivotRow, j] /= pivotValue;
        }

        for (int r = 0; r < rows; r++)
        {
            if (r == pivotRow) continue;
            double factor = tableau.Matrix[r, pivotCol];
            if (Math.Abs(factor) < Epsilon) continue;
            for (int j = 0; j < cols; j++)
            {
                tableau.Matrix[r, j] -= factor * tableau.Matrix[pivotRow, j];
            }
        }

        tableau.BasicVariableIndices[pivotRow - 1] = pivotCol;
    }

    public static bool IsOptimal(Tableau tableau) => FindPivotColumn(tableau) == -1;

    // Internal max-form cost for every column in BuildInitialTableau's layout:
    // structural variables use sign*c_j (Max solved directly, Min negated), slack and
    // surplus cost nothing, artificials are penalized at -bigM. Shared by PrimalSimplex
    // (to back out CB after solving, see below) and mirrored inline by
    // RevisedPrimalSimplex's own matrix-form builder — both must agree on what "the
    // cost of column j" means for the same input model.
    public static double[] BuildCostVector(LPModel model, double bigM)
    {
        int varCount = model.VariableCount;
        int extraCols = model.Constraints.Sum(c => c.Relation switch
        {
            RelationType.LessThanOrEqualTo => 1,
            RelationType.GreaterThanOrEqualTo => 2,
            RelationType.EqualTo => 1,
            _ => 0
        });

        var costs = new double[varCount + extraCols];
        double sign = model.Objective == ObjectiveType.Max ? 1.0 : -1.0;
        for (int j = 0; j < varCount; j++)
        {
            costs[j] = sign * model.ObjectiveCoefficients[j];
        }

        int colCursor = varCount;
        foreach (var c in model.Constraints)
        {
            switch (c.Relation)
            {
                case RelationType.LessThanOrEqualTo:
                    colCursor += 1; // slack cost 0 (array default)
                    break;

                case RelationType.GreaterThanOrEqualTo:
                    colCursor += 1;     // surplus cost 0
                    costs[colCursor] = -bigM;
                    colCursor += 1;
                    break;

                case RelationType.EqualTo:
                    costs[colCursor] = -bigM;
                    colCursor += 1;
                    break;
            }
        }
        return costs;
    }

    // Deep copy for SolutionResult.Iterations — see Tableau.Clone() for why this
    // can't just store a reference to the live, still-being-pivoted matrix.
    public static Tableau Snapshot(Tableau tableau, int iterationNumber)
    {
        int rows = tableau.Matrix.GetLength(0);
        int cols = tableau.Matrix.GetLength(1);

        var copy = new double[rows, cols];
        Array.Copy(tableau.Matrix, copy, tableau.Matrix.Length);

        return new Tableau
        {
            Matrix = copy,
            BasicVariableIndices = (int[])tableau.BasicVariableIndices.Clone(),
            BasisInverse = tableau.BasisInverse,
            CB = tableau.CB,
            IterationNumber = iterationNumber
        };
    }
}
