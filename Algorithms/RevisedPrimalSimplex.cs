using System.Diagnostics;
using Linear_Programming_and_Integer_Programming_Model_Solver.Core;

namespace Linear_Programming_and_Integer_Programming_Model_Solver.Algorithms;

// Matrix-form ("revised") Simplex. Instead of carrying a full dense tableau through
// every pivot, this maintains B^-1 via ProductFormBasisInverse (eta matrices) and
// recomputes only what's needed each iteration: reduced costs ("price out") and
// B^-1 * b (the current basic solution) — the two things the spec's "Product Form
// and Price Out iterations" display requirement actually asks for.
public class RevisedPrimalSimplex : IAlgorithm
{
    private const int MaxIterations = 1000;
    private const double Epsilon = 1e-9;
    private const double BigM = 1_000_000;

    public SolutionResult Solve(LPModel model)
    {
        var result = new SolutionResult();
        var (A, c, b, basis, artificialCols) = BuildStandardForm(model);
        int m = A.GetLength(0);
        int totalVars = A.GetLength(1);

        var pfi = new ProductFormBasisInverse(m);

        int iteration = 0;
        result.Iterations.Add(SnapshotIteration(pfi, basis, c, b, iteration));

        while (true)
        {
            double[,] Binv = pfi.ComputeBasisInverse();
            double[] xB = MatVec(Binv, b);
            double[] CB = basis.Select(j => c[j]).ToArray();

            // Price out: reduced cost r_j = c_j - CB . B^-1 . A_j for every non-basic j.
            // Dantzig's rule: enter the column with the largest positive reduced cost.
            int enteringCol = -1;
            double bestReducedCost = Epsilon;
            double[] enteringDirection = Array.Empty<double>();

            for (int j = 0; j < totalVars; j++)
            {
                if (basis.Contains(j)) continue;

                var Aj = GetColumn(A, j);
                var direction = MatVec(Binv, Aj);
                double zj = Dot(CB, direction);
                double reducedCost = c[j] - zj;

                if (reducedCost > bestReducedCost)
                {
                    bestReducedCost = reducedCost;
                    enteringCol = j;
                    enteringDirection = direction;
                }
            }

            if (enteringCol == -1)
            {
                break; // optimal — no non-basic column can still improve Z  
            }

            if (++iteration > MaxIterations)
            {
                result.IsFeasible = false;
                result.ErrorMessage = "Revised Simplex did not converge within the iteration limit";
                return result;
            }

            // Ratio test on the transformed entering column
            int leavingRow = -1;
            double bestRatio = double.PositiveInfinity;
            for (int i = 0; i < m; i++)
            {
                if (enteringDirection[i] > Epsilon)
                {
                    double ratio = xB[i] / enteringDirection[i];
                    if (ratio < bestRatio - Epsilon)
                    {
                        bestRatio = ratio;
                        leavingRow = i;
                    }
                }
            }

            if (leavingRow == -1)
            {
                result.IsBounded = false;
                result.ErrorMessage = "Model is unbounded";
                return result;
            }

            // Record this pivot as an eta matrix (the "Product Form" update) and swap
            // the basis — no dense tableau touched at all
            pfi.AddEta(leavingRow, enteringDirection);
            basis[leavingRow] = enteringCol;

            result.Iterations.Add(SnapshotIteration(pfi, basis, c, b, iteration));
        }

        // Infeasibility check: any artificial variable left basic with a positive value
        var finalBinv = pfi.ComputeBasisInverse();
        var finalXB = MatVec(finalBinv, b);
        for (int i = 0; i < m; i++)
        {
            if (artificialCols.Contains(basis[i]) && finalXB[i] > Epsilon)
            {
                result.IsFeasible = false;
                result.ErrorMessage = "Model is infeasible (artificial variable remains in basis)";
                return result;
            }
        }

        result.IsOptimal = true;
        result.FinalTableau = SnapshotIteration(pfi, basis, c, b, iteration);

        double sign = model.Objective == ObjectiveType.Max ? 1.0 : -1.0;
        var CBFinal = basis.Select(j => c[j]).ToArray();
        result.ObjectiveValue = sign * Dot(CBFinal, finalXB);

        result.VariableValues = new double[model.VariableCount];
        for (int i = 0; i < m; i++)
        {
            if (basis[i] < model.VariableCount)
            {
                result.VariableValues[basis[i]] = finalXB[i];
            }
        }

        return result;
    }

    // Builds A, c, b in standard equality form: slack for <=, surplus+artificial for >=,
    // artificial for =. Deliberately mirrors SimplexUtilities.BuildInitialTableau's
    // column order so PrimalSimplex and RevisedPrimalSimplex agree on what column N
    // means for the same input model (matters if any downstream code compares them).
    private static (double[,] A, double[] c, double[] b, List<int> basis, HashSet<int> artificialCols) BuildStandardForm(LPModel model)
    {
        int varCount = model.VariableCount;
        int m = model.Constraints.Count;

        int extraCols = model.Constraints.Sum(cst => cst.Relation switch
        {
            RelationType.LessThanOrEqualTo => 1,
            RelationType.GreaterThanOrEqualTo => 2,
            RelationType.EqualTo => 1,
            _ => 0
        });

        int totalVars = varCount + extraCols;
        var A = new double[m, totalVars];
        var c = new double[totalVars];
        var b = new double[m];
        var basis = new List<int>(new int[m]);
        var artificialCols = new HashSet<int>();

        double sign = model.Objective == ObjectiveType.Max ? 1.0 : -1.0;
        for (int j = 0; j < varCount; j++)
        {
            c[j] = sign * model.ObjectiveCoefficients[j];
        }

        int colCursor = varCount;
        for (int i = 0; i < m; i++)
        {
            var constraint = model.Constraints[i];
            for (int j = 0; j < varCount; j++)
            {
                A[i, j] = constraint.Coefficients[j];
            }

            switch (constraint.Relation)
            {
                case RelationType.LessThanOrEqualTo:
                    A[i, colCursor] = 1;
                    basis[i] = colCursor;
                    colCursor++;
                    break;
                case RelationType.GreaterThanOrEqualTo:
                    A[i, colCursor] = -1;
                    colCursor++;
                    A[i, colCursor] = 1;
                    c[colCursor] = -BigM; // heavily penalized in the internal max-form objective
                    basis[i] = colCursor;
                    artificialCols.Add(colCursor);
                    colCursor++;
                    break;
                case RelationType.EqualTo:
                    A[i, colCursor] = 1;
                    c[colCursor] = -BigM;
                    basis[i] = colCursor;
                    artificialCols.Add(colCursor);
                    colCursor++;
                    break;
            }
            b[i] = constraint.RHS;
        }

        return (A, c, b, basis, artificialCols);
    }

    // Packs the current iteration into the shared Tableau shape: B^-1 augmented with
    // the current basic solution B^-1*b as the last column, plus CB — exactly the two
    // fields (BasisInverse, CB) Member 3's sensitivity analysis reads directly off
    // SolutionResult.FinalTableau, regardless of which simplex variant produced it
    private static Tableau SnapshotIteration(ProductFormBasisInverse pfi, List<int> basis, double[] c, double[] b, int iterationNumber)
    {
        var Binv = pfi.ComputeBasisInverse();
        var xB = MatVec(Binv, b);
        int m = basis.Count;

        var matrix = new double[m, m + 1];
        for (int i = 0; i < m; i++)
        {
            // Populate the first 'm' columns with the Basis Inverse (B^-1)
            for (int j = 0; j < m; j++)
            {
                matrix[i, j] = Binv[i, j];
            }
            // Set the final column to the current basic solution (B^-1 * b)
            matrix[i, m] = xB[i];
        }

        return new Tableau
        {
            Matrix = matrix,
            BasicVariableIndices = basis.ToArray(),
            BasisInverse = Binv,
            CB = basis.Select(j => c[j]).ToArray(),
            IterationNumber = iterationNumber
        };
    }

    private static double[] GetColumn(double[,] A, int col)
    {
        int rows = A.GetLength(0);
        var result = new double[rows];
        for (int i = 0; i < rows; i++) result[i] = A[i, col];
        return result;
    }

    private static double[] MatVec(double[,] M, double[] v)
    {
        int rows = M.GetLength(0);
        int cols = M.GetLength(1);

        if (v.Length != cols)
        {
            throw new ArgumentException(
                "Matrix and vector dimensions do not match."
            );
        }

        var result = new double[rows];

        for (int i = 0; i < rows; i++)
        {
            double sum = 0;

            for (int j = 0; j < cols; j++)
            {
                sum += M[i, j] * v[j];
            }

            result[i] = sum;
        }

        return result;
    }

    private static double Dot(double[] a, double[] b)
    {
        if (a.Length != b.Length)
        {
            throw new ArgumentException(
                "Vector dimensions do not match."
            );
        }

        double sum = 0;

        for (int i = 0; i < a.Length; i++)
        {
            sum += a[i] * b[i];
        }

        return sum;
    }
}