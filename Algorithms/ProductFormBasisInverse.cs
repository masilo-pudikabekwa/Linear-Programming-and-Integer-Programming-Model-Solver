using System.Data.Common;

namespace Linear_Programming_and_Integer_Programming_Model_Solver.Algorithms;

// Product Form of the Inverse: rather than storing a dense B^-1 and recomputing it
// from scratch every pivot, each pivot is recorded as a single "eta" (elementary)
// matrix. B^-1 is then reconstructed on demand by applying the eta sequence to the
// identity. This is what makes Revised Simplex "revised" instead of a rebuild of
// tableau simplex in matrix clothing — and it's reused as-is by Member 2's
// CuttingPlaneAlgorithm for its Product Form / Price Out iterations.

public class ProductFormBasisInverse
{
    // Identity matrix with one column replaced by the (already B^-1-transformed)
    // pivot column of that iteration. Applying it is O(m), not a full O(m^3) multiply.
    public class EtaMatrix
    {
        public int PivotColumn { get; set; }     // basis ROW position this eta updates
        public double[] Column { get; set; }    // transformed pivot column at that iteration 

        public EtaMatrix(int pivotColumn, double[] column)
        {
            PivotColumn = pivotColumn;
            Column = column;
        }

    }

    private readonly List<EtaMatrix> _etas = new();
    private readonly int _dimension;

    public ProductFormBasisInverse(int dimension)
    {
        _dimension = dimension;
    }

    public IReadOnlyList<EtaMatrix> Etas => _etas;

    // Records a new pivot as an eta matrix. 'transformedColumn' must already be
    // B^-1 * (entering column) under the CURRENT basis, i.e. the direction vector
    // computed during that iteration's price-out step.
    public void AddEta(int pivotRow, double[] transformedColumn)
    {
        if (transformedColumn.Length != _dimension)
        {
            throw new ArgumentException(
                $"Eta column length {transformedColumn.Length} does not match basis {_dimension}"
            );
        }

        _etas.Add(new EtaMatrix(pivotRow, (double[])transformedColumn.Clone()));
    }

    // Reconstructs the dense B^-1 by applying every recorded eta, in order, to the
    // identity. Called on demand (e.g. once per iteration for display, once at the
    // end for Member 3's sensitivity analysis) rather than kept dense at every step.
    public double[,] ComputeBasisInverse()
    {
        var result = Identity(_dimension);
        foreach (var eta in _etas)
        {
            ApplyEta(result, eta);
        }

        return result;
    }


    // Applies one eta update to every column of 'target' in place — the standard
    // product-form pivot update, generalized from a vector to a full matrix.
    private void ApplyEta(double[,] target, EtaMatrix eta)
    {
        int n = _dimension;
        double pivotValue = eta.Column[eta.PivotColumn];

        if (Math.Abs(pivotValue) < 1e-9)
        {
            throw new InvalidOperationException("Degenerate eta matrix - pivot value is zero");
        }

        for (int col = 0; col < n; col++)
        {
            double pivotRowValue = target[eta.PivotColumn, col];
            for (int row = 0; row < n; row++)
            {
                if (row == eta.PivotColumn) continue;
                target[row, col] -= eta.Column[row] / pivotValue * pivotRowValue;
            }

            target[eta.PivotColumn, col] = pivotRowValue / pivotValue;
        }
    }

    private static double[,] Identity(int n)
    {
        var m = new double[n, n];
        for (int i = 0; i < n; i++) m[i, i] = 1.0;
        return m;
    }

    // Solves B * x = b for x by applying the eta sequence directly to the vector,
    // instead of materializing B^-1 first — the cheaper path used mid-solve
    // (RevisedPrimalSimplex uses ComputeBasisInverse() instead purely for the
    // per-iteration display snapshot, where the dense form is what gets shown).
    public double[] Solve(double[] b)
    {
        var x = (double[])b.Clone();
        foreach (var eta in _etas)
        {
            double pivotValue = eta.Column[eta.PivotColumn];
            double pivotEntry = x[eta.PivotColumn];
            for (int row = 0; row < _dimension; row++)
            {
                if (row == eta.PivotColumn) continue;
                x[row] -= eta.Column[row] / pivotValue * pivotEntry;
            }

            x[eta.PivotColumn] = pivotEntry / pivotValue;
        }
        return x;
    }
}