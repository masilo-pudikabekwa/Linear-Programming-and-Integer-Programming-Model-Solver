namespace Linear_Programming_and_Integer_Programming_Model_Solver.Core;

// Every IAlgorithm.Solve() call returns one of these, regardless of which algorithm
// ran. Infeasible/unbounded are EXPECTED outcomes signalled through the flags below,
// not exceptions — this is what lets Member 2's B&B fathom a node just by checking
// node.Relaxation.IsFeasible instead of wrapping every call in a try/catch.

public class SolutionResult
{
    public bool IsFeasible { get; set; } = true;
    public bool IsBounded { get; set; } = true;
    public bool IsOptimal { get; set; }
    public double ObjectiveValue { get; set; }

    public string? ErrorMessage { get; set; }

    public double[] VariableValues { get; set; }

    //Every tableau is shown, in solve order - this is the "display all tableau iterations" feature.
    public List<Tableau> Iterations { get; set; }

    //Member 3 reads this directly for BasisInverse/CB, must be non null whenever
    //IsOptimal is true.
    public Tableau FinalTableau { get; set; } = new();

    public SolutionResult()
    {
        VariableValues = Array.Empty<double>();
        Iterations = new List<Tableau>();
    }
}