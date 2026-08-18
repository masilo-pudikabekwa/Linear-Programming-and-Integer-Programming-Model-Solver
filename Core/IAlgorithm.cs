namespace Linear_Programming_and_Integer_Programming_Model_Solver.Core;

public interface IAlgorithm
{
    SolutionResult Solve(LPModel model);
}