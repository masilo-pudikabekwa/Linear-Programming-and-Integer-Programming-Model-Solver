namespace Linear_Programming_and_Integer_Programming_Model_Solver.Algorithms;

public class KnapsackNode
{
    public int Level { get; set; }

    public double Value { get; set; }

    public double Weight { get; set; }

    public double Bound { get; set; }

    public bool[] Included { get; set; } = Array.Empty<bool>();
}