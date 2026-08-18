namespace Linear_Programming_and_Integer_Programming_Model_Solver.Core;

public enum ObjectiveType
{
    Max,
    Min
}

//Singe shared representation of the "the problem",
//that every algorithm consumes and every I/O clas produces/reads.
public class LPModel
{

    public ObjectiveType Objective { get; set; }
    public double[] ObjectiveCoefficients { get; set; }
    public List<Constraint> Constraints { get; set; }

    //One restriction per decision variable, same order as ObjectiveCoefficients.
    public SignRestriction[] Restrictions { get; set; }

    // Derived, not stored — VariableCount can never drift out of sync with the
    // coefficients array because there's nothing to keep in sync.
    public int VariableCount => ObjectiveCoefficients.Length;

    public LPModel()
    {
        ObjectiveCoefficients = Array.Empty<double>();
        Constraints = new List<Constraint>();
        Restrictions = Array.Empty<SignRestriction>();
    }

    // Deep copy. This is the single most load-bearing method in the whole shared
    // foundation: every node in Member 2's Branch & Bound tree needs its OWN model
    // instance (parent's constraints + one extra bound), and a shallow copy here would
    // let sibling branches silently mutate each other's arrays/lists through shared
    // references

    public LPModel Clone()
    {
        return new LPModel
        {
            Objective = Objective,
            ObjectiveCoefficients = (double[])ObjectiveCoefficients.Clone(),
            Constraints = Constraints.Select(c => c.Clone()).ToList(),
            Restrictions = (SignRestriction[])Restrictions.Clone()
        };
    }
}