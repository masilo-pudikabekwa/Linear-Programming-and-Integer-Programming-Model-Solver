namespace Linear_Programming_and_Integer_Programming_Model_Solver.Core;

public enum RelationType
{
    LessThanOrEqualTo,
    GreaterThanOrEqualTo,
    EqualTo
}

public class Constraint
{

    public double[] Coefficients { get; set; }
    public RelationType Relation { get; set; }
    public double RHS { get; set; }

    public Constraint(double[] coefficients, RelationType relation, double rhs)
    {
        Coefficients = coefficients;
        Relation = relation;
        RHS = rhs;
    }

    // Method to clone the constraint
    //Branch and Bound clones a constraint list every time it adds
    //a new bound (x <= floor(v) / x >= ceil(v)) for a child sub-problem — without this,
    // mutating one branch's constraint would corrupt its sibling's array reference.
    public Constraint Clone()
    {
       return new Constraint((double[])Coefficients.Clone(), Relation, RHS);
    }
}
