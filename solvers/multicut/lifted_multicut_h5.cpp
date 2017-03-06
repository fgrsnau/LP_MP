#include "multicut.h"
#include "visitors/standard_visitor.hxx"
#include "solver.hxx"
using namespace LP_MP;
using FMC = FMC_LIFTED_MULTICUT;
//using SolverType = ProblemConstructorRoundingSolver<FMC>;
using SolverType = ProblemConstructorRoundingSolver<Solver<FMC,LP,StandardTighteningVisitor>>;
//using InputFct = MulticutH5Input::ParseLiftedProblem<SolverType>;
int main(int argc, char* argv[])
{
   SolverType solver(argc,argv);
   solver.ReadProblem(MulticutH5Input::ParseLiftedProblem<Solver<FMC,LP,StandardTighteningVisitor>>);
   return solver.Solve();
}

//LP_MP_CONSTRUCT_SOLVER_WITH_INPUT_AND_VISITOR_AND_SOLVER(FMC, InputFct, StandardTighteningVisitor, SolverType);

