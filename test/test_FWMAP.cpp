#include "test_model.hxx"
#include "LP_FWMAP.hxx"
#include "solver.hxx"
#include "visitors/standard_visitor.hxx"

using namespace LP_MP;

int main(int argc, char** argv)
{
  Solver<test_FMC, LP_tree_FWMAP, StandardVisitor> s;
  auto& lp = s.GetLP();

  build_test_model(lp);

  s.Solve();
}


