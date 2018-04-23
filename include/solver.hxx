#ifndef LP_MP_SOLVER_HXX
#define LP_MP_SOLVER_HXX

#include <type_traits>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <fstream>
#include <sstream>

#include "LP_MP.h"
#include "function_existence.hxx"
#include "template_utilities.hxx"
#include "static_if.hxx"
#include "tclap/CmdLine.h"
#include "lp_interface/lp_interface.h"

namespace LP_MP {

static std::vector<std::string> default_solver_options = {
   {""}, 
   {"-i"}, {""}, 
   {"--maxIter"}, {"1000"}
};

// class containing the LP, problem constructor list, input function and visitor
// binds together problem constructors and solver and organizes input/output
// base class for solvers with primal rounding, e.g. LP-based rounding heuristics, message passing rounding and rounding provided by problem constructors.

template<typename LP_TYPE, typename VISITOR>
class Solver {

public:
   using SolverType = Solver<LP_TYPE, VISITOR>;
   using FMC = typename LP_TYPE::FMC;
   using ProblemDecompositionList = typename FMC::ProblemDecompositionList;

   // default parameters

   Solver() : Solver(default_solver_options) {}

   Solver(int argc, char** argv) : Solver(ProblemDecompositionList{}) 
   {
      cmd_.parse(argc,argv);
      Init_(); 
   }
   Solver(std::vector<std::string> options) : Solver(ProblemDecompositionList{})
   {
      cmd_.parse(options);
      Init_(); 
   }

private:
   template<typename... PROBLEM_CONSTRUCTORS>
   Solver(meta::list<PROBLEM_CONSTRUCTORS...>&& pc_list)
     :
        cmd_(std::string("Command line options for ") + FMC::name, ' ', "0.0.1"),
        lp_(cmd_),
        inputFileArg_("i","inputFile","file from which to read problem instance",false,"","file name",cmd_),
        outputFileArg_("o","outputFile","file to write solution",false,"","file name",cmd_),
        verbosity_arg_("v","verbosity","verbosity level: 0 = silent, 1 = important runtime information, 2 = further diagnostics",false,1,"0,1,2",cmd_),
        visitor_(cmd_)
   {
      for_each_tuple(this->problemConstructor_, [this](auto& l) {
           assert(l == nullptr);
           l = new typename std::remove_pointer<typename std::remove_reference<decltype(l)>::type>::type(*this); // note: this is not so nice: if problem constructor needs other problem constructors, those must already be allocated, otherwise address is invalid. This is only a problem for circular references, though, otherwise order problem constructors accordingly. This should be resolved when std::tuple will be constructed without move and copy constructors.
      }); 

      std::cout << std::setprecision(10);
   }

public:

   ~Solver() 
   {
      for_each_tuple(this->problemConstructor_, [this](auto& l) {
            assert(l != nullptr);
            delete l;
      });
   }

   TCLAP::CmdLine& get_cmd() { return cmd_; }

   void Init_()
   {
      try {  
         inputFile_ = inputFileArg_.getValue();
         outputFile_ = outputFileArg_.getValue();
         verbosity = verbosity_arg_.getValue();
         if(verbosity > 2) { throw TCLAP::ArgException("verbosity must be 0,1 or 2"); }
      } catch (TCLAP::ArgException &e) {
         std::cerr << "error: " << e.error() << " for arg " << e.argId() << std::endl; 
         exit(1);
      }
   }

   template<class INPUT_FUNCTION, typename... ARGS>
   bool ReadProblem(INPUT_FUNCTION inputFct, ARGS... args)
   {
      const bool success = inputFct(inputFile_, *this, args...);

      assert(success);
      if(!success) throw std::runtime_error("could not parse problem file");

      //spdlog::get("logger")->info("loading file " + inputFile_ + " succeeded");
      return success;
   }

   LP_MP_FUNCTION_EXISTENCE_CLASS(HasWritePrimal,WritePrimal)
   template<typename PC>
   constexpr static bool
   CanWritePrimalIntoFile()
   {
      return HasWritePrimal<PC, void, std::ofstream>();
   }
   template<typename PC>
   constexpr static bool
   CanWritePrimalIntoString()
   {
      return HasWritePrimal<PC, void, std::stringstream>();
   } 

   void WritePrimal()
   {
      if(outputFileArg_.isSet()) {
         std::ofstream output_file;
         output_file.open(outputFile_, std::ofstream::out);
         if(!output_file.is_open()) {
            throw std::runtime_error("could not open file " + outputFile_);
         }
         
         output_file << solution_; // to do: not so nice, better have primal serialized, write back into factors, then write primal?

         //for_each_tuple(this->problemConstructor_, [this,&output_file](auto* l) {
         //      using pc_type = typename std::remove_pointer<decltype(l)>::type;
         //      static_if<SolverType::CanWritePrimalIntoFile<pc_type>()>([&](auto f) {
         //            f(*l).WritePrimal(output_file);
         //      });
         //}); 
      }
   }

   std::string write_primal_into_string()
   {
      std::stringstream ss;

      for_each_tuple(this->problemConstructor_, [&ss,this](auto* l) {
            using pc_type = typename std::remove_pointer<decltype(l)>::type;
            static_if<SolverType::CanWritePrimalIntoString<pc_type>()>([&](auto f) {
                  f(*l).WritePrimal(ss);
            });
      }); 

      std::string sol = ss.str();
      return std::move(sol);
   }

   // invoke the corresponding functions of problem constructors
   LP_MP_FUNCTION_EXISTENCE_CLASS(HasCheckPrimalConsistency,CheckPrimalConsistency)
   template<typename PROBLEM_CONSTRUCTOR>
   constexpr static bool
   CanCheckPrimalConsistency()
   {
      return HasCheckPrimalConsistency<PROBLEM_CONSTRUCTOR, bool>();
   }
   
   bool CheckPrimalConsistency()
   {
      bool feasible = true;
      for_each_tuple(this->problemConstructor_, [this,&feasible](auto* l) {
            using pc_type = typename std::remove_pointer<decltype(l)>::type;
            static_if<SolverType::CanCheckPrimalConsistency<pc_type>()>([&](auto f) {
                  if(feasible) {
                     const bool feasible_pc = f(*l).CheckPrimalConsistency();
                     if(!feasible_pc) {
                        feasible = false;
                     }
                  }
            });
      });

      if(feasible) {
         feasible = this->lp_.CheckPrimalConsistency();
      }

      return feasible;
   }


   LP_MP_FUNCTION_EXISTENCE_CLASS(HasTighten,Tighten)
   template<typename PROBLEM_CONSTRUCTOR>
   constexpr static bool
   CanTighten()
   {
      return HasTighten<PROBLEM_CONSTRUCTOR, INDEX, INDEX>();
   }

   // maxConstraints gives maximum number of constraints to add for each problem constructor
   INDEX Tighten(const INDEX maxConstraints) 
   {
      INDEX constraints_added = 0;
      for_each_tuple(this->problemConstructor_, [this,maxConstraints,&constraints_added](auto* l) {
            using pc_type = typename std::remove_pointer<decltype(l)>::type;
            static_if<SolverType::CanTighten<pc_type>()>([&](auto f) {
                  constraints_added += f(*l).Tighten(maxConstraints);
            });
       });

      return constraints_added;
   }
   
   template<INDEX PROBLEM_CONSTRUCTOR_NO>
   meta::at_c<ProblemDecompositionList, PROBLEM_CONSTRUCTOR_NO>& GetProblemConstructor() 
   {
      auto& pc = *std::get<PROBLEM_CONSTRUCTOR_NO>(problemConstructor_);
      return pc;
   }

   LP_TYPE& GetLP() { return lp_; }
   
   LP_MP_FUNCTION_EXISTENCE_CLASS(has_solution,solution)
   constexpr static bool
   visitor_has_solution()
   {
      return has_solution<VISITOR, void, std::string>();
   }
   

   
   int Solve()
   {
      if(debug()) {
         std::cout << "lower bound before optimization = " << lp_.LowerBound() << "\n";
      }

      this->Begin();
      LpControl c = visitor_.begin(this->lp_);
      while(!c.end && !c.error) {
         this->PreIterate(c);
         this->Iterate(c);
         this->PostIterate(c);
         c = visitor_.visit(c, this->lowerBound_, this->bestPrimalCost_);
         ++iter;
      }
      if(!c.error) {
         this->End();
         RegisterPrimal();
         lowerBound_ = lp_.LowerBound();
         // possibly primal has been computed in end. Call visitor again
         visitor_.end(this->lowerBound_, this->bestPrimalCost_);
         static_if<visitor_has_solution()>([this](auto f) {
               f(this)->visitor_.solution(this->solution_);
         });
         this->WritePrimal();
      }
      return !c.error;
   }


   // called before first iterations
   virtual void Begin() 
   {
      lp_.Begin(); 
   }

   // what to do before improving lower bound, e.g. setting reparametrization mode
   virtual void PreIterate(LpControl c) 
   {
      lp_.set_reparametrization(c.repam);
   } 

   // what to do for improving lower bound, typically ComputePass or ComputePassAndPrimal
   virtual void Iterate(LpControl c) {
      lp_.ComputePass(iter);
   } 

   // what to do after one iteration of message passing, e.g. primal computation and/or tightening
   virtual void PostIterate(LpControl c) 
   {
      if(c.computeLowerBound) {
         lowerBound_ = lp_.LowerBound();
         assert(std::isfinite(lowerBound_));
      }
      if(c.tighten) {
         Tighten(c.tightenConstraints);
      }
   } 

   // called after last iteration
   LP_MP_FUNCTION_EXISTENCE_CLASS(HasEnd,End)
   template<typename PROBLEM_CONSTRUCTOR>
   constexpr static bool
   CanCallEnd()
   {
      return HasEnd<PROBLEM_CONSTRUCTOR, void>();
   }

   virtual void End() 
   {
      for_each_tuple(this->problemConstructor_, [this](auto* l) {
            using pc_type = typename std::remove_pointer<decltype(l)>::type;
            static_if<SolverType::CanCallEnd<pc_type>()>([&](auto f) {
                  f(*l).End();
            });
      }); 
      lp_.End();
   }

   // register evaluated primal solution
   void RegisterPrimal(const REAL cost)
   {
      assert(false);
      if(cost < bestPrimalCost_) {
         // assume solution is feasible
         bestPrimalCost_ = cost;
         solution_ = write_primal_into_string();
      }
   }

   // evaluate and register primal solution
   void RegisterPrimal()
   {
      const REAL cost = lp_.EvaluatePrimal();
      if(debug()) { std::cout << "register primal cost = " << cost << "\n"; }
      if(cost < bestPrimalCost_) {
         // assume solution is feasible
         const bool feasible = CheckPrimalConsistency();
         if(feasible) {
            if(debug()) {
               std::cout << "solution feasible\n";
            }
            bestPrimalCost_ = cost;
            solution_ = write_primal_into_string();
         } else {
            if(debug()) {
               std::cout << "solution infeasible\n";
            }
         } 
      }
   }

   REAL lower_bound() const { return lowerBound_; }
   REAL primal_cost() const { return bestPrimalCost_; }

protected:
   TCLAP::CmdLine cmd_;

   LP_TYPE lp_;

   struct add_pointer {
      template<typename T>
         using invoke = T*;
      template<typename T>
         using type = T*;
   };
   using constructor_pointer_list = meta::transform< ProblemDecompositionList, add_pointer >;
   meta::apply<meta::quote<std::tuple>, constructor_pointer_list> problemConstructor_;
   // unfortunately, std::tuple cannot intialize properly when tuple elements are non-copyable and non-moveable. This happens, when problem constructors hold TCLAP arguments. Hence we must hold references to problem constructors, which is less nice.
   //meta::apply<meta::quote<std::tuple>, ProblemDecompositionList> problemConstructor_;
   //tuple_from_list<ProblemDecompositionList> problemConstructor_;
   //problem_constructor_storage_from_list<ProblemDecompositionList> problemConstructor_;

   // command line arguments
   TCLAP::ValueArg<std::string> inputFileArg_;
   TCLAP::ValueArg<std::string> outputFileArg_;
   std::string inputFile_;
   std::string outputFile_;

   TCLAP::ValueArg<INDEX> verbosity_arg_;

   REAL lowerBound_;
   // while Solver does not know how to compute primal, derived solvers do know. After computing a primal, they are expected to register their primals with the base solver
   REAL bestPrimalCost_ = std::numeric_limits<REAL>::infinity();
   std::string solution_;

   VISITOR visitor_;
   INDEX iter = 0;
};

// local rounding interleaved with message passing 
template<typename SOLVER>
class MpRoundingSolver : public SOLVER
{
public:
  using SOLVER::SOLVER;

  virtual void Iterate(LpControl c)
  {
    if(c.computePrimal) {
      SOLVER::lp_.ComputeForwardPassAndPrimal(this->iter);
      this->RegisterPrimal();
      SOLVER::lp_.ComputeBackwardPassAndPrimal(this->iter);
      this->RegisterPrimal();
    } else {
      SOLVER::Iterate(c);
    }
  }

private:
};

// rounding based on primal heuristics provided by problem constructor
template<typename SOLVER>
class ProblemConstructorRoundingSolver : public SOLVER
{
public:
   using SOLVER::SOLVER;

   LP_MP_FUNCTION_EXISTENCE_CLASS(HasComputePrimal,ComputePrimal)
   template<typename PROBLEM_CONSTRUCTOR>
   constexpr static bool
   CanComputePrimal()
   {
      return HasComputePrimal<PROBLEM_CONSTRUCTOR, void>();
   }

   void ComputePrimal()
   {
      // compute the primal in parallel.
      // for this, first we have to wait until the rounding procedure has read off everything from the LP model before optimizing further
      for_each_tuple(this->problemConstructor_, [this](auto* l) {
            using pc_type = typename std::remove_pointer<decltype(l)>::type;
            static_if<ProblemConstructorRoundingSolver<SOLVER>::CanComputePrimal<pc_type>()>([&](auto f) {
                  f(*l).ComputePrimal();
            });
      });
   }

   virtual void PostIterate(LpControl c)
   {
      if(c.computePrimal) {
         // do zrobienia: possibly run this in own thread similar to lp solver
         ComputePrimal();
         this->RegisterPrimal();
      }
      SOLVER::PostIterate(c);
   }

   virtual void End()
   {
      SOLVER::End(); // first let problem constructors end (done in Solver)
      this->RegisterPrimal();
   }
   
};

// rounding based on (i) interleaved message passing followed by (ii) problem constructor rounding.
// It is expected that (ii) takes (i)'s rounding into account.
template<typename SOLVER>
class CombinedMPProblemConstructorRoundingSolver : public ProblemConstructorRoundingSolver<SOLVER>
{
public:
   using ProblemConstructorRoundingSolver<SOLVER>::ProblemConstructorRoundingSolver;

   virtual void Iterate(LpControl c)
   {
      if(c.computePrimal) {
         this->RegisterPrimal();
         // alternatively  compute forward and backward based rounding
         if(cur_primal_computation_direction_ == Direction::forward) {
            if(verbosity >= 2) { std::cout << "compute primal for forward pass\n"; }
            this->lp_.ComputeForwardPassAndPrimal(iter);
            this->lp_.ComputeBackwardPass();
            cur_primal_computation_direction_ = Direction::backward;
         } else {
            assert(cur_primal_computation_direction_ == Direction::backward);
            if(verbosity >= 2) { std::cout << "compute primal for backward pass\n"; }
            this->lp_.ComputeForwardPass();
            this->lp_.ComputeBackwardPassAndPrimal(iter);
            cur_primal_computation_direction_ = Direction::forward;
         }
      } else {
         SOLVER::Iterate(c);
      }
    ++iter;
  }

private:
   INDEX iter = 0;
   Direction cur_primal_computation_direction_ = Direction::forward; 
};





// Macro for generating main function 
// do zrobienia: get version number automatically from CMake 
// do zrobienia: version number for individual solvers?
#define LP_MP_CONSTRUCT_SOLVER_WITH_INPUT_AND_VISITOR(FMC,PARSE_PROBLEM_FUNCTION,VISITOR) \
using namespace LP_MP; \
int main(int argc, char* argv[]) \
{ \
   Solver<FMC,LP,VISITOR> solver(argc,argv); \
   solver.ReadProblem(PARSE_PROBLEM_FUNCTION); \
   return solver.Solve(); \
}

#define LP_MP_CONSTRUCT_SOLVER_WITH_INPUT_AND_VISITOR_MP_ROUNDING(FMC,PARSE_PROBLEM_FUNCTION,VISITOR) \
using namespace LP_MP; \
int main(int argc, char* argv[]) \
{ \
   MpRoundingSolver<Solver<FMC,LP,VISITOR>> solver(argc,argv); \
   solver.ReadProblem(PARSE_PROBLEM_FUNCTION); \
   return solver.Solve(); \
}

// Macro for generating main function with specific primal rounding solver
#define LP_MP_CONSTRUCT_SOLVER_WITH_INPUT_VISITOR_AND_SOLVER(FMC,PARSE_PROBLEM_FUNCTION,VISITOR,SOLVER) \
using namespace LP_MP; \
int main(int argc, char* argv[]) \
{ \
   MpRoundingSolver<FMC,LP,VISITOR> solver(argc,argv); \
   solver.ReadProblem(PARSE_PROBLEM_FUNCTION); \
   return solver.Solve(); \
}

// Macro for generating main function with specific solver with additional LP option
#define LP_MP_CONSTRUCT_SOLVER_WITH_INPUT_AND_VISITOR_LP(FMC,PARSE_PROBLEM_FUNCTION,VISITOR,LPSOLVER) \
using namespace LP_MP; \
int main(int argc, char* argv[]) \
{ \
   VisitorSolver<LpSolver<MpRoundingSolver<FMC>,LPSOLVER>,VISITOR> solver(argc,argv); \
   solver.ReadProblem(PARSE_PROBLEM_FUNCTION); \
   return solver.Solve(); \
}

// Macro for generating main function with specific solver with SAT based rounding
#define LP_MP_CONSTRUCT_SOLVER_WITH_INPUT_AND_VISITOR_SAT(FMC,PARSE_PROBLEM_FUNCTION,VISITOR) \
using namespace LP_MP; \
int main(int argc, char* argv[]) \
{ \
   MpRoundingSolver<Solver<FMC,LP_sat<LP>,VISITOR>> solver(argc,argv); \
   solver.ReadProblem(PARSE_PROBLEM_FUNCTION<Solver<FMC,LP_sat<LP>,VISITOR>>); \
   return solver.Solve(); \
}

// Macro for generating main function with specific solver with SAT based rounding
#define LP_MP_CONSTRUCT_SOLVER_WITH_INPUT_AND_VISITOR_SAT_CONCURRENT(FMC,PARSE_PROBLEM_FUNCTION,VISITOR) \
using namespace LP_MP; \
int main(int argc, char* argv[]) \
{ \
   MpRoundingSolver<Solver<FMC,LP_sat<LP_concurrent<LP>>,VISITOR>> solver(argc,argv); \
   solver.ReadProblem(PARSE_PROBLEM_FUNCTION<Solver<FMC,LP_sat<LP_concurrent<LP>>,VISITOR>>); \
   return solver.Solve(); \
}


} // end namespace LP_MP

#endif // LP_MP_SOLVER_HXX

