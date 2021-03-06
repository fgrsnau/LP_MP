#ifndef LP_MP_LP_FWMAP_HXX
#define LP_MP_LP_FWMAP_HXX

#include "tree_decomposition.hxx"
#include "FW-MAP.h"

namespace LP_MP {

class LP_tree_FWMAP : public LP_with_trees<Lagrangean_factor_FWMAP, LP_tree_FWMAP> {
public:
   // for the Frank Wolfe implementation
   // to do: change the FWMAP implementation and make these methods virtual instead of static.
   // _y is the primal labeling to be computed
   // wi is the Lagrangean variables
   static double max_fn(double* wi, FWMAP::YPtr _y, FWMAP::TermData term_data)
   {
     
      LP_tree_Lagrangean<Lagrangean_factor_FWMAP>* t = (LP_tree_Lagrangean<Lagrangean_factor_FWMAP>*) term_data;

      // first add weights to problem
      // we only need to add Lagrange variables to Lagrangean_factors_ (others are not shared)
      t->add_weights(wi, +1.0);

      // compute optimal labeling
      t->solve();
      // store primal solution in archive
      t->save_primal(_y);

      // remove weights again
      t->add_weights(wi, -1.0);

      return t->primal_cost();
   }

   // copy values provided by subgradient from Lagrangean factors into ai
   static void copy_fn(double* ai, FWMAP::YPtr _y, FWMAP::TermData term_data)
   {
      LP_tree_Lagrangean<Lagrangean_factor_FWMAP>* t = (LP_tree_Lagrangean<Lagrangean_factor_FWMAP>*) term_data;

      // possibly this is not needed anymore
      std::fill(ai, ai+t->dual_size(), double(0.0));

      // read in primal solution from which to compute subgradient
      t->read_in_primal(_y);

      for(auto L : t->Lagrangean_factors_) {
         L.copy_fn(ai);
      }
   }

   static double dot_product_fn(double* wi, FWMAP::YPtr _y, FWMAP::TermData term_data)
   {
      LP_tree_Lagrangean<Lagrangean_factor_FWMAP>* t = (LP_tree_Lagrangean<Lagrangean_factor_FWMAP>*) term_data;

      // read in primal solution
      t->read_in_primal(_y);

      double v = 0.0;
      for(auto L : t->Lagrangean_factors_) {
         v += L.dot_product_fn(wi);
      }

      return v;
   }

   /*
   void Begin()
   {
     LP_with_trees<Lagrangean_factor_FWMAP>::Begin(); 
     // add to mapping an extra dimension
     for(auto& t : trees_) {
       t.mapping().push_back(this->Lagrangean_vars_size_);
     } 
   }
   */

   void construct_decomposition()
   {
     bundle_solver = build_up_solver();
   }

   FWMAP* build_up_solver()
   {
      auto* bundle_solver = new FWMAP(this->no_Lagrangean_vars(), trees_.size(), LP_tree_FWMAP::max_fn, LP_tree_FWMAP::copy_fn, LP_tree_FWMAP::dot_product_fn);//int d, int n, MaxFn max_fn, CopyFn copy_fn, DotProductFn dot_product_fn);

      for(INDEX i=0; i<trees_.size(); ++i) {
         auto& t = trees_[i];
         const INDEX primal_size_in_bytes = t.primal_size_in_bytes();

         bundle_solver->SetTerm(i, &t, t.mapping().size(), &t.mapping()[0], t.primal_size_in_bytes()); // although mapping is of length di + 1 (the last entry being di itself, its length must be given as di!
      }

      //svm->options.gap_threshold = 0.0001;
      bundle_solver->options.iter_max = 1000000;
      bundle_solver->options.c = proximal_weight_arg_.getValue();
      bundle_solver->init();

      lb_ = -std::numeric_limits<REAL>::infinity();

      return bundle_solver;
   }

   REAL decomposition_lower_bound() const
   {
     return lb_; 
   }


public:
   LP_tree_FWMAP(TCLAP::CmdLine& cmd) 
     : LP_with_trees(cmd),
     proximal_weight_arg_("","proximalWeight","inverse weight for the proximal term", false, 1.0, "", cmd)
  {
    bundle_solver = nullptr;
  }

   ~LP_tree_FWMAP()
   {
     if(bundle_solver != nullptr) {
       delete bundle_solver;
     }
   }

   void optimize_decomposition(const INDEX iteration)
   {
      std::cout << "compute pass fw\n";
      // compute descent with quadratic term centered at current reparametrization.
      // unfortunately, the SVM solver has to be built up from scratch in every iteration

      //SVM_FW_visitor visitor({0.0,lb});
      //auto visitor_func = std::bind(&SVM_FW_visitor::visit, &visitor, std::placeholders::_1);
      //svm->options.callback_fn = visitor_func;
      double cost = bundle_solver->do_descent_step();
      lb_ = std::max(cost, lb_);
      //double* w = svm->GetLambda()
      //add_weights(w, -1.0);
      //std::cout << "after lower bound = " << this->LowerBound() << "\n";
   }

private:
  FWMAP* bundle_solver;
  REAL lb_;
  TCLAP::ValueArg<double> proximal_weight_arg_; 
};
} // namespace LP_MP

#endif // LP_MP_LP_FWMAP_HXX


